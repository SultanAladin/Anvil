#pragma once

#include "CoreMinimal.h"
#include "DeployableUnit/GroundUnit/GroundUnit.h"
#include "Components/RepulsorSpecifications.h"
#include "Controllers/Input/InputTensor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "Chaos/SimCallbackObject.h"
#include "PBDRigidsSolver.h"
#include "Chaos/ParticleHandle.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Net/UnrealNetwork.h"
#include "ArmoredGroundUnit.generated.h"

class UStaticMeshComponent;
class USpringArmComponent;
class UCameraComponent;
class AArmoredGroundUnit;
class AGroundWeaponModule;
class AImpactProjectile;

USTRUCT(BlueprintType)
struct ANVIL_API FCanonSpecs
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    bool bInfiniteAmmo = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon", meta = (ClampMin = "0.01"))
    float FireInterval_s = 1.333333f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    float ProjectileSpeed_cms = 12000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    float RecoilTravel_cm = 8.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    float RecoilReturnRate = 18.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    float RecoilPitchKick_deg = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    float RecoilYawKick_deg = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    float RecoilPitchRandom_deg = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    float RecoilYawRandom_deg = 0.25f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Canon")
    float RecoilAngularReturnRate = 12.0f;
};

//------------------------------------------------------------------------------
//                          data channel (GT↔PT sync)
//------------------------------------------------------------------------------

/** Thread-safe data channel for GT↔PT synchronization */
template<typename T>
struct TDataChannel
{
    void Write(const T& InData) { FScopeLock Lock(&CriticalSection); Data = InData; bHasData = true; }
    bool Read(T& OutData) { FScopeLock Lock(&CriticalSection); if (bHasData) { OutData = Data; bHasData = false; return true; } return false; }
    bool Peek(T& OutData) const { FScopeLock Lock(&CriticalSection); if (bHasData) { OutData = Data; return true; } return false; }

private:
    mutable FCriticalSection CriticalSection;
    T Data;
    bool bHasData = false;
};

//------------------------------------------------------------------------------
//                          repulsor solver input (GT→PT)
//------------------------------------------------------------------------------

/** Physics callback input data */
struct FRepulsorSolverInput
{
    TArray<FRepulsorSpecifications> RepulsorSpecs;  // [-] - Repulsor configuration
    FTransform HullTransform;  // [cm,rad] - Current hull world transform
    FVector WorldCoM = FVector::ZeroVector;
    TArray<float> MeasuredAltitudes;  // [cm] - GT trace results per repulsor
    TArray<FVector> HitNormals;  // [-] - Surface normals from traces (for slope conformance)

    //--------------------------------------------------------------------------
    //                          PPU commands
    //--------------------------------------------------------------------------
    float Throttle_CMD = 0.0f;  // [-1..1] - Longitudinal thrust
    float Lateral_CMD = 0.0f;  // [-1..1] - Lateral strafe
    float MouseYaw_CMD = 0.0f;  // [-1..1] - Mouse X (yaw)
    float MousePitch_CMD = 0.0f;  // [-1..1] - Mouse Y (pitch)
    bool bVerticalThrust = false;  // [-] - Height boost active
    bool bBoostActive = false;  // [-] - Override power engaged

    //--------------------------------------------------------------------------
    //                          Height Boost (replaces force-based lift)
    //--------------------------------------------------------------------------
    float Lift_Capacitor = 1.0f;  // [0..1] - Current charge (1.0 = full)
    float BaseDesiredHeight_cm = 150.0f;  // [cm] - Normal hover height
    float MaxHeightBoost_cm = 300.0f;  // [cm] - Max additional height when boosting
    float CurrentHeightOffset_cm = 0.0f;  // [cm] - Current boost offset (GT synced)

    //--------------------------------------------------------------------------
    //                          propulsion dynamics
    //--------------------------------------------------------------------------
    float MaxThrustForce_N = 50000.0f;  // [N] - Base thrust (before mass scaling)
    float MaxStrafeForce_N = 30000.0f;  // [N] - Base strafe (before mass scaling)
    float MaxYawTorque_Nm = 100000.0f;  // [N·m] - Yaw authority
    float ThrusterResponseExponent = 2.0f;  // [-] - Input curve power
    float BoostMultiplier = 1.5f;  // [-] - Force multiplier when boosting

    float MaxSpeed_cms = 2500.0f;  // [cm/s] - 25 m/s speed cap for horizontal thrusters

    //--------------------------------------------------------------------------
    //                          attitude dynamics (mouse-controlled)
    //--------------------------------------------------------------------------
    float RollAngleLimit_deg = 12.0f;  // [°] - Max roll from strafe
    float PitchAngleLimit_deg = 20.0f;  // [°] - Max pitch from mouse Y
    float RollTorqueGain = 50000.0f;  // [N·m] - Roll responsiveness
    float PitchTorqueGain = 50000.0f;  // [N·m] - Pitch responsiveness
    float YawTorqueGain = 80000.0f;  // [N·m] - Yaw responsiveness (mouse X)

    //--------------------------------------------------------------------------
    //                          VRC (Velocity Retention Control)
    //--------------------------------------------------------------------------
    float VRC_Coefficient = 0.8f;  // [s⁻¹] - Velocity decay rate when idle

    //--------------------------------------------------------------------------
    //                          mass cache (computed at init)
    //--------------------------------------------------------------------------
    float CachedMass_kg = 1000.0f;  // [kg] - Vehicle mass for force scaling
    float MassNormalizedAccel_mss = 15.0f;  // [m⋅s⁻²] - Target accel (reduced from 62.5)
};

//------------------------------------------------------------------------------
//                          repulsor solver output (PT→GT)
//------------------------------------------------------------------------------

/** Physics callback output data */
struct FRepulsorSolverOutput
{
    TArray<FRepulsorStateVector> RepulsorStates;  // [-] - Per-repulsor state
    float TotalVerticalForce_N = 0.0f;  // [N]
    float AverageAltitude_cm = 0.0f;  // [cm]
    bool bGrounded = false;  // [-]
    FVector CurrentVelocity_cms = FVector::ZeroVector;  // [cm/s] - For telemetry
    float CurrentRoll_deg = 0.0f;  // [°] - Hull roll angle
    float CurrentPitch_deg = 0.0f;  // [°] - Hull pitch angle
    float AppliedThrustForce_N = 0.0f;  // [N] - Debug: actual thrust applied
    float AppliedStrafeForce_N = 0.0f;  // [N] - Debug: actual strafe applied
    float AppliedLiftForce_N = 0.0f;  // [N] - Debug: vertical thrust applied
    FVector ThrustForceVec = FVector::ZeroVector;  // [N] - Debug: thrust vector
    FVector StrafeForceVec = FVector::ZeroVector;  // [N] - Debug: strafe vector
    FVector LiftForceVec = FVector::ZeroVector;  // [N] - Debug: lift vector
};

//------------------------------------------------------------------------------
//                          repulsor solver callback (physics thread)
//------------------------------------------------------------------------------

/** Chaos physics callback for repulsor force computation (UE 5.7+) */
class FRepulsorSolverCallback : public Chaos::TSimCallbackObject<Chaos::FSimCallbackNoInput, Chaos::FSimCallbackNoOutput, Chaos::ESimCallbackOptions::Presimulate>
{
public:
    Chaos::FPhysicsSolverBase* PhysicsSolver = nullptr;
    Chaos::FRigidBodyHandle_Internal* BodyHandle = nullptr;
    AArmoredGroundUnit* Owner = nullptr;

    TDataChannel<FRepulsorSolverInput> InputDataChannel;  // GT→PT
    TDataChannel<FRepulsorSolverOutput> OutputDataChannel;  // PT→GT

    FRepulsorSolverInput CurrentInput;
    FRepulsorSolverOutput CurrentOutput;

    void Initialize(Chaos::FPhysicsSolverBase* InSolver, Chaos::FRigidBodyHandle_Internal* InBodyHandle);
    void SetOwner(AArmoredGroundUnit* InOwner) { Owner = InOwner; }

    virtual void OnPreSimulate_Internal() override;
    void SolveRepulsorForces(float DeltaTime);
    void SolvePropulsionForces(float DeltaTime);
    void SolveVerticalThrust(float DeltaTime);
    void SolveVRC(float DeltaTime);
    void SolveAttitudeControl(float DeltaTime);
    void SolveRollDynamics(float DeltaTime);  // Deprecated - now handled by spring differential
};

//------------------------------------------------------------------------------
//                          armored ground unit
//------------------------------------------------------------------------------

DECLARE_LOG_CATEGORY_EXTERN(LogArmoredGroundUnit, Log, All);

UCLASS(BlueprintType, Blueprintable)
class ANVIL_API AArmoredGroundUnit : public AGroundUnit
{
    GENERATED_BODY()

    //--------------------------------------------------------------------------
    //                          core
    //--------------------------------------------------------------------------
public:
    AArmoredGroundUnit();

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
    virtual void Tick(float DeltaTime) override;
    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    const FRepulsorSolverOutput& GetLatestRepulsorOutput() const { return LatestRepulsorOutput; }
    float GetTurretYaw_deg() const { return CurrentTurretYaw_deg; }
    float GetOrdnancePitch_deg() const { return CurrentOrdnancePitch_deg; }
    FVector GetCachedAimPoint() const { return CachedAimPoint; }
    FText GetActiveWeaponText() const;
    bool IsCannonAimValid() const { return bCannonAimValid; }

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

    //--------------------------------------------------------------------------
    //                          hull component
    //--------------------------------------------------------------------------
public:
    /** Hull geometry mesh (root, physics simulated) */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components|Hull")
    UStaticMeshComponent* HullGeometry;

    //--------------------------------------------------------------------------
    //                          turret components
    //--------------------------------------------------------------------------
public:
    /** Turret base - rotates on Z-axis (yaw), attached to hull */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components|Turret")
    UStaticMeshComponent* TurretBase;

    /** Turret module - hosts weapon systems, attached to turret base */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components|Turret")
    UStaticMeshComponent* TurretModule;

    /** Ordnance tube - rotates on Y-axis (pitch), attached to turret module */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components|Turret")
    UStaticMeshComponent* OrdnanceTube;

    //--------------------------------------------------------------------------
    //                          camera components
    //--------------------------------------------------------------------------
public:
    /** Camera arm */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components|Camera")
    USpringArmComponent* CameraArm;

    /** Main camera */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components|Camera")
    UCameraComponent* Camera;

    //--------------------------------------------------------------------------
    //                          repulsor configuration
    //--------------------------------------------------------------------------
public:
    /** Socket prefix for repulsor discovery */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Repulsor|Configuration")
    FString RepulsorSocketPrefix = TEXT("Repulsor_");

    /** Base repulsor specification */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Repulsor|Configuration")
    FRepulsorSpecifications BaseRepulsorSpec;

    /** Discovered repulsor array */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Repulsor|Runtime")
    TArray<FRepulsorSpecifications> RepulsorArray;

    //--------------------------------------------------------------------------
    //                          propulsion base
    //--------------------------------------------------------------------------
public:
    /** Maximum forward thrust force (base, before mass scaling) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Base", meta = (ClampMin = "1000.0"))
    float MaxThrustForce_N = 50000.0f;  // [N]

    /** Maximum lateral strafe force (base, before mass scaling) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Base", meta = (ClampMin = "1000.0"))
    float MaxStrafeForce_N = 30000.0f;  // [N]

    /** Maximum yaw torque */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Base", meta = (ClampMin = "1000.0"))
    float MaxYawTorque_Nm = 100000.0f;  // [N·m]

    //--------------------------------------------------------------------------
    //                          propulsion dynamics
    //--------------------------------------------------------------------------
public:
    /** Target acceleration for mass-normalized speed (lower = slower) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Dynamics", meta = (ClampMin = "1.0", ClampMax = "100.0"))
    float MassNormalizedAccel_mss = 15.0f;  // [m⋅s⁻²] - Reduced from 62.5

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Dynamics", meta = (ClampMin = "0.0"))
    float MaxThrusterSpeed_mps = 25.0f;

    /** Thruster response curve exponent (2.0 = quadratic, snappier feel) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Dynamics", meta = (ClampMin = "1.0", ClampMax = "4.0"))
    float ThrusterResponseExponent = 2.0f;  // [-]

    //--------------------------------------------------------------------------
    //                          roll dynamics
    //--------------------------------------------------------------------------
public:
    /** Maximum roll angle during lateral acceleration */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Roll", meta = (ClampMin = "0.0", ClampMax = "30.0"))
    float RollAngleLimit_deg = 8.0f;  // [°]

    /** Maximum pitch angle from mouse Y input */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Attitude", meta = (ClampMin = "0.0", ClampMax = "30.0"))
    float PitchAngleLimit_deg = 17.0f;  // [°] - Mouse Y controls pitch

    /** Roll torque responsiveness (higher = faster tilt) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Roll", meta = (ClampMin = "1000.0"))
    float RollTorqueGain = 50000.0f;  // [N·m] - Increased from 5000

    /** Pitch torque responsiveness (higher = faster tilt) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Roll", meta = (ClampMin = "1000.0"))
    float PitchTorqueGain = 30000.0f;  // [N·m] - Increased from 3000

    //--------------------------------------------------------------------------
    //                          boost system
    //--------------------------------------------------------------------------
public:
    /** Boost force multiplier */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Boost", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    float BoostMultiplier = 1.5f;  // [-]

    /** Boost duration before cooldown */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Boost", meta = (ClampMin = "0.5"))
    float BoostDuration_s = 3.0f;  // [s]

    /** Boost cooldown time */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|Boost", meta = (ClampMin = "0.5"))
    float BoostCooldown_s = 5.0f;  // [s]

protected:
    float BoostRemainingTime_s = 0.0f;  // [s] - Current boost reserve
    float BoostCooldownTimer_s = 0.0f;  // [s] - Cooldown remaining
    bool bBoostActive = false;  // [-] - Currently boosting

    //--------------------------------------------------------------------------
    //                          Height Boost (Lift Capacitor powered)
    //--------------------------------------------------------------------------
public:
    /** Maximum additional hover height when vertical thrust active */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|HeightBoost", meta = (ClampMin = "50.0", ClampMax = "500.0"))
    float MaxHeightBoost_cm = 300.0f;  // [cm]

    /** Rate of height change when boosting (cm/s) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|HeightBoost", meta = (ClampMin = "50.0"))
    float HeightBoostRate_cmps = 200.0f;  // [cm/s]

    /** Rate at which capacitor discharges while vertical thrust active */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|HeightBoost", meta = (ClampMin = "0.01"))
    float LiftCapacitor_DischargeRate = 0.05f;  // [s⁻¹] ~20 sec to empty

    /** Rate at which capacitor recharges when not in use */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|HeightBoost", meta = (ClampMin = "0.01"))
    float LiftCapacitor_RechargeRate = 0.033f;  // [s⁻¹] ~30 sec to full

    /** Current Lift Capacitor charge (0.0 - 1.0) */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Propulsion|HeightBoost")
    float Lift_Capacitor = 1.0f;  // [0..1] normalized

    /** Current height offset from boost (0 to MaxHeightBoost_cm) */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Propulsion|HeightBoost")
    float CurrentHeightOffset_cm = 0.0f;  // [cm]

    //--------------------------------------------------------------------------
    //                          VRC (Velocity Retention Control)
    //--------------------------------------------------------------------------
public:
    /** Velocity decay coefficient when thrusters idle (higher = faster stop) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Propulsion|VRC", meta = (ClampMin = "0.1", ClampMax = "5.0"))
    float VRC_Coefficient = 0.8f;  // [s⁻¹]

    //--------------------------------------------------------------------------
    //                          debug visualization
    //--------------------------------------------------------------------------
public:
    /** Enable debug visualization for forces and traces */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bDrawDebug = true;

    /** Force arrow scale (cm per Newton) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta = (ClampMin = "0.001", ClampMax = "1.0"))
    float DebugForceScale = 0.01f;  // [cm/N]

    /** Cached surface normals from traces for debug visualization */
    TArray<FVector> CachedHitNormals;

    //--------------------------------------------------------------------------
    //                          turret control (DISABLED)
    //--------------------------------------------------------------------------
    // TODO: Re-enable after movement system is finalized
public:
    /** Turret yaw rotation speed */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Turret", meta = (ClampMin = "1.0", ClampMax = "180.0"))
    float TurretYawRate_degps = 45.0f;  // [°/s]

    /** Ordnance tube pitch rotation speed */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Turret", meta = (ClampMin = "1.0", ClampMax = "90.0"))
    float OrdnancePitchRate_degps = 30.0f;  // [°/s]

    /** Ordnance tube minimum pitch (depression) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Turret", meta = (ClampMin = "0.0", ClampMax = "90.0"))
    float OrdnanceMinPitch_deg = 0.0f;  // [°]

    /** Ordnance tube maximum pitch (elevation) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Turret", meta = (ClampMin = "0.0", ClampMax = "90.0"))
    float OrdnanceMaxPitch_deg = 45.0f;  // [°]

protected:
    /** Current turret azimuth (debug visible) */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Turret|Debug")
    float CurrentTurretYaw_deg = 0.0f;  // [°]

    /** Current tube elevation (debug visible) */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Turret|Debug")
    float CurrentOrdnancePitch_deg = 0.0f;  // [°]

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Armament")
    TSubclassOf<AImpactProjectile> ProjectileClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Armament")
    FCanonSpecs CanonSpecs;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Armament")
    int32 CannonAmmo = 200;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Armament")
    float VehicleProjectileSpeed_cms = 12000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Armament")
    FName VehicleMuzzleSocketName = TEXT("WeaponOut");

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Armament")
    int32 ActiveWeaponSlot = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Armament")
    TArray<TObjectPtr<AGroundWeaponModule>> MountedWeaponModules;

    float VehicleWeaponCooldown_s = 0.0f;
    FVector CachedAimPoint = FVector::ZeroVector;

    bool bCannonAimValid = true;

    FVector OrdnanceBaseLocalPos = FVector::ZeroVector;
    float CanonRecoilSlide_cm = 0.0f;
    float CanonRecoilPitchOffset_deg = 0.0f;
    float CanonRecoilYawOffset_deg = 0.0f;
    int32 CannonShotCounter = 0;

    //--------------------------------------------------------------------------
    //                          input tensor
    //--------------------------------------------------------------------------
public:
    /** Current input state */
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Input")
    FInputTensor InputTensor;

    //--------------------------------------------------------------------------
    //                          physics state
    //--------------------------------------------------------------------------
protected:
    FPhysScene* PhysicsScene = nullptr;
    FRepulsorSolverCallback* RepulsorCallback = nullptr;
    FRepulsorSolverOutput LatestRepulsorOutput;
    float CachedMass_kg = 1000.0f;  // [kg] - Cached at physics init

    //--------------------------------------------------------------------------
    //                          network replication
    //--------------------------------------------------------------------------
public:
    UPROPERTY(Replicated)
    FVector ReplicatedLocation;

    UPROPERTY(Replicated)
    FRotator ReplicatedRotation;

    //--------------------------------------------------------------------------
    //                          repulsor system methods
    //--------------------------------------------------------------------------
public:
    void DiscoverRepulsorSockets();
    void CalibrateRepulsorAssembly();
    bool ComputeRepulsorSprungMasses(const TArray<FVector>& LocalPositions, const FVector& LocalCoM, float TotalMass, TArray<float>& OutMasses);
    void ApplyEquilibriumTransform();

protected:
    bool bPhysicsInitialized = false;
    bool InitializePhysicsCallback();
    void ShutdownPhysicsCallback();
    void PublishInputToPhysicsThread();
    void ReadStateFromPhysicsThread();
    void UpdateBoostState(float DeltaTime);
    void UpdateLiftCapacitor(float DeltaTime);
    void DrawDebugVisualization();
    void UpdateTurretAiming(float DeltaTime);
    void HandleWeaponControl(float DeltaTime);

public:
    void RegisterWeaponModule(AGroundWeaponModule* Module);

    FName ChooseWeaponPortSocket(UStaticMeshComponent* ParentComp, FName PreferredSocket) const;

    //--------------------------------------------------------------------------
    //                          turret control methods (DISABLED)
    //--------------------------------------------------------------------------
public:
    /** Update turret and ordnance tube rotation from input - DISABLED */
    // void UpdateTurretAiming(float DeltaTime);
};
