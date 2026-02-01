//==============================================================================
// ArmoredGroundUnit.cpp - Hover Tank Implementation
// Physics-thread repulsor simulation with Battlezone-style movement
//==============================================================================

#include "ArmoredGroundUnit.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Controllers/UnitController/UnitController.h"
#include "DeployableUnit/GroundUnit/GroundWeaponModule.h"
#include "DeployableUnit/GroundUnit/ImpactProjectile.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Math/RotationMatrix.h"

DEFINE_LOG_CATEGORY(LogArmoredGroundUnit);

//==============================================================================
//                          repulsor solver callback
//==============================================================================

void FRepulsorSolverCallback::Initialize(Chaos::FPhysicsSolverBase* InSolver, Chaos::FRigidBodyHandle_Internal* InBodyHandle)
{
    PhysicsSolver = InSolver;
    BodyHandle = InBodyHandle;
}

void FRepulsorSolverCallback::OnPreSimulate_Internal()
{
    if (!BodyHandle || !Owner) { return; }

    InputDataChannel.Read(CurrentInput);
    const float DeltaTime = GetDeltaTime_Internal();
    
    SolveRepulsorForces(DeltaTime);
    SolvePropulsionForces(DeltaTime);
    SolveAttitudeControl(DeltaTime);
    SolveVRC(DeltaTime);
    
    OutputDataChannel.Write(CurrentOutput);
}

//------------------------------------------------------------------------------
// repulsor force computation (GRIT-style with proper torque)
//------------------------------------------------------------------------------
void FRepulsorSolverCallback::SolveRepulsorForces(float DeltaTime)
{
    if (!BodyHandle) { return; }

    const Chaos::FVec3 BodyVelocity = BodyHandle->GetV();
    const Chaos::FVec3 BodyAngularVel_Radps = BodyHandle->GetW();
    const Chaos::FRotation3 BodyRotation = BodyHandle->R();
    const FVector BodyUpVector = BodyRotation.RotateVector(FVector(0, 0, 1));
    
    const FVector WorldCoM = CurrentInput.WorldCoM;

    CurrentOutput.RepulsorStates.SetNum(CurrentInput.RepulsorSpecs.Num());
    CurrentOutput.TotalVerticalForce_N = 0.0f;
    float TotalAltitude = 0.0f;
    int32 GroundedCount = 0;

    // Effective desired height (base + height boost offset)
    const float EffectiveDesiredHeight = CurrentInput.BaseDesiredHeight_cm + CurrentInput.CurrentHeightOffset_cm;

    // Accumulators for force and torque (GRIT-style)
    FVector TotalForce_UU = FVector::ZeroVector;
    FVector TotalTorque_UU = FVector::ZeroVector;

    //--------------------------------------------------------------------------
    // repulsor spring-damper force computation with lever-arm torque
    //--------------------------------------------------------------------------
    for (int32 i = 0; i < CurrentInput.RepulsorSpecs.Num(); ++i)
    {
        FRepulsorSpecifications Spec = CurrentInput.RepulsorSpecs[i];
        Spec.DesiredHeight_cm = EffectiveDesiredHeight;
        
        FRepulsorStateVector& State = CurrentOutput.RepulsorStates[i];

        const FVector WorldMount = CurrentInput.HullTransform.TransformPosition(Spec.LocalMountPoint);
        State.WorldMountPoint = WorldMount;

        const FVector LinearVel_Cms(BodyVelocity.X, BodyVelocity.Y, BodyVelocity.Z);
        const FVector AngularVel_Radps(BodyAngularVel_Radps.X, BodyAngularVel_Radps.Y, BodyAngularVel_Radps.Z);
        const FVector LeverArm = WorldMount - WorldCoM;
        const FVector MountVelocity_Cms = LinearVel_Cms + FVector::CrossProduct(AngularVel_Radps, LeverArm);
        
        // Get ground normal from traces (use world up as fallback for stability)
        FVector GroundNormal = FVector::UpVector;  // Default to world up
        if (CurrentInput.HitNormals.IsValidIndex(i) && !CurrentInput.HitNormals[i].IsNearlyZero())
        {
            GroundNormal = CurrentInput.HitNormals[i].GetSafeNormal();
        }
        State.GroundNormal = GroundNormal;
        
        // Vertical velocity component for damping
        State.VerticalVelocity_cms = FVector::DotProduct(MountVelocity_Cms, BodyUpVector);

        if (CurrentInput.MeasuredAltitudes.IsValidIndex(i)) { State.CurrentAltitude_cm = CurrentInput.MeasuredAltitudes[i]; }
        else { State.CurrentAltitude_cm = Spec.MaxTraceRange_cm; }

        State.bGroundDetected = State.CurrentAltitude_cm < Spec.MaxTraceRange_cm;

        if (State.bGroundDetected)
        {
            // Compute spring-damper force (N)
            State.OutputForce_N = Spec.ComputeForce(State.CurrentAltitude_cm, State.VerticalVelocity_cms);
            State.OutputForce_N = FMath::Max(0.0f, State.OutputForce_N);  // No pulling forces
            
            // Force vector along GROUND NORMAL (for slope conformance)
            const FVector ForceVector_N = GroundNormal * State.OutputForce_N;
            const FVector ForceVector_UU = ForceVector_N * 100.0f;  // Newton to Unreal units
            
            // Torque from this repulsor (CrossProduct gives rotation direction)
            const FVector Torque_UU = FVector::CrossProduct(LeverArm, ForceVector_UU);
            
            // Accumulate
            TotalForce_UU += ForceVector_UU;
            TotalTorque_UU += Torque_UU;
            
            CurrentOutput.TotalVerticalForce_N += State.OutputForce_N;
            TotalAltitude += State.CurrentAltitude_cm;
            GroundedCount++;
        }
        else { State.OutputForce_N = 0.0f; }

        State.PreviousAltitude_cm = State.CurrentAltitude_cm;
    }

    //--------------------------------------------------------------------------
    // apply accumulated force and torque (GRIT-style)
    //--------------------------------------------------------------------------
    if (TotalForce_UU.SizeSquared() > UE_SMALL_NUMBER)
    {
        BodyHandle->AddForce(Chaos::FVec3(TotalForce_UU.X, TotalForce_UU.Y, TotalForce_UU.Z));
    }
    if (TotalTorque_UU.SizeSquared() > UE_SMALL_NUMBER)
    {
        BodyHandle->AddTorque(Chaos::FVec3(TotalTorque_UU.X, TotalTorque_UU.Y, TotalTorque_UU.Z));
    }

    //--------------------------------------------------------------------------
    // aggregate state
    //--------------------------------------------------------------------------
    if (GroundedCount > 0)
    {
        CurrentOutput.AverageAltitude_cm = TotalAltitude / GroundedCount;
        CurrentOutput.bGrounded = (GroundedCount >= 1);
    }
    else
    {
        CurrentOutput.AverageAltitude_cm = 0.0f;
        CurrentOutput.bGrounded = false;
    }

    // Store velocity for telemetry
    CurrentOutput.CurrentVelocity_cms = FVector(BodyVelocity.X, BodyVelocity.Y, BodyVelocity.Z);
    
    // Store current angles
    const FRotator CurrentRot = BodyRotation.Rotator();
    CurrentOutput.CurrentRoll_deg = CurrentRot.Roll;
    CurrentOutput.CurrentPitch_deg = CurrentRot.Pitch;
}

//------------------------------------------------------------------------------
// propulsion force computation (horizontal only)
//------------------------------------------------------------------------------
void FRepulsorSolverCallback::SolvePropulsionForces(float DeltaTime)
{
    if (!BodyHandle) { return; }

    const Chaos::FVec3 BodyVel = BodyHandle->GetV();
    const Chaos::FRotation3 BodyRotation = BodyHandle->R();
    const FVector ForwardDir = BodyRotation.RotateVector(FVector(1, 0, 0));
    const FVector RightDir = BodyRotation.RotateVector(FVector(0, 1, 0));

    FVector GroundNormal = FVector::UpVector;
    if (CurrentOutput.RepulsorStates.Num() > 0)
    {
        FVector NormalSum = FVector::ZeroVector;
        int32 NormalCount = 0;
        for (const FRepulsorStateVector& State : CurrentOutput.RepulsorStates)
        {
            if (State.bGroundDetected && !State.GroundNormal.IsNearlyZero()) { NormalSum += State.GroundNormal; ++NormalCount; }
        }
        if (NormalCount > 0) { GroundNormal = (NormalSum / static_cast<float>(NormalCount)).GetSafeNormal(); }
    }

    // Mass-relative force calculation
    const float MassKg = FMath::Max(CurrentInput.CachedMass_kg, 100.0f);
    const float MaxForce_N = MassKg * CurrentInput.MassNormalizedAccel_mss;

    // Exponential response curve
    const float ThrottleRaw = CurrentInput.Throttle_CMD;
    const float LateralRaw = CurrentInput.Lateral_CMD;
    const float Exponent = FMath::Max(CurrentInput.ThrusterResponseExponent, 1.0f);

    const float ThrottleMag = FMath::Sign(ThrottleRaw) * FMath::Pow(FMath::Abs(ThrottleRaw), Exponent);
    const float LateralMag = FMath::Sign(LateralRaw) * FMath::Pow(FMath::Abs(LateralRaw), Exponent);

    // Boost multiplier
    const float BoostMult = CurrentInput.bBoostActive ? CurrentInput.BoostMultiplier : 1.0f;

    // Apply thrust forces (project to horizontal plane)
    const float ThrustForce_N = ThrottleMag * MaxForce_N * BoostMult;
    const float StrafeForce_N = LateralMag * MaxForce_N * 0.6f * BoostMult;

    FVector ThrustDir = ForwardDir - GroundNormal * FVector::DotProduct(ForwardDir, GroundNormal);
    FVector StrafeDir = RightDir - GroundNormal * FVector::DotProduct(RightDir, GroundNormal);
    
    if (ThrustDir.SizeSquared() > UE_SMALL_NUMBER) { ThrustDir.Normalize(); }
    if (StrafeDir.SizeSquared() > UE_SMALL_NUMBER) { StrafeDir.Normalize(); }

    FVector ThrustVec = ThrustDir * ThrustForce_N;
    FVector StrafeVec = StrafeDir * StrafeForce_N;

    const float MaxSpeed_cms = FMath::Max(CurrentInput.MaxSpeed_cms, 0.0f);
    if (MaxSpeed_cms > 0.0f)
    {
        const FVector Velocity_cms(BodyVel.X, BodyVel.Y, BodyVel.Z);
        const FVector GroundVel_cms = Velocity_cms - GroundNormal * FVector::DotProduct(Velocity_cms, GroundNormal);
        const float Speed_cms = GroundVel_cms.Size();
        if (Speed_cms >= MaxSpeed_cms && !GroundVel_cms.IsNearlyZero())
        {
            const FVector VelDir = GroundVel_cms.GetSafeNormal();
            const float ThrustAlongVel = FVector::DotProduct(ThrustVec, VelDir);
            const float StrafeAlongVel = FVector::DotProduct(StrafeVec, VelDir);
            if (ThrustAlongVel > 0.0f) { ThrustVec -= VelDir * ThrustAlongVel; }
            if (StrafeAlongVel > 0.0f) { StrafeVec -= VelDir * StrafeAlongVel; }
        }
    }

    BodyHandle->AddForce(Chaos::FVec3(ThrustVec.X, ThrustVec.Y, ThrustVec.Z) * 100.0f);
    BodyHandle->AddForce(Chaos::FVec3(StrafeVec.X, StrafeVec.Y, StrafeVec.Z) * 100.0f);

    // Debug output
    CurrentOutput.AppliedThrustForce_N = ThrustVec.Size();
    CurrentOutput.AppliedStrafeForce_N = StrafeVec.Size();
    CurrentOutput.ThrustForceVec = ThrustVec;
    CurrentOutput.StrafeForceVec = StrafeVec;
}

//------------------------------------------------------------------------------
// attitude control (yaw from mouse, no artificial roll/pitch - slopes handled by springs)
//------------------------------------------------------------------------------
void FRepulsorSolverCallback::SolveAttitudeControl(float DeltaTime)
{
    if (!BodyHandle) { return; }

    const Chaos::FRotation3 BodyRotation = BodyHandle->R();
    const Chaos::FVec3 BodyAngularVel_Radps = BodyHandle->GetW();

    const FVector CurrentUp = BodyRotation.RotateVector(FVector(0, 0, 1));
    const FVector CurrentForward = BodyRotation.RotateVector(FVector(1, 0, 0));
    const FVector CurrentRight = BodyRotation.RotateVector(FVector(0, 1, 0));
    const FVector AngularVel_Radps(BodyAngularVel_Radps.X, BodyAngularVel_Radps.Y, BodyAngularVel_Radps.Z);

    FVector GroundNormal = CurrentUp;
    if (CurrentOutput.RepulsorStates.Num() > 0)
    {
        FVector NormalSum = FVector::ZeroVector;
        int32 NormalCount = 0;
        for (const FRepulsorStateVector& State : CurrentOutput.RepulsorStates)
        {
            if (State.bGroundDetected && !State.GroundNormal.IsNearlyZero()) { NormalSum += State.GroundNormal; ++NormalCount; }
        }
        if (NormalCount > 0) { GroundNormal = (NormalSum / static_cast<float>(NormalCount)).GetSafeNormal(); }
    }

    FVector DesiredUp = GroundNormal;

    FVector BaseForward = CurrentForward - DesiredUp * FVector::DotProduct(CurrentForward, DesiredUp);
    if (BaseForward.SizeSquared() < UE_SMALL_NUMBER)
    {
        BaseForward = FVector::CrossProduct(DesiredUp, CurrentRight);
    }
    BaseForward = BaseForward.GetSafeNormal();
    FVector BaseRight = FVector::CrossProduct(DesiredUp, BaseForward).GetSafeNormal();

    const float RollInput = CurrentInput.Lateral_CMD + CurrentInput.MouseYaw_CMD;
    const float RollCmd = -FMath::Clamp(RollInput, -1.0f, 1.0f);
    const float PitchCmd = -FMath::Clamp(CurrentInput.MousePitch_CMD - CurrentInput.Throttle_CMD, -1.0f, 1.0f);
    const float TargetRoll_Rad = FMath::DegreesToRadians(CurrentInput.RollAngleLimit_deg * RollCmd);
    const float TargetPitch_Rad = FMath::DegreesToRadians(CurrentInput.PitchAngleLimit_deg * PitchCmd);

    const FQuat PitchQuat(BaseRight, TargetPitch_Rad);
    FVector DesiredForward = PitchQuat.RotateVector(BaseForward);
    DesiredUp = PitchQuat.RotateVector(DesiredUp);

    const FQuat RollQuat(DesiredForward, TargetRoll_Rad);
    DesiredUp = RollQuat.RotateVector(DesiredUp);

    DesiredForward = (DesiredForward - DesiredUp * FVector::DotProduct(DesiredForward, DesiredUp)).GetSafeNormal();
    const FQuat CurrentQuat = FRotationMatrix::MakeFromXZ(CurrentForward, CurrentUp).ToQuat();
    const FQuat DesiredQuat = FRotationMatrix::MakeFromXZ(DesiredForward, DesiredUp).ToQuat();

    FQuat ErrorQuat = DesiredQuat * CurrentQuat.Inverse();
    ErrorQuat.Normalize();

    FVector ErrorAxis = FVector::ZeroVector;
    float ErrorAngle_Rad = 0.0f;
    ErrorQuat.ToAxisAndAngle(ErrorAxis, ErrorAngle_Rad);
    if (ErrorAngle_Rad > PI) { ErrorAngle_Rad -= 2.0f * PI; }
    const FVector ErrorRadVec = ErrorAxis * ErrorAngle_Rad;

    const FVector ErrorRadVec_RP = ErrorRadVec - DesiredUp * FVector::DotProduct(ErrorRadVec, DesiredUp);
    const FVector AngularVel_RP = AngularVel_Radps - DesiredUp * FVector::DotProduct(AngularVel_Radps, DesiredUp);

    const float Kp_NmPerRad = 0.5f * (FMath::Max(CurrentInput.RollTorqueGain, 0.0f) + FMath::Max(CurrentInput.PitchTorqueGain, 0.0f));
    const float Kd_NmPerRadps = Kp_NmPerRad * 0.35f;

    FVector AttitudeTorque_Nm = ErrorRadVec_RP * Kp_NmPerRad - AngularVel_RP * Kd_NmPerRadps;

    const float YawInput = CurrentInput.MouseYaw_CMD;
    const float YawRate_Radps = FVector::DotProduct(AngularVel_Radps, DesiredUp);
    const float YawDamp_NmPerRadps = CurrentInput.YawTorqueGain * 0.25f;
    const float YawTorque_Nm = YawInput * CurrentInput.YawTorqueGain - YawRate_Radps * YawDamp_NmPerRadps;
    AttitudeTorque_Nm += DesiredUp * YawTorque_Nm;

    const float MaxTorque_Nm = FMath::Max3(CurrentInput.RollTorqueGain, CurrentInput.PitchTorqueGain, CurrentInput.YawTorqueGain);
    AttitudeTorque_Nm = AttitudeTorque_Nm.GetClampedToMaxSize(MaxTorque_Nm);

    if (AttitudeTorque_Nm.SizeSquared() > UE_SMALL_NUMBER)
    {
        BodyHandle->AddTorque(Chaos::FVec3(AttitudeTorque_Nm.X, AttitudeTorque_Nm.Y, AttitudeTorque_Nm.Z) * 10000.0f);
    }
}

void FRepulsorSolverCallback::SolveRollDynamics(float DeltaTime)
{
    // Deprecated - slope conformance now handled by differential spring forces in SolveRepulsorForces
}

void FRepulsorSolverCallback::SolveVerticalThrust(float DeltaTime)
{
    // Height-based lift is handled in SolveRepulsorForces via CurrentHeightOffset_cm
    CurrentOutput.AppliedLiftForce_N = 0.0f;
    CurrentOutput.LiftForceVec = FVector::ZeroVector;
}

//------------------------------------------------------------------------------
// VRC (Velocity Retention Control) - damping when no input
//------------------------------------------------------------------------------
void FRepulsorSolverCallback::SolveVRC(float DeltaTime)
{
    if (!BodyHandle) { return; }

    const float ThrottleInput = CurrentInput.Throttle_CMD;
    const float LateralInput = CurrentInput.Lateral_CMD;
    const bool bThrustersIdle = FMath::Abs(ThrottleInput) < 0.05f && FMath::Abs(LateralInput) < 0.05f;
    
    if (bThrustersIdle)
    {
        const Chaos::FVec3 BodyVelocity = BodyHandle->GetV();
        const FVector Velocity_Cms(BodyVelocity.X, BodyVelocity.Y, BodyVelocity.Z);

        FVector GroundNormal = FVector::UpVector;
        if (CurrentOutput.RepulsorStates.Num() > 0)
        {
            FVector NormalSum = FVector::ZeroVector;
            int32 NormalCount = 0;
            for (const FRepulsorStateVector& State : CurrentOutput.RepulsorStates)
            {
                if (State.bGroundDetected && !State.GroundNormal.IsNearlyZero()) { NormalSum += State.GroundNormal; ++NormalCount; }
            }
            if (NormalCount > 0) { GroundNormal = (NormalSum / static_cast<float>(NormalCount)).GetSafeNormal(); }
        }

        const FVector TangentVel_Cms = Velocity_Cms - GroundNormal * FVector::DotProduct(Velocity_Cms, GroundNormal);
        const float TangentSpeedSq = TangentVel_Cms.SizeSquared();
        if (TangentSpeedSq > 100.0f)
        {
            const float MassKg = FMath::Max(CurrentInput.CachedMass_kg, 100.0f);
            const float BrakeCoeff_sInv = FMath::Max(CurrentInput.VRC_Coefficient, 0.0f);
            const FVector BrakeForce_UU = -TangentVel_Cms * (MassKg * BrakeCoeff_sInv);
            BodyHandle->AddForce(Chaos::FVec3(BrakeForce_UU.X, BrakeForce_UU.Y, BrakeForce_UU.Z));
        }

        if (CurrentOutput.bGrounded)
        {
            const float MassKg = FMath::Max(CurrentInput.CachedMass_kg, 100.0f);
            const FVector Gravity_Cms2(0.0f, 0.0f, -980.0f);
            const FVector GravTangent_Cms2 = Gravity_Cms2 - GroundNormal * FVector::DotProduct(Gravity_Cms2, GroundNormal);
            FVector HoldForce_UU = -GravTangent_Cms2 * MassKg;
            HoldForce_UU = HoldForce_UU.GetClampedToMaxSize(MassKg * 980.0f);
            BodyHandle->AddForce(Chaos::FVec3(HoldForce_UU.X, HoldForce_UU.Y, HoldForce_UU.Z));
        }
    }
}

//==============================================================================
//                          armored ground unit - construction
//==============================================================================

AArmoredGroundUnit::AArmoredGroundUnit()
{
    PrimaryActorTick.bCanEverTick = true;

    HullGeometry = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("HullGeometry"));
    SetRootComponent(HullGeometry);
    HullGeometry->SetSimulatePhysics(true);
    HullGeometry->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    HullGeometry->SetCollisionObjectType(ECC_PhysicsBody);
    HullGeometry->SetLinearDamping(0.0f);
    HullGeometry->SetAngularDamping(0.0f);  // No angular damping - let springs handle stability

    TurretBase = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TurretBase"));
    TurretBase->SetupAttachment(HullGeometry);
    TurretBase->SetSimulatePhysics(false);
    TurretBase->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    TurretModule = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TurretModule"));
    TurretModule->SetupAttachment(TurretBase);
    TurretModule->SetSimulatePhysics(false);
    TurretModule->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    OrdnanceTube = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("OrdnanceTube"));
    OrdnanceTube->SetupAttachment(TurretModule);
    OrdnanceTube->SetSimulatePhysics(false);
    OrdnanceTube->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    CameraArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraArm"));
    CameraArm->SetupAttachment(HullGeometry);
    CameraArm->TargetArmLength = 800.0f;
    CameraArm->bUsePawnControlRotation = false;
    CameraArm->bDoCollisionTest = true;

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(CameraArm);

    BaseRepulsorSpec.DesiredHeight_cm = 150.0f;
    BaseRepulsorSpec.NaturalFrequency_Hz = 1.5f;
    BaseRepulsorSpec.DampingRatio = 0.7f;
    
    BoostRemainingTime_s = BoostDuration_s;
    Lift_Capacitor = 1.0f;
    CurrentHeightOffset_cm = 0.0f;
}

//==============================================================================
//                          armored ground unit - lifecycle
//==============================================================================

void AArmoredGroundUnit::BeginPlay()
{
    Super::BeginPlay();
    DiscoverRepulsorSockets();
    CalibrateRepulsorAssembly();
    if (HullGeometry) { CachedMass_kg = HullGeometry->GetMass(); }
    if (OrdnanceTube) { OrdnanceBaseLocalPos = OrdnanceTube->GetRelativeLocation(); }
    CanonRecoilSlide_cm = 0.0f;
    CanonRecoilPitchOffset_deg = 0.0f;
    CanonRecoilYawOffset_deg = 0.0f;
    bCannonAimValid = true;
    UE_LOG(LogArmoredGroundUnit, Log, TEXT("BeginPlay | Mass: %.1f kg | Repulsors: %d"), CachedMass_kg, RepulsorArray.Num());
}

void AArmoredGroundUnit::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    ShutdownPhysicsCallback();
    Super::EndPlay(EndPlayReason);
}

void AArmoredGroundUnit::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!bPhysicsInitialized)
    {
        if (InitializePhysicsCallback())
        {
            bPhysicsInitialized = true;
            if (HullGeometry) { CachedMass_kg = HullGeometry->GetMass(); }
        }
        else { return; }
    }

    UpdateBoostState(DeltaTime);
    UpdateLiftCapacitor(DeltaTime);
    PublishInputToPhysicsThread();
    ReadStateFromPhysicsThread();

    UpdateTurretAiming(DeltaTime);
    HandleWeaponControl(DeltaTime);
    
    if (bDrawDebug) { DrawDebugVisualization(); }
}

void AArmoredGroundUnit::UpdateTurretAiming(float DeltaTime)
{
    AUnitController* PC = Cast<AUnitController>(GetController());
    if (!PC || !PC->IsLocalController() || !Camera || !TurretBase || !TurretModule || !OrdnanceTube || !HullGeometry) { return; }

    int32 SizeX = 0;
    int32 SizeY = 0;
    PC->GetViewportSize(SizeX, SizeY);
    if (SizeX <= 0 || SizeY <= 0) { return; }

    const FVector2D ReticleOffset_Px = PC->GetReticleOffset_Px();
    const float ScreenX = 0.5f * static_cast<float>(SizeX) + ReticleOffset_Px.X;
    const float ScreenY = 0.5f * static_cast<float>(SizeY) + ReticleOffset_Px.Y;

    FVector WorldOrigin = FVector::ZeroVector;
    FVector WorldDir = FVector::ForwardVector;
    if (!PC->DeprojectScreenPositionToWorld(ScreenX, ScreenY, WorldOrigin, WorldDir)) { return; }

    const FVector TraceStart = Camera->GetComponentLocation();
    const FVector TraceEnd = TraceStart + WorldDir.GetSafeNormal() * 250000.0f;

    FHitResult Hit;
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(this);
    Params.bTraceComplex = false;

    const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECC_Visibility, Params);
    const FVector CameraAimPoint = bHit ? Hit.ImpactPoint : TraceEnd;

    CanonRecoilSlide_cm = FMath::FInterpTo(CanonRecoilSlide_cm, 0.0f, DeltaTime, CanonSpecs.RecoilReturnRate);
    CanonRecoilPitchOffset_deg = FMath::FInterpTo(CanonRecoilPitchOffset_deg, 0.0f, DeltaTime, CanonSpecs.RecoilAngularReturnRate);
    CanonRecoilYawOffset_deg = FMath::FInterpTo(CanonRecoilYawOffset_deg, 0.0f, DeltaTime, CanonSpecs.RecoilAngularReturnRate);

    const FVector TurretPivot = TurretBase->GetComponentLocation();
    const FVector TurretToAim = CameraAimPoint - TurretPivot;
    if (TurretToAim.SizeSquared() < UE_SMALL_NUMBER) { return; }

    const FVector HullDirLocal = HullGeometry->GetComponentTransform().InverseTransformVectorNoScale(TurretToAim).GetSafeNormal();
    const float TargetYaw_deg = FMath::RadiansToDegrees(FMath::Atan2(HullDirLocal.Y, HullDirLocal.X));
    CurrentTurretYaw_deg = FMath::FixedTurn(CurrentTurretYaw_deg, TargetYaw_deg, TurretYawRate_degps * DeltaTime);
    TurretBase->SetRelativeRotation(FRotator(0.0f, CurrentTurretYaw_deg, 0.0f));

    const FVector TubePivot = OrdnanceTube->GetComponentLocation();
    const FVector TubeToAim = CameraAimPoint - TubePivot;
    if (TubeToAim.SizeSquared() < UE_SMALL_NUMBER) { return; }

    const FVector TubeDirLocal = TurretModule->GetComponentTransform().InverseTransformVectorNoScale(TubeToAim).GetSafeNormal();
    const float UnclampedPitch_deg = FMath::RadiansToDegrees(FMath::Atan2(TubeDirLocal.Z, TubeDirLocal.X));
    const float EffectiveMinPitch_deg = FMath::Max(OrdnanceMinPitch_deg, 0.0f);
    const float ClampedPitch_deg = FMath::Clamp(UnclampedPitch_deg, EffectiveMinPitch_deg, OrdnanceMaxPitch_deg);
    bCannonAimValid = (UnclampedPitch_deg >= (EffectiveMinPitch_deg - 0.35f)) && (UnclampedPitch_deg <= (OrdnanceMaxPitch_deg + 0.35f));

    float TargetPitch_deg = ClampedPitch_deg;
    CurrentOrdnancePitch_deg = FMath::FixedTurn(CurrentOrdnancePitch_deg, TargetPitch_deg, OrdnancePitchRate_degps * DeltaTime);

    TurretBase->SetRelativeRotation(FRotator(0.0f, CurrentTurretYaw_deg + CanonRecoilYawOffset_deg, 0.0f));
    OrdnanceTube->SetRelativeRotation(FRotator(CurrentOrdnancePitch_deg + CanonRecoilPitchOffset_deg, 0.0f, 0.0f));
    OrdnanceTube->SetRelativeLocation(OrdnanceBaseLocalPos);
    OrdnanceTube->AddLocalOffset(FVector(-CanonRecoilSlide_cm, 0.0f, 0.0f));

    FTransform MuzzleXf = OrdnanceTube->GetComponentTransform();
    if (ActiveWeaponSlot <= 0)
    {
        if (OrdnanceTube->DoesSocketExist(VehicleMuzzleSocketName))
        {
            MuzzleXf = OrdnanceTube->GetSocketTransform(VehicleMuzzleSocketName);
        }
    }
    else
    {
        const int32 ModuleIndex = ActiveWeaponSlot - 1;
        if (MountedWeaponModules.IsValidIndex(ModuleIndex) && MountedWeaponModules[ModuleIndex] && MountedWeaponModules[ModuleIndex]->GunMesh)
        {
            UStaticMeshComponent* ModuleGun = MountedWeaponModules[ModuleIndex]->GunMesh;
            const FName ModuleMuzzleSocket = MountedWeaponModules[ModuleIndex]->MuzzleSocketName;
            MuzzleXf = ModuleGun->DoesSocketExist(ModuleMuzzleSocket) ? ModuleGun->GetSocketTransform(ModuleMuzzleSocket) : ModuleGun->GetComponentTransform();
        }
    }

    const FVector MuzzleStart = MuzzleXf.GetLocation();
    const FVector MuzzleForward = MuzzleXf.GetRotation().GetForwardVector();
    const float TraceDistance = 250000.0f;

    const FVector DesiredAimPoint = bCannonAimValid ? CameraAimPoint : (MuzzleStart + MuzzleForward * TraceDistance);

    FVector AimPoint = DesiredAimPoint;
    FHitResult MuzzleHit;
    if (GetWorld()->LineTraceSingleByChannel(MuzzleHit, MuzzleStart, DesiredAimPoint, ECC_Visibility, Params))
    {
        AimPoint = MuzzleHit.ImpactPoint;
    }
    CachedAimPoint = AimPoint;

    if (bDrawDebug)
    {
        DrawDebugLine(GetWorld(), TraceStart, CameraAimPoint, FColor::Green, false, 0.0f, 0, 1.5f);
        DrawDebugPoint(GetWorld(), CameraAimPoint, 10.0f, FColor::Green, false, 0.0f);
        DrawDebugLine(GetWorld(), MuzzleStart, AimPoint, FColor::Red, false, 0.0f, 0, 1.5f);
        DrawDebugPoint(GetWorld(), AimPoint, 10.0f, FColor::Red, false, 0.0f);
    }
}

void AArmoredGroundUnit::HandleWeaponControl(float DeltaTime)
{
    if (VehicleWeaponCooldown_s > 0.0f) { VehicleWeaponCooldown_s = FMath::Max(0.0f, VehicleWeaponCooldown_s - DeltaTime); }

    if (InputTensor.FCU_Station_Step)
    {
        const int32 SlotCount = 1 + MountedWeaponModules.Num();
        if (SlotCount > 0) { ActiveWeaponSlot = (ActiveWeaponSlot + 1) % SlotCount; }
    }

    if (!InputTensor.FCU_Trigger_Main) { return; }

    if (ActiveWeaponSlot <= 0)
    {
        if (VehicleWeaponCooldown_s > 0.0f) { return; }
        if (!ProjectileClass || !OrdnanceTube || !GetWorld()) { return; }
        if (!bCannonAimValid) { return; }
        if (!CanonSpecs.bInfiniteAmmo && CannonAmmo <= 0) { return; }

        FVector MuzzleLoc = OrdnanceTube->GetComponentLocation();
        FRotator MuzzleRot = OrdnanceTube->GetComponentRotation();
        if (OrdnanceTube->DoesSocketExist(VehicleMuzzleSocketName))
        {
            const FTransform MuzzleXf = OrdnanceTube->GetSocketTransform(VehicleMuzzleSocketName);
            MuzzleLoc = MuzzleXf.GetLocation();
            MuzzleRot = MuzzleXf.Rotator();
        }

        FActorSpawnParameters SpawnParams;
        SpawnParams.Owner = this;
        SpawnParams.Instigator = this;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        AImpactProjectile* Projectile = GetWorld()->SpawnActor<AImpactProjectile>(ProjectileClass, MuzzleLoc, MuzzleRot, SpawnParams);
        if (Projectile)
        {
            Projectile->SetIgnoredActor(this);
            FVector ShootDir = MuzzleRot.Vector();
            const FVector ToAim = CachedAimPoint - MuzzleLoc;
            if (ToAim.SizeSquared() > UE_SMALL_NUMBER) { ShootDir = ToAim.GetSafeNormal(); }
            const float Speed_cms = (CanonSpecs.ProjectileSpeed_cms > 0.0f) ? CanonSpecs.ProjectileSpeed_cms : VehicleProjectileSpeed_cms;
            Projectile->SetInitialVelocity(ShootDir * Speed_cms);
        }

        if (!CanonSpecs.bInfiniteAmmo) { CannonAmmo = FMath::Max(CannonAmmo - 1, 0); }

        CanonRecoilSlide_cm = FMath::Max(CanonRecoilSlide_cm, CanonSpecs.RecoilTravel_cm);
        const float PitchKick = CanonSpecs.RecoilPitchKick_deg + FMath::FRandRange(-CanonSpecs.RecoilPitchRandom_deg, CanonSpecs.RecoilPitchRandom_deg);
        const float PatternSign = (CannonShotCounter % 2 == 0) ? 1.0f : -1.0f;
        const float YawKick = (CanonSpecs.RecoilYawKick_deg * PatternSign) + FMath::FRandRange(-CanonSpecs.RecoilYawRandom_deg, CanonSpecs.RecoilYawRandom_deg);
        CanonRecoilPitchOffset_deg += PitchKick;
        CanonRecoilYawOffset_deg += YawKick;
        ++CannonShotCounter;

        VehicleWeaponCooldown_s = FMath::Max(CanonSpecs.FireInterval_s, 0.01f);
        return;
    }

    const int32 ModuleIndex = ActiveWeaponSlot - 1;
    if (MountedWeaponModules.IsValidIndex(ModuleIndex) && MountedWeaponModules[ModuleIndex])
    {
        MountedWeaponModules[ModuleIndex]->TriggerFire(CachedAimPoint);
    }
}

void AArmoredGroundUnit::RegisterWeaponModule(AGroundWeaponModule* Module)
{
    if (!Module) { return; }
    if (MountedWeaponModules.Contains(Module)) { return; }
    MountedWeaponModules.Add(Module);
    ActiveWeaponSlot = MountedWeaponModules.Num();
}

FName AArmoredGroundUnit::ChooseWeaponPortSocket(UStaticMeshComponent* ParentComp, FName PreferredSocket) const
{
    if (!ParentComp) { return PreferredSocket; }

    const FName PrimarySocket(TEXT("WeaponPort-Primary"));
    const FName SecondarySocket(TEXT("WeaponPort-Secondary"));

    auto IsOccupied = [&](FName SocketName) -> bool
    {
        for (const TObjectPtr<AGroundWeaponModule>& Module : MountedWeaponModules)
        {
            if (!Module) { continue; }
            USceneComponent* RootComp = Module->GetRootComponent();
            if (!RootComp) { continue; }
            if (RootComp->GetAttachParent() != ParentComp) { continue; }
            if (RootComp->GetAttachSocketName() == SocketName) { return true; }
        }
        return false;
    };

    const bool bIsPortPreference = PreferredSocket.IsNone() || PreferredSocket == PrimarySocket || PreferredSocket == SecondarySocket;
    if (!bIsPortPreference) { return PreferredSocket; }

    const FName First = PreferredSocket.IsNone() ? PrimarySocket : PreferredSocket;
    const FName Second = (First == PrimarySocket) ? SecondarySocket : PrimarySocket;

    if (ParentComp->DoesSocketExist(First) && !IsOccupied(First)) { return First; }
    if (ParentComp->DoesSocketExist(Second) && !IsOccupied(Second)) { return Second; }
    if (ParentComp->DoesSocketExist(First)) { return First; }
    if (ParentComp->DoesSocketExist(Second)) { return Second; }

    return PreferredSocket;
}

FText AArmoredGroundUnit::GetActiveWeaponText() const
{
    if (ActiveWeaponSlot <= 0) { return FText::FromString(TEXT("Vehicle")); }
    const int32 ModuleIndex = ActiveWeaponSlot - 1;
    if (MountedWeaponModules.IsValidIndex(ModuleIndex) && MountedWeaponModules[ModuleIndex])
    {
        return MountedWeaponModules[ModuleIndex]->GetModuleLabel();
    }
    return FText::FromString(TEXT("Module"));
}

void AArmoredGroundUnit::UpdateBoostState(float DeltaTime)
{
    if (BoostCooldownTimer_s > 0.0f)
    {
        BoostCooldownTimer_s -= DeltaTime;
        if (BoostCooldownTimer_s <= 0.0f) { BoostRemainingTime_s = BoostDuration_s; BoostCooldownTimer_s = 0.0f; }
        bBoostActive = false;
        return;
    }

    if (InputTensor.Override_Power && BoostRemainingTime_s > 0.0f)
    {
        bBoostActive = true;
        BoostRemainingTime_s -= DeltaTime;
        if (BoostRemainingTime_s <= 0.0f) { BoostRemainingTime_s = 0.0f; BoostCooldownTimer_s = BoostCooldown_s; bBoostActive = false; }
    }
    else { bBoostActive = false; }
}

void AArmoredGroundUnit::UpdateLiftCapacitor(float DeltaTime)
{
    if (InputTensor.Vertical_CMD && Lift_Capacitor > 0.0f)
    {
        Lift_Capacitor -= LiftCapacitor_DischargeRate * DeltaTime;
        Lift_Capacitor = FMath::Max(Lift_Capacitor, 0.0f);
        
        CurrentHeightOffset_cm += HeightBoostRate_cmps * DeltaTime * Lift_Capacitor;
        CurrentHeightOffset_cm = FMath::Min(CurrentHeightOffset_cm, MaxHeightBoost_cm);
    }
    else
    {
        Lift_Capacitor += LiftCapacitor_RechargeRate * DeltaTime;
        Lift_Capacitor = FMath::Min(Lift_Capacitor, 1.0f);
        
        CurrentHeightOffset_cm -= HeightBoostRate_cmps * DeltaTime * 0.5f;
        CurrentHeightOffset_cm = FMath::Max(CurrentHeightOffset_cm, 0.0f);
    }
}

//==============================================================================
//                          debug visualization (game thread)
//==============================================================================

void AArmoredGroundUnit::DrawDebugVisualization()
{
    if (!HullGeometry || !GetWorld()) { return; }

    const FVector CoM = HullGeometry->GetCenterOfMass();
    const FVector VehicleUp = HullGeometry->GetUpVector();
    const FVector VehicleForward = HullGeometry->GetForwardVector();
    const FVector VehicleRight = HullGeometry->GetRightVector();
    
    const float Scale = DebugForceScale;

    // Thrust force (green)
    if (LatestRepulsorOutput.ThrustForceVec.SizeSquared() > 1.0f)
    {
        const FVector ThrustEnd = CoM + LatestRepulsorOutput.ThrustForceVec * Scale;
        DrawDebugDirectionalArrow(GetWorld(), CoM, ThrustEnd, 25.0f, FColor::Green, false, -1.0f, 0, 3.0f);
    }

    // Strafe force (cyan)
    if (LatestRepulsorOutput.StrafeForceVec.SizeSquared() > 1.0f)
    {
        const FVector StrafeEnd = CoM + LatestRepulsorOutput.StrafeForceVec * Scale;
        DrawDebugDirectionalArrow(GetWorld(), CoM, StrafeEnd, 25.0f, FColor::Cyan, false, -1.0f, 0, 3.0f);
    }

    // Draw repulsor line traces
    for (int32 i = 0; i < RepulsorArray.Num(); ++i)
    {
        const FRepulsorSpecifications& Spec = RepulsorArray[i];
        const FVector MountLocation = HullGeometry->GetSocketLocation(Spec.RepulsorID);
        const FVector TraceEnd = MountLocation - (VehicleUp * Spec.MaxTraceRange_cm);
        
        float CurrentAlt = Spec.MaxTraceRange_cm;
        if (LatestRepulsorOutput.RepulsorStates.IsValidIndex(i)) { CurrentAlt = LatestRepulsorOutput.RepulsorStates[i].CurrentAltitude_cm; }
        
        const bool bGroundDetected = CurrentAlt < Spec.MaxTraceRange_cm;
        const FColor TraceColor = bGroundDetected ? FColor::Yellow : FColor::Red;
        const FVector HitPoint = MountLocation - (VehicleUp * CurrentAlt);
        
        DrawDebugLine(GetWorld(), MountLocation, HitPoint, TraceColor, false, -1.0f, 0, 2.0f);
        
        if (bGroundDetected)
        {
            DrawDebugSphere(GetWorld(), HitPoint, 10.0f, 8, FColor::Yellow, false, -1.0f, 0, 1.0f);
            // Draw surface normal at hit point (magenta)
            if (CachedHitNormals.IsValidIndex(i) && !CachedHitNormals[i].IsNearlyZero())
            {
                DrawDebugLine(GetWorld(), HitPoint, HitPoint + CachedHitNormals[i] * 50.0f, FColor::Magenta, false, -1.0f, 0, 2.0f);
            }
            // Draw force arrow at mount point (orange)
            const float RepulsorForce_N = LatestRepulsorOutput.RepulsorStates.IsValidIndex(i) ? LatestRepulsorOutput.RepulsorStates[i].OutputForce_N : 0.0f;
            if (RepulsorForce_N > 10.0f && CachedHitNormals.IsValidIndex(i))
            {
                const FVector ForceDir = CachedHitNormals[i].IsNearlyZero() ? FVector::UpVector : CachedHitNormals[i];
                const FVector ForceEnd = MountLocation + ForceDir * RepulsorForce_N * Scale * 0.1f;
                DrawDebugDirectionalArrow(GetWorld(), MountLocation, ForceEnd, 15.0f, FColor::Orange, false, -1.0f, 0, 2.0f);
            }
        }
    }

    // Vehicle axes at CoM (RGB = XYZ = Forward/Right/Up)
    const float AxisLen = 100.0f;
    DrawDebugLine(GetWorld(), CoM, CoM + VehicleForward * AxisLen, FColor::Red, false, -1.0f, 0, 2.0f);
    DrawDebugLine(GetWorld(), CoM, CoM + VehicleRight * AxisLen, FColor::Green, false, -1.0f, 0, 2.0f);
    DrawDebugLine(GetWorld(), CoM, CoM + VehicleUp * AxisLen, FColor::Blue, false, -1.0f, 0, 2.0f);

    // On-screen HUD
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(1, 0.0f, FColor::White, FString::Printf(TEXT("Thr: %.2f | Lat: %.2f | YawM: %.2f"), InputTensor.Throttle_CMD, InputTensor.Lateral_CMD, InputTensor.MouseYaw_CMD));
        GEngine->AddOnScreenDebugMessage(2, 0.0f, FColor::Green, FString::Printf(TEXT("Thrust: %.0f N | Strafe: %.0f N"), LatestRepulsorOutput.AppliedThrustForce_N, LatestRepulsorOutput.AppliedStrafeForce_N));
        GEngine->AddOnScreenDebugMessage(3, 0.0f, FColor::Magenta, FString::Printf(TEXT("HeightBoost: %.0f cm | Capacitor: %.0f%%"), CurrentHeightOffset_cm, Lift_Capacitor * 100.0f));
        GEngine->AddOnScreenDebugMessage(4, 0.0f, FColor::Yellow, FString::Printf(TEXT("Roll: %.1f° | Pitch: %.1f°"), LatestRepulsorOutput.CurrentRoll_deg, LatestRepulsorOutput.CurrentPitch_deg));
        GEngine->AddOnScreenDebugMessage(5, 0.0f, FColor::Cyan, FString::Printf(TEXT("Mass: %.0f kg | Grounded: %s | Altitude: %.0f cm"), CachedMass_kg, LatestRepulsorOutput.bGrounded ? TEXT("YES") : TEXT("NO"), LatestRepulsorOutput.AverageAltitude_cm));
    }
}

//==============================================================================
//                          physics callback management
//==============================================================================

bool AArmoredGroundUnit::InitializePhysicsCallback()
{
    UWorld* World = GetWorld();
    if (!World) { return false; }

    FPhysScene* PhysScene = World->GetPhysicsScene();
    if (!PhysScene) { return false; }

    PhysicsScene = PhysScene;
    if (!HullGeometry) { return false; }

    FBodyInstance* BodyInstance = HullGeometry->GetBodyInstance();
    if (!BodyInstance || !BodyInstance->ActorHandle) { return false; }

    Chaos::FRigidBodyHandle_Internal* BodyHandle = BodyInstance->ActorHandle->GetPhysicsThreadAPI();
    if (!BodyHandle) { return false; }

    RepulsorCallback = PhysicsScene->GetSolver()->CreateAndRegisterSimCallbackObject_External<FRepulsorSolverCallback>();
    if (RepulsorCallback)
    {
        RepulsorCallback->Initialize(PhysicsScene->GetSolver(), BodyHandle);
        RepulsorCallback->SetOwner(this);
        return true;
    }
    return false;
}

void AArmoredGroundUnit::ShutdownPhysicsCallback()
{
    if (RepulsorCallback && PhysicsScene)
    {
        PhysicsScene->GetSolver()->UnregisterAndFreeSimCallbackObject_External(RepulsorCallback);
        RepulsorCallback = nullptr;
    }
    bPhysicsInitialized = false;
}

void AArmoredGroundUnit::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(AArmoredGroundUnit, ReplicatedLocation);
    DOREPLIFETIME(AArmoredGroundUnit, ReplicatedRotation);
}

#if WITH_EDITOR
void AArmoredGroundUnit::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    const FName PropertyName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;
    if (PropertyName == GET_MEMBER_NAME_CHECKED(AArmoredGroundUnit, RepulsorSocketPrefix) || PropertyName == GET_MEMBER_NAME_CHECKED(AArmoredGroundUnit, BaseRepulsorSpec))
    {
        DiscoverRepulsorSockets();
        CalibrateRepulsorAssembly();
    }
}
#endif

//==============================================================================
//                          repulsor socket discovery
//==============================================================================

void AArmoredGroundUnit::DiscoverRepulsorSockets()
{
    RepulsorArray.Empty();
    if (!HullGeometry || !HullGeometry->GetStaticMesh()) { return; }

    TArray<FName> SocketNames = HullGeometry->GetAllSocketNames();
    for (const FName& SocketName : SocketNames)
    {
        if (SocketName.ToString().StartsWith(RepulsorSocketPrefix))
        {
            FRepulsorSpecifications NewSpec = BaseRepulsorSpec;
            NewSpec.RepulsorID = SocketName;
            const FTransform SocketTransform = HullGeometry->GetSocketTransform(SocketName, ERelativeTransformSpace::RTS_Component);
            NewSpec.LocalMountPoint = SocketTransform.GetLocation();
            RepulsorArray.Add(NewSpec);
        }
    }
}

//==============================================================================
//                          repulsor calibration
//==============================================================================

void AArmoredGroundUnit::CalibrateRepulsorAssembly()
{
    if (!HullGeometry) { return; }

    const int32 RepulsorCount = RepulsorArray.Num();
    if (RepulsorCount == 0) { return; }

    const float TotalMass_kg = HullGeometry->GetMass();
    const FVector WorldCoM = HullGeometry->GetCenterOfMass();
    const FTransform HullXfm = HullGeometry->GetComponentTransform();
    const FVector LocalCoM = HullXfm.InverseTransformPosition(WorldCoM);

    TArray<FVector> LocalSpringPositions;
    LocalSpringPositions.Reserve(RepulsorCount);
    for (const FRepulsorSpecifications& Spec : RepulsorArray)
    {
        if (HullGeometry->DoesSocketExist(Spec.RepulsorID))
        {
            const FVector WorldLoc = HullGeometry->GetSocketLocation(Spec.RepulsorID);
            LocalSpringPositions.Add(HullXfm.InverseTransformPosition(WorldLoc));
        }
        else { LocalSpringPositions.Add(FVector::ZeroVector); }
    }

    TArray<float> SprungMasses;
    bool bSolved = ComputeRepulsorSprungMasses(LocalSpringPositions, LocalCoM, TotalMass_kg, SprungMasses);
    if (!bSolved)
    {
        const float MassPerRepulsor = TotalMass_kg / static_cast<float>(RepulsorCount);
        SprungMasses.Init(MassPerRepulsor, RepulsorCount);
    }

    for (int32 i = 0; i < RepulsorCount; ++i)
    {
        FRepulsorSpecifications& Spec = RepulsorArray[i];
        if (SprungMasses.IsValidIndex(i)) { Spec.Calibrate(SprungMasses[i]); }
    }
}

//==============================================================================
//                          sprung mass computation
//==============================================================================

bool AArmoredGroundUnit::ComputeRepulsorSprungMasses(const TArray<FVector>& LocalPositions, const FVector& LocalCoM, float TotalMass, TArray<float>& OutMasses)
{
    const int32 N = LocalPositions.Num();
    if (N < 3) { OutMasses.Init(TotalMass / FMath::Max(1, N), N); return false; }

    TArray<FVector> R;
    R.Reserve(N);
    for (const FVector& P : LocalPositions) { R.Add(P - LocalCoM); }

    double A[4][5] = {{0}};
    for (int32 i = 0; i < N; ++i)
    {
        A[0][0] += 1.0;      A[0][1] += R[i].X;  A[0][2] += R[i].Y;  A[0][3] += R[i].Z;  A[0][4] += TotalMass / N;
        A[1][0] += R[i].X;   A[1][1] += R[i].X * R[i].X;  A[1][2] += R[i].X * R[i].Y;  A[1][3] += R[i].X * R[i].Z;  A[1][4] += 0.0;
        A[2][0] += R[i].Y;   A[2][1] += R[i].Y * R[i].X;  A[2][2] += R[i].Y * R[i].Y;  A[2][3] += R[i].Y * R[i].Z;  A[2][4] += 0.0;
        A[3][0] += R[i].Z;   A[3][1] += R[i].Z * R[i].X;  A[3][2] += R[i].Z * R[i].Y;  A[3][3] += R[i].Z * R[i].Z;  A[3][4] += 0.0;
    }

    for (int32 Col = 0; Col < 4; ++Col)
    {
        int32 MaxRow = Col;
        for (int32 Row = Col + 1; Row < 4; ++Row)
        {
            if (FMath::Abs(A[Row][Col]) > FMath::Abs(A[MaxRow][Col])) { MaxRow = Row; }
        }
        for (int32 K = Col; K <= 4; ++K) { double Tmp = A[Col][K]; A[Col][K] = A[MaxRow][K]; A[MaxRow][K] = Tmp; }
        if (FMath::Abs(A[Col][Col]) < 1e-12) { return false; }
        for (int32 Row = Col + 1; Row < 4; ++Row)
        {
            double Factor = A[Row][Col] / A[Col][Col];
            for (int32 K = Col; K <= 4; ++K) { A[Row][K] -= Factor * A[Col][K]; }
        }
    }

    double Lambda[4];
    for (int32 Row = 3; Row >= 0; --Row)
    {
        Lambda[Row] = A[Row][4];
        for (int32 K = Row + 1; K < 4; ++K) { Lambda[Row] -= A[Row][K] * Lambda[K]; }
        Lambda[Row] /= A[Row][Row];
    }

    OutMasses.SetNum(N);
    for (int32 i = 0; i < N; ++i)
    {
        OutMasses[i] = static_cast<float>(TotalMass / N + Lambda[0] + Lambda[1] * R[i].X + Lambda[2] * R[i].Y + Lambda[3] * R[i].Z);
        if (OutMasses[i] < 0.0f) { OutMasses[i] = TotalMass / N; }
    }
    return true;
}

//==============================================================================
//                          equilibrium transform
//==============================================================================

void AArmoredGroundUnit::ApplyEquilibriumTransform()
{
    if (!HullGeometry || RepulsorArray.Num() < 3 || !GetWorld()) { return; }

    const FTransform CurrentTransform = GetActorTransform();
    const FVector VehicleUpDir = CurrentTransform.GetUnitAxis(EAxis::Z);

    TArray<FVector> GroundPoints;
    GroundPoints.Reserve(RepulsorArray.Num());

    FCollisionQueryParams TraceParams(TEXT("EquilibriumTrace"), true, this);
    FCollisionObjectQueryParams ObjectParams(ECC_WorldStatic);

    for (const FRepulsorSpecifications& Spec : RepulsorArray)
    {
        const FVector MountWorld = HullGeometry->GetSocketLocation(Spec.RepulsorID);
        const FVector TraceStart = MountWorld + VehicleUpDir * 50.0f;
        const FVector TraceEnd = MountWorld - VehicleUpDir * Spec.MaxTraceRange_cm;

        FHitResult Hit;
        if (GetWorld()->LineTraceSingleByObjectType(Hit, TraceStart, TraceEnd, ObjectParams, TraceParams)) { GroundPoints.Add(Hit.ImpactPoint); }
        else { GroundPoints.Add(MountWorld - VehicleUpDir * Spec.DesiredHeight_cm); }
    }

    if (GroundPoints.Num() < 3) { return; }

    FVector GroundCenter = FVector::ZeroVector;
    for (const FVector& P : GroundPoints) { GroundCenter += P; }
    GroundCenter /= static_cast<float>(GroundPoints.Num());

    const FVector TargetPosition = GroundCenter + FVector::UpVector * BaseRepulsorSpec.DesiredHeight_cm;
    SetActorLocation(TargetPosition, false, nullptr, ETeleportType::TeleportPhysics);

    if (HullGeometry->IsSimulatingPhysics())
    {
        HullGeometry->SetPhysicsLinearVelocity(FVector::ZeroVector);
        HullGeometry->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    }
}

//==============================================================================
//                          gt↔pt data transfer
//==============================================================================

void AArmoredGroundUnit::PublishInputToPhysicsThread()
{
    if (!RepulsorCallback || !HullGeometry) { return; }

    FRepulsorSolverInput Input;
    
    Input.RepulsorSpecs = RepulsorArray;
    Input.HullTransform = HullGeometry->GetComponentTransform();
    Input.WorldCoM = HullGeometry->GetCenterOfMass();

    // PPU commands
    Input.Throttle_CMD = InputTensor.Throttle_CMD;
    Input.Lateral_CMD = InputTensor.Lateral_CMD;
    Input.MouseYaw_CMD = InputTensor.MouseYaw_CMD;
    Input.MousePitch_CMD = InputTensor.MousePitch_CMD;
    Input.bVerticalThrust = InputTensor.Vertical_CMD;
    Input.bBoostActive = bBoostActive;

    // Height boost
    Input.Lift_Capacitor = Lift_Capacitor;
    Input.BaseDesiredHeight_cm = BaseRepulsorSpec.DesiredHeight_cm;
    Input.MaxHeightBoost_cm = MaxHeightBoost_cm;
    Input.CurrentHeightOffset_cm = CurrentHeightOffset_cm;

    // Propulsion dynamics
    Input.MaxThrustForce_N = MaxThrustForce_N;
    Input.MaxStrafeForce_N = MaxStrafeForce_N;
    Input.ThrusterResponseExponent = ThrusterResponseExponent;
    Input.BoostMultiplier = BoostMultiplier;
    Input.MaxSpeed_cms = MaxThrusterSpeed_mps * 100.0f;
    
    // Attitude dynamics
    Input.RollAngleLimit_deg = RollAngleLimit_deg;
    Input.PitchAngleLimit_deg = PitchAngleLimit_deg;
    Input.RollTorqueGain = RollTorqueGain;
    Input.PitchTorqueGain = PitchTorqueGain;
    Input.YawTorqueGain = MaxYawTorque_Nm;
    
    // VRC
    Input.VRC_Coefficient = VRC_Coefficient;
    
    // Mass
    Input.CachedMass_kg = CachedMass_kg;
    Input.MassNormalizedAccel_mss = MassNormalizedAccel_mss;

    //--------------------------------------------------------------------------
    // altitude traces using VEHICLE UP VECTOR + store HIT NORMALS
    //--------------------------------------------------------------------------
    const FVector VehicleUp = HullGeometry->GetUpVector();
    
    Input.MeasuredAltitudes.SetNum(RepulsorArray.Num());
    Input.HitNormals.SetNum(RepulsorArray.Num());
    CachedHitNormals.SetNum(RepulsorArray.Num());
    
    FCollisionQueryParams TraceParams;
    TraceParams.AddIgnoredActor(this);
    TraceParams.bTraceComplex = false;

    for (int32 i = 0; i < RepulsorArray.Num(); ++i)
    {
        const FRepulsorSpecifications& Spec = RepulsorArray[i];
        const FVector MountLocation = HullGeometry->GetSocketLocation(Spec.RepulsorID);
        const FVector TraceStart = MountLocation;
        const FVector TraceEnd = MountLocation - (VehicleUp * Spec.MaxTraceRange_cm);

        FHitResult Hit;
        if (GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECC_Visibility, TraceParams))
        {
            Input.MeasuredAltitudes[i] = Hit.Distance;
            Input.HitNormals[i] = Hit.ImpactNormal;
            CachedHitNormals[i] = Hit.ImpactNormal;
        }
        else
        {
            Input.MeasuredAltitudes[i] = Spec.MaxTraceRange_cm;
            Input.HitNormals[i] = FVector::UpVector;  // Use world up as fallback
            CachedHitNormals[i] = FVector::ZeroVector;
        }
    }

    RepulsorCallback->InputDataChannel.Write(Input);
}

void AArmoredGroundUnit::ReadStateFromPhysicsThread()
{
    if (!RepulsorCallback) { return; }
    RepulsorCallback->OutputDataChannel.Read(LatestRepulsorOutput);
}
