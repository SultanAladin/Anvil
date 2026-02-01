#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GroundWeaponModule.generated.h"

class USphereComponent;
class UStaticMeshComponent;
class UPrimitiveComponent;
class AArmoredGroundUnit;
class AImpactProjectile;
struct FHitResult;

USTRUCT(BlueprintType)
struct ANVIL_API FWeaponConfig
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    float FireInterval_s = 0.12f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    float ProjectileSpeed_cms = 12000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    int32 MagazineSize = 60;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    int32 ReserveAmmo = 240;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    float RecoilTravel_cm = 8.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    float RecoilReturnRate = 18.0f;
};

UCLASS()
class ANVIL_API AGroundWeaponModule : public AActor
{
    GENERATED_BODY()

public:
    AGroundWeaponModule();

    virtual void Tick(float DeltaTime) override;

    UFUNCTION(BlueprintCallable)
    void TriggerFire(const FVector& AimPoint);

    UFUNCTION(BlueprintCallable)
    void RequestEquip(AArmoredGroundUnit* Unit);

    UFUNCTION(BlueprintCallable)
    FText GetModuleLabel() const { return ModuleLabel; }

protected:
    virtual void BeginPlay() override;

    UFUNCTION()
    void OnPickupOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

    void StartEquip(AArmoredGroundUnit* Unit);
    void EnterDroppedState();
    void UpdateEquip(float DeltaTime);
    void UpdateAim(float DeltaTime, const FVector& AimPoint);

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* TurretBaseMesh = nullptr;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* GunMesh = nullptr;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* AmmoContainerMesh = nullptr;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USphereComponent* PickupTrigger = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mount")
    float EquipMoveSpeed_cmps = 140.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mount")
    float EquipRotSpeed_degps = 220.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mount")
    float EquipAttachDistance_cm = 8.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mount")
    float EquipAttachRotError_deg = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped")
    bool bHoverWhenDropped = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped", meta = (ClampMin = "0.0"))
    float HoverHeight_cm = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped", meta = (ClampMin = "0.0"))
    float HoverTraceLength_cm = 400.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped", meta = (ClampMin = "0.0"))
    float HoverSpring = 8.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped", meta = (ClampMin = "0.0"))
    float HoverDamping = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped", meta = (ClampMin = "0.0"))
    float UprightTorqueStrength = 6.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped", meta = (ClampMin = "0.0"))
    float AngularDampingStrength = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped", meta = (ClampMin = "0.0"))
    float DroppedVRC_Coefficient = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropped", meta = (ClampMin = "0.0"))
    float GroundNormalInterpSpeed = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mount")
    bool bAutoEquipOnOverlap = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mount")
    bool bMountOnTurretModule = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mount")
    FName MountSocketName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    FText ModuleLabel = FText::FromString(TEXT("Weapon"));

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    FWeaponConfig Config;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    TSubclassOf<AImpactProjectile> ProjectileClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    FName MuzzleSocketName = TEXT("WeaponOut");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    float YawRate_degps = 90.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    float PitchRate_degps = 60.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    float MinPitch_deg = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weapon")
    float MaxPitch_deg = 45.0f;

protected:
    UPROPERTY(Transient)
    TObjectPtr<AArmoredGroundUnit> OwnerUnit = nullptr;

    FName ActiveMountSocketName = NAME_None;

    FVector DroppedGroundNormal = FVector::UpVector;

    bool bEquipInProgress = false;

    float CurrentYaw_deg = 0.0f;
    float CurrentPitch_deg = 0.0f;

    float FireCooldown_s = 0.0f;

    float AutoEquipQueryCooldown_s = 0.0f;

    FVector GunBaseLocalPos = FVector::ZeroVector;
    float RecoilOffset_cm = 0.0f;
};
