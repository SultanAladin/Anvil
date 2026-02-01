#include "GroundWeaponModule.h"

#include "ArmoredGroundUnit.h"
#include "ImpactProjectile.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/EngineTypes.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"

AGroundWeaponModule::AGroundWeaponModule()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PostPhysics;

    TurretBaseMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TurretBaseMesh"));
    SetRootComponent(TurretBaseMesh);
    TurretBaseMesh->SetSimulatePhysics(true);
    TurretBaseMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    TurretBaseMesh->SetLinearDamping(0.0f);
    TurretBaseMesh->SetAngularDamping(0.0f);

    GunMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("GunMesh"));
    GunMesh->SetupAttachment(TurretBaseMesh);
    GunMesh->SetSimulatePhysics(false);
    GunMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

    AmmoContainerMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("AmmoContainerMesh"));
    AmmoContainerMesh->SetupAttachment(TurretBaseMesh);
    AmmoContainerMesh->SetSimulatePhysics(false);
    AmmoContainerMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

    PickupTrigger = CreateDefaultSubobject<USphereComponent>(TEXT("PickupTrigger"));
    PickupTrigger->SetupAttachment(TurretBaseMesh);
    PickupTrigger->InitSphereRadius(130.0f);
    PickupTrigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
    PickupTrigger->SetCollisionResponseToAllChannels(ECR_Ignore);
    PickupTrigger->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
    PickupTrigger->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap);

    PickupTrigger->OnComponentBeginOverlap.AddDynamic(this, &AGroundWeaponModule::OnPickupOverlap);
}

void AGroundWeaponModule::BeginPlay()
{
    Super::BeginPlay();

    if (GunMesh)
    {
        GunBaseLocalPos = GunMesh->GetRelativeLocation();
    }

    if (PickupTrigger)
    {
        if (TurretBaseMesh)
        {
            PickupTrigger->AttachToComponent(TurretBaseMesh, FAttachmentTransformRules::KeepRelativeTransform);

            FVector LocalMin = FVector::ZeroVector;
            FVector LocalMax = FVector::ZeroVector;
            TurretBaseMesh->GetLocalBounds(LocalMin, LocalMax);
            const FVector LocalCenter = 0.5f * (LocalMin + LocalMax);
            PickupTrigger->SetRelativeLocation(LocalCenter);
        }
        else if (RootComponent)
        {
            PickupTrigger->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
            PickupTrigger->SetRelativeLocation(FVector::ZeroVector);
        }
    }
}

void AGroundWeaponModule::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (FireCooldown_s > 0.0f) { FireCooldown_s = FMath::Max(0.0f, FireCooldown_s - DeltaTime); }

    if (bEquipInProgress)
    {
        UpdateEquip(DeltaTime);
        return;
    }

    if (!OwnerUnit)
    {
        if (!GetActorEnableCollision()) { SetActorEnableCollision(true); }
        if (TurretBaseMesh)
        {
            if (!TurretBaseMesh->IsSimulatingPhysics()) { TurretBaseMesh->SetSimulatePhysics(true); }
            if (TurretBaseMesh->GetCollisionEnabled() != ECollisionEnabled::QueryAndPhysics) { TurretBaseMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics); }
        }
        if (GunMesh && GunMesh->GetCollisionEnabled() != ECollisionEnabled::NoCollision) { GunMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
        if (AmmoContainerMesh && AmmoContainerMesh->GetCollisionEnabled() != ECollisionEnabled::NoCollision) { AmmoContainerMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
        if (PickupTrigger && PickupTrigger->GetCollisionEnabled() != ECollisionEnabled::QueryOnly) { PickupTrigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly); }
    }

    if (OwnerUnit)
    {
        if (TurretBaseMesh)
        {
            if (TurretBaseMesh->IsSimulatingPhysics()) { TurretBaseMesh->SetSimulatePhysics(false); }
            if (TurretBaseMesh->GetCollisionEnabled() != ECollisionEnabled::NoCollision) { TurretBaseMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
        }
        if (GunMesh && GunMesh->GetCollisionEnabled() != ECollisionEnabled::NoCollision) { GunMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
        if (AmmoContainerMesh && AmmoContainerMesh->GetCollisionEnabled() != ECollisionEnabled::NoCollision) { AmmoContainerMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
        if (PickupTrigger && PickupTrigger->GetCollisionEnabled() != ECollisionEnabled::NoCollision) { PickupTrigger->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
        if (GetActorEnableCollision()) { SetActorEnableCollision(false); }
    }

    if (!OwnerUnit && bHoverWhenDropped && TurretBaseMesh && TurretBaseMesh->IsSimulatingPhysics() && GetWorld())
    {
        const float HoverHeightMin_cm = 100.0f;
        const float DroppedVRC_Coefficient_Use = 0.0f;
        const float AngularDampingStrength_Use = 0.0f;
        const float HoverTraceMin_cm = HoverHeightMin_cm + 200.0f;

        const FVector UpDir = FVector::UpVector;
        const FVector Start = TurretBaseMesh->GetCenterOfMass();
        const float TraceLen_cm = FMath::Max(FMath::Max(HoverTraceLength_cm, HoverTraceMin_cm), 0.0f);
        const FVector End = Start - UpDir * TraceLen_cm;

        DrawDebugLine(GetWorld(), Start, End, FColor::Green, false, 0.0f, 0, 1.5f);

        FCollisionQueryParams Params;
        Params.AddIgnoredActor(this);

        FHitResult Hit;
        const FCollisionObjectQueryParams ObjParams(ECollisionChannel::ECC_WorldStatic);
        if (GetWorld()->LineTraceSingleByObjectType(Hit, Start, End, ObjParams, Params))
        {
            DrawDebugPoint(GetWorld(), Hit.ImpactPoint, 8.0f, FColor::Green, false, 0.0f);
            DroppedGroundNormal = UpDir;

            const float Dist_cm = FVector::DotProduct(Start - Hit.ImpactPoint, UpDir);
            const float ExtentZ_cm = TurretBaseMesh->Bounds.BoxExtent.Z;
            const float DesiredCOMHeight_cm = FMath::Max(FMath::Max(HoverHeight_cm, HoverHeightMin_cm), 0.0f) + FMath::Max(ExtentZ_cm, 0.0f);
            const float MaxErr_cm = FMath::Max(DesiredCOMHeight_cm, 1.0f);
            const float Error_cm = FMath::Clamp(DesiredCOMHeight_cm - Dist_cm, -MaxErr_cm, MaxErr_cm);

            const float Accel_cms2 = (Error_cm * HoverSpring);

            const float MassKg = FMath::Max(TurretBaseMesh->GetMass(), 1.0f);
            const float TotalAccel_cms2 = 980.0f + Accel_cms2;
            FVector Force = UpDir * (TotalAccel_cms2 * MassKg);
            Force = Force.GetClampedToMaxSize(MassKg * 3920.0f);
            TurretBaseMesh->AddForceAtLocation(Force, Start);

            const FVector Vel_cms = TurretBaseMesh->GetComponentVelocity();
            const FVector TangentVel_cms = Vel_cms - UpDir * FVector::DotProduct(Vel_cms, UpDir);
            if (DroppedVRC_Coefficient_Use > 0.0f && TangentVel_cms.SizeSquared() > 25.0f)
            {
                const float BrakeCoeff_sInv = FMath::Max(DroppedVRC_Coefficient_Use, 0.0f);
                FVector BrakeForce = -TangentVel_cms * (MassKg * BrakeCoeff_sInv);
                BrakeForce = BrakeForce.GetClampedToMaxSize(MassKg * 1960.0f);
                TurretBaseMesh->AddForceAtLocation(BrakeForce, Start);
            }

            const FVector Gravity_cms2(0.0f, 0.0f, -980.0f);
            const FVector GravTangent_cms2 = Gravity_cms2 - UpDir * FVector::DotProduct(Gravity_cms2, UpDir);
            if (DroppedVRC_Coefficient_Use > 0.0f)
            {
                FVector HoldForce = -GravTangent_cms2 * MassKg;
                HoldForce = HoldForce.GetClampedToMaxSize(MassKg * 1960.0f);
                TurretBaseMesh->AddForceAtLocation(HoldForce, Start);
            }
        }

        const FVector CurrentUp = TurretBaseMesh->GetUpVector();
        const FVector Axis = FVector::CrossProduct(CurrentUp, UpDir);
        const float AxisLen = Axis.Size();
        if (AxisLen > UE_SMALL_NUMBER)
        {
            const FVector TorqueAxis = Axis / AxisLen;
            const float MassKg = FMath::Max(TurretBaseMesh->GetMass(), 1.0f);
            FVector UprightTorque = TorqueAxis * (AxisLen * UprightTorqueStrength * MassKg);
            UprightTorque = UprightTorque.GetClampedToMaxSize(MassKg * 300000.0f);
            TurretBaseMesh->AddTorqueInRadians(UprightTorque, NAME_None, false);
        }

        const FVector AngVel_Rad = TurretBaseMesh->GetPhysicsAngularVelocityInRadians();
        if (AngularDampingStrength_Use > 0.0f && !AngVel_Rad.IsNearlyZero())
        {
            const float MassKg = FMath::Max(TurretBaseMesh->GetMass(), 1.0f);
            FVector DampTorque = -AngVel_Rad * (AngularDampingStrength_Use * MassKg);
            DampTorque = DampTorque.GetClampedToMaxSize(MassKg * 300000.0f);
            TurretBaseMesh->AddTorqueInRadians(DampTorque, NAME_None, false);
        }
    }

    if (OwnerUnit)
    {
        UpdateAim(DeltaTime, OwnerUnit->GetCachedAimPoint());
    }

    if (GunMesh)
    {
        RecoilOffset_cm = FMath::FInterpTo(RecoilOffset_cm, 0.0f, DeltaTime, FMath::Max(Config.RecoilReturnRate, 0.0f));
        GunMesh->SetRelativeLocation(GunBaseLocalPos - FVector(RecoilOffset_cm, 0.0f, 0.0f));
    }
}

void AGroundWeaponModule::OnPickupOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
    return;
}

void AGroundWeaponModule::RequestEquip(AArmoredGroundUnit* Unit)
{
    StartEquip(Unit);
}

void AGroundWeaponModule::StartEquip(AArmoredGroundUnit* Unit)
{
    if (!Unit || OwnerUnit) { return; }

    DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);

    if (TurretBaseMesh)
    {
        if (USceneComponent* RootComp = GetRootComponent())
        {
            if (RootComp != TurretBaseMesh && TurretBaseMesh->GetAttachParent() == RootComp)
            {
                TurretBaseMesh->SetRelativeLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator, false, nullptr, ETeleportType::TeleportPhysics);
            }
        }
    }

    OwnerUnit = Unit;
    bEquipInProgress = true;
    ActiveMountSocketName = NAME_None;
    DroppedGroundNormal = FVector::UpVector;

    if (TurretBaseMesh)
    {
        TurretBaseMesh->SetSimulatePhysics(false);
        TurretBaseMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        TurretBaseMesh->SetLinearDamping(0.0f);
        TurretBaseMesh->SetAngularDamping(0.0f);
    }
    if (GunMesh) { GunMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
    if (AmmoContainerMesh) { AmmoContainerMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
    SetActorEnableCollision(false);

    if (PickupTrigger) { PickupTrigger->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
}

void AGroundWeaponModule::EnterDroppedState()
{
    OwnerUnit = nullptr;
    bEquipInProgress = false;
    ActiveMountSocketName = NAME_None;

    DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    SetActorEnableCollision(true);

    if (USceneComponent* RootComp = GetRootComponent())
    {
        RootComp->SetUsingAbsoluteLocation(false);
        RootComp->SetUsingAbsoluteRotation(false);
        RootComp->SetUsingAbsoluteScale(false);
    }

    if (TurretBaseMesh)
    {
        if (USceneComponent* RootComp = GetRootComponent())
        {
            if (RootComp != TurretBaseMesh && TurretBaseMesh->GetAttachParent() == RootComp)
            {
                TurretBaseMesh->SetRelativeLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator, false, nullptr, ETeleportType::TeleportPhysics);
            }
        }
    }

    if (TurretBaseMesh)
    {
        TurretBaseMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        TurretBaseMesh->SetSimulatePhysics(true);
        TurretBaseMesh->SetLinearDamping(0.0f);
        TurretBaseMesh->SetAngularDamping(0.0f);
    }
    if (GunMesh) { GunMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
    if (AmmoContainerMesh) { AmmoContainerMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); }
    if (PickupTrigger) { PickupTrigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly); }
}

void AGroundWeaponModule::UpdateEquip(float DeltaTime)
{
    if (!OwnerUnit) { bEquipInProgress = false; return; }

    UStaticMeshComponent* ParentComp = nullptr;

    const FName PrimarySocket(TEXT("WeaponPort-Primary"));
    const FName SecondarySocket(TEXT("WeaponPort-Secondary"));
    const bool bIsPortPreference = MountSocketName.IsNone() || MountSocketName == PrimarySocket || MountSocketName == SecondarySocket;

    auto HasDesiredSocket = [&](UStaticMeshComponent* Comp) -> bool
    {
        if (!Comp) { return false; }
        if (bIsPortPreference) { return Comp->DoesSocketExist(PrimarySocket) || Comp->DoesSocketExist(SecondarySocket); }
        return Comp->DoesSocketExist(MountSocketName);
    };

    UStaticMeshComponent* CandidateA = (bMountOnTurretModule && OwnerUnit->TurretModule) ? OwnerUnit->TurretModule : nullptr;
    UStaticMeshComponent* CandidateB = OwnerUnit->TurretBase;
    UStaticMeshComponent* CandidateC = OwnerUnit->HullGeometry;

    if (HasDesiredSocket(CandidateA)) { ParentComp = CandidateA; }
    else if (HasDesiredSocket(CandidateB)) { ParentComp = CandidateB; }
    else if (HasDesiredSocket(CandidateC)) { ParentComp = CandidateC; }
    else if (CandidateA) { ParentComp = CandidateA; }
    else if (CandidateB) { ParentComp = CandidateB; }
    else { ParentComp = CandidateC; }

    if (!ParentComp)
    {
        EnterDroppedState();
        return;
    }

    if (ActiveMountSocketName.IsNone())
    {
        ActiveMountSocketName = OwnerUnit->ChooseWeaponPortSocket(ParentComp, MountSocketName);
        if (bIsPortPreference)
        {
            if (!ParentComp->DoesSocketExist(ActiveMountSocketName)) { ActiveMountSocketName = NAME_None; }
        }
        else
        {
            if (!ParentComp->DoesSocketExist(ActiveMountSocketName)) { ActiveMountSocketName = NAME_None; }
        }
    }

    const FName UseSocketName = ActiveMountSocketName;

    if (UseSocketName.IsNone() || !ParentComp->DoesSocketExist(UseSocketName))
    {
        EnterDroppedState();
        return;
    }

    const FTransform TargetXf = ParentComp->GetSocketTransform(UseSocketName, RTS_World);

    if (OwnerUnit && OwnerUnit->bDrawDebug && GetWorld())
    {
        DrawDebugCoordinateSystem(GetWorld(), TargetXf.GetLocation(), TargetXf.Rotator(), 25.0f, false, 0.0f, 0, 1.0f);
        DrawDebugLine(GetWorld(), GetActorLocation(), TargetXf.GetLocation(), FColor::Cyan, false, 0.0f, 0, 1.0f);
    }

    const FVector NewLoc = FMath::VInterpConstantTo(GetActorLocation(), TargetXf.GetLocation(), DeltaTime, FMath::Max(EquipMoveSpeed_cmps, 0.0f));
    const FRotator NewRot = FMath::RInterpConstantTo(GetActorRotation(), TargetXf.Rotator(), DeltaTime, FMath::Max(EquipRotSpeed_degps, 0.0f));
    if (USceneComponent* RootComp = GetRootComponent())
    {
        RootComp->SetWorldLocationAndRotation(NewLoc, NewRot, false, nullptr, ETeleportType::TeleportPhysics);
    }
    else
    {
        SetActorLocationAndRotation(NewLoc, NewRot, false, nullptr, ETeleportType::TeleportPhysics);
    }

    const float AttachDist_cm = FMath::Max(EquipAttachDistance_cm, 0.0f);
    const float DistSq = FVector::DistSquared(NewLoc, TargetXf.GetLocation());

    const FRotator DeltaRot = (NewRot - TargetXf.Rotator()).GetNormalized();
    const float RotErr_deg = FMath::Max3(FMath::Abs(DeltaRot.Pitch), FMath::Abs(DeltaRot.Yaw), FMath::Abs(DeltaRot.Roll));

    if (DistSq < (AttachDist_cm * AttachDist_cm) && RotErr_deg < FMath::Max(EquipAttachRotError_deg, 0.0f))
    {
        if (USceneComponent* RootComp = GetRootComponent())
        {
            RootComp->SetUsingAbsoluteLocation(false);
            RootComp->SetUsingAbsoluteRotation(false);
            RootComp->SetUsingAbsoluteScale(false);

            if (TurretBaseMesh && TurretBaseMesh == RootComp)
            {
                RootComp->AttachToComponent(ParentComp, FAttachmentTransformRules::SnapToTargetNotIncludingScale, UseSocketName);
                RootComp->SetRelativeLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator, false, nullptr, ETeleportType::TeleportPhysics);
            }
            else
            {
                FTransform DesiredRootWorld = TargetXf;
                if (TurretBaseMesh)
                {
                    const FTransform TurretRelToRoot = TurretBaseMesh->GetRelativeTransform();
                    DesiredRootWorld = TargetXf * TurretRelToRoot.Inverse();
                }

                RootComp->SetWorldTransform(DesiredRootWorld, false, nullptr, ETeleportType::TeleportPhysics);
                RootComp->AttachToComponent(ParentComp, FAttachmentTransformRules::KeepWorldTransform, UseSocketName);
            }
        }
        bEquipInProgress = false;
        SetActorEnableCollision(false);
        if (TurretBaseMesh) { TurretBaseMesh->SetSimulatePhysics(false); }
        OwnerUnit->RegisterWeaponModule(this);
    }
}

void AGroundWeaponModule::UpdateAim(float DeltaTime, const FVector& AimPoint)
{
    if (!TurretBaseMesh || !GunMesh) { return; }

    const USceneComponent* ParentComp = TurretBaseMesh->GetAttachParent();
    const UStaticMeshComponent* ParentMeshComp = Cast<UStaticMeshComponent>(ParentComp);

    const FTransform BaseFrameWorld = (OwnerUnit && ParentMeshComp && !ActiveMountSocketName.IsNone() && ParentMeshComp->DoesSocketExist(ActiveMountSocketName))
        ? ParentMeshComp->GetSocketTransform(ActiveMountSocketName, RTS_World)
        : (ParentComp ? ParentComp->GetComponentTransform() : FTransform::Identity);

    const FVector BaseLoc = BaseFrameWorld.GetLocation();
    const FVector ToAim = AimPoint - BaseLoc;
    if (ToAim.SizeSquared() < UE_SMALL_NUMBER) { return; }

    const FVector LocalDir = BaseFrameWorld.InverseTransformVectorNoScale(ToAim).GetSafeNormal();
    const float TargetYaw_deg = FMath::RadiansToDegrees(FMath::Atan2(LocalDir.Y, LocalDir.X));
    CurrentYaw_deg = FMath::FixedTurn(CurrentYaw_deg, TargetYaw_deg, YawRate_degps * DeltaTime);
    TurretBaseMesh->SetRelativeRotation(FRotator(0.0f, CurrentYaw_deg, 0.0f));

    const FVector GunLoc = GunMesh->GetComponentLocation();
    const FVector ToAimGun = AimPoint - GunLoc;
    if (ToAimGun.SizeSquared() < UE_SMALL_NUMBER) { return; }

    const FVector LocalDirGun = TurretBaseMesh->GetComponentTransform().InverseTransformVectorNoScale(ToAimGun).GetSafeNormal();
    float TargetPitch_deg = FMath::RadiansToDegrees(FMath::Atan2(LocalDirGun.Z, LocalDirGun.X));
    TargetPitch_deg = FMath::Clamp(TargetPitch_deg, MinPitch_deg, MaxPitch_deg);

    CurrentPitch_deg = FMath::FixedTurn(CurrentPitch_deg, TargetPitch_deg, PitchRate_degps * DeltaTime);
    GunMesh->SetRelativeRotation(FRotator(CurrentPitch_deg, 0.0f, 0.0f));

    if (OwnerUnit && OwnerUnit->bDrawDebug && GetWorld())
    {
        const FTransform MuzzleXf = GunMesh->DoesSocketExist(MuzzleSocketName) ? GunMesh->GetSocketTransform(MuzzleSocketName) : GunMesh->GetComponentTransform();

        const FVector MuzzleStart = MuzzleXf.GetLocation();
        const FVector MuzzleForward = MuzzleXf.GetRotation().GetForwardVector();
        const float TraceDistance = 250000.0f;
        const FVector TraceEnd = MuzzleStart + MuzzleForward * TraceDistance;

        FCollisionQueryParams Params;
        Params.AddIgnoredActor(OwnerUnit);
        Params.AddIgnoredActor(this);
        Params.bTraceComplex = false;

        FHitResult Hit;
        const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, MuzzleStart, TraceEnd, ECC_Visibility, Params);
        const FVector EndPoint = bHit ? Hit.ImpactPoint : TraceEnd;

        DrawDebugLine(GetWorld(), MuzzleStart, EndPoint, FColor::Cyan, false, 0.0f, 0, 1.5f);
        DrawDebugPoint(GetWorld(), MuzzleStart, 6.0f, FColor::Cyan, false, 0.0f);
    }
}

void AGroundWeaponModule::TriggerFire(const FVector& AimPoint)
{
    if (!OwnerUnit || !ProjectileClass || !GunMesh || !GetWorld()) { return; }
    if (FireCooldown_s > 0.0f) { return; }

    const FTransform MuzzleXf = GunMesh->DoesSocketExist(MuzzleSocketName) ? GunMesh->GetSocketTransform(MuzzleSocketName) : GunMesh->GetComponentTransform();

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = OwnerUnit;
    SpawnParams.Instigator = OwnerUnit;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    AImpactProjectile* Projectile = GetWorld()->SpawnActor<AImpactProjectile>(ProjectileClass, MuzzleXf.GetLocation(), MuzzleXf.Rotator(), SpawnParams);
    if (Projectile)
    {
        Projectile->SetIgnoredActor(OwnerUnit);
        Projectile->SetIgnoredActor(this);
        const FVector ShootDir = MuzzleXf.GetRotation().GetForwardVector();
        Projectile->SetInitialVelocity(ShootDir * Config.ProjectileSpeed_cms);
        RecoilOffset_cm = FMath::Min(RecoilOffset_cm + Config.RecoilTravel_cm, Config.RecoilTravel_cm);
    }

    FireCooldown_s = FMath::Max(Config.FireInterval_s, 0.01f);
}
