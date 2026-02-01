//==============================================================================
// UnitController.cpp - Player Controller for Armored Ground Units
// GRIT-style input handling with network replication
//==============================================================================

#include "UnitController.h"
#include "DeployableUnit/GroundUnit/ArmoredGroundUnit.h"
#include "DeployableUnit/GroundUnit/GroundWeaponModule.h"
#include "DeployableUnit/Userinterface/TacticalDisplayInterface.h"
#include "Blueprint/UserWidget.h"
#include "CollisionShape.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/EngineTypes.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "WorldCollision.h"
#include "InputMappingContext.h"

DEFINE_LOG_CATEGORY(LogUnitController);

//==============================================================================
//                          construction
//==============================================================================

AUnitController::AUnitController()
{
    PrimaryActorTick.bCanEverTick = true;
}

//==============================================================================
//                          lifecycle
//==============================================================================

void AUnitController::BeginPlay()
{
    Super::BeginPlay();

    if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(GetLocalPlayer()))
    {
        if (DefaultMappingContext) { Subsystem->AddMappingContext(DefaultMappingContext, 0); } // End if (mapping context)
    } // End if (subsystem)

    if (IsLocalController() && TacticalDisplayClass)
    {
        TacticalDisplayWidget = CreateWidget<UTacticalDisplayInterface>(this, TacticalDisplayClass);
        if (TacticalDisplayWidget) { TacticalDisplayWidget->AddToViewport(); }
    }
}

void AUnitController::SetupInputComponent()
{
    Super::SetupInputComponent();

    if (UEnhancedInputComponent* EnhancedInput = Cast<UEnhancedInputComponent>(InputComponent))
    {
        if (IA_Move) { EnhancedInput->BindAction(IA_Move, ETriggerEvent::Triggered, this, &AUnitController::OnMoveInput); EnhancedInput->BindAction(IA_Move, ETriggerEvent::Completed, this, &AUnitController::OnMoveInput); } // End if (move)
        if (IA_Look) { EnhancedInput->BindAction(IA_Look, ETriggerEvent::Triggered, this, &AUnitController::OnLookInput); } // End if (look)
        if (IA_PrimaryFire) { EnhancedInput->BindAction(IA_PrimaryFire, ETriggerEvent::Triggered, this, &AUnitController::OnPrimaryFireInput); EnhancedInput->BindAction(IA_PrimaryFire, ETriggerEvent::Completed, this, &AUnitController::OnPrimaryFireInput); } // End if (fire)
        if (IA_Reload) { EnhancedInput->BindAction(IA_Reload, ETriggerEvent::Started, this, &AUnitController::OnReloadInput); } // End if (reload)
        if (IA_Interact) { EnhancedInput->BindAction(IA_Interact, ETriggerEvent::Started, this, &AUnitController::OnInteractInput); } // End if (interact)
        if (IA_CycleWeapon) { EnhancedInput->BindAction(IA_CycleWeapon, ETriggerEvent::Started, this, &AUnitController::OnCycleWeaponInput); } // End if (cycle)
        if (IA_VerticalThrust) { EnhancedInput->BindAction(IA_VerticalThrust, ETriggerEvent::Triggered, this, &AUnitController::OnVerticalThrustInput); EnhancedInput->BindAction(IA_VerticalThrust, ETriggerEvent::Completed, this, &AUnitController::OnVerticalThrustInput); } // End if (vertical)
        if (IA_Boost) { EnhancedInput->BindAction(IA_Boost, ETriggerEvent::Triggered, this, &AUnitController::OnBoostInput); EnhancedInput->BindAction(IA_Boost, ETriggerEvent::Completed, this, &AUnitController::OnBoostInput); } // End if (boost)
    } // End if (enhanced input)
}

void AUnitController::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (ControlledUnit)
    {
        CurrentInputTensor.Timestamp = GetWorld()->GetTimeSeconds();

        const float ReticlePitchNorm = (MaxReticleOffsetY_Px > UE_SMALL_NUMBER) ? (ReticleOffset_Px.Y / MaxReticleOffsetY_Px) : 0.0f;
        CurrentInputTensor.MousePitch_CMD = FMath::Clamp(-ReticlePitchNorm, -1.0f, 1.0f);

        if (IsLocalController())
        {
            ControlledUnit->InputTensor = CurrentInputTensor;
            if (!HasAuthority()) { Server_SendUnitInput(CurrentInputTensor); } // End if (client)
        } // End if (local controller)

        // Reset per-frame inputs
        CurrentInputTensor.Reset();
    } // End if (controlled unit)

    if (IsLocalController() && TacticalDisplayWidget)
    {
        FTacticalDisplayTelemetry Telemetry;
        Telemetry.ReticleOffset_Px = ReticleOffset_Px;
        Telemetry.SelectedWeaponText = ControlledUnit ? ControlledUnit->GetActiveWeaponText() : FText::GetEmpty();
        Telemetry.bAimValid = ControlledUnit ? ControlledUnit->IsCannonAimValid() : true;

        if (ControlledUnit)
        {
            const FRepulsorSolverOutput& Out = ControlledUnit->GetLatestRepulsorOutput();
            Telemetry.Speed_cms = Out.CurrentVelocity_cms.Size();
            Telemetry.Altitude_cm = Out.AverageAltitude_cm;
            Telemetry.bGrounded = Out.bGrounded;
            Telemetry.HullRoll_deg = Out.CurrentRoll_deg;
            Telemetry.HullPitch_deg = Out.CurrentPitch_deg;
            Telemetry.TurretYaw_deg = ControlledUnit->GetTurretYaw_deg();
            Telemetry.OrdnancePitch_deg = ControlledUnit->GetOrdnancePitch_deg();
        }

        TacticalDisplayWidget->SetTelemetry(Telemetry);
    }
}

void AUnitController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);
    ControlledUnit = Cast<AArmoredGroundUnit>(InPawn);
    if (ControlledUnit) { UE_LOG(LogUnitController, Log, TEXT("Possessed ArmoredGroundUnit: %s"), *ControlledUnit->GetName()); } // End if (possessed)
}

void AUnitController::OnUnPossess()
{
    if (ControlledUnit) { UE_LOG(LogUnitController, Log, TEXT("Unpossessed ArmoredGroundUnit: %s"), *ControlledUnit->GetName()); } // End if (unpossess log)
    ControlledUnit = nullptr;
    Super::OnUnPossess();
}

void AUnitController::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
}

//==============================================================================
//                          input handlers
//==============================================================================

void AUnitController::OnMoveInput(const FInputActionValue& Value)
{
    const FVector2D MoveValue = Value.Get<FVector2D>();
    CurrentInputTensor.Throttle_CMD = MoveValue.Y;  // W/S → forward/reverse
    CurrentInputTensor.Lateral_CMD = MoveValue.X;   // A/D → strafe left/right
}

void AUnitController::OnLookInput(const FInputActionValue& Value)
{
    const FVector2D LookValue = Value.Get<FVector2D>();
    // Mouse X drives vehicle yaw (turn left/right)
    CurrentInputTensor.MouseYaw_CMD = FMath::Clamp(LookValue.X * MouseYawSensitivity, -1.0f, 1.0f);

    ReticleOffset_Px.Y = FMath::Clamp(ReticleOffset_Px.Y - LookValue.Y * ReticleSensitivity_PxPerMouseUnit, -MaxReticleOffsetY_Px, MaxReticleOffsetY_Px);
}

void AUnitController::OnPrimaryFireInput(const FInputActionValue& Value)
{
    CurrentInputTensor.FCU_Trigger_Main = Value.Get<bool>();
}

void AUnitController::OnReloadInput(const FInputActionValue& Value)
{
    CurrentInputTensor.FCU_Cycle_Feeder = true;
}

void AUnitController::OnInteractInput(const FInputActionValue& Value)
{
    if (!Value.Get<bool>()) { return; }
    if (!IsLocalController() || !ControlledUnit || !GetWorld()) { return; }

    const FVector Origin = ControlledUnit->GetActorLocation();
    const float Radius = 250.0f;

    TArray<FOverlapResult> Overlaps;
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(ControlledUnit);

    const FCollisionObjectQueryParams ObjParams(FCollisionObjectQueryParams::AllDynamicObjects);
    const bool bAny = GetWorld()->OverlapMultiByObjectType(
        Overlaps,
        Origin,
        FQuat::Identity,
        ObjParams,
        FCollisionShape::MakeSphere(Radius),
        Params);

    if (!bAny) { return; }

    AGroundWeaponModule* BestModule = nullptr;
    float BestDistSq = TNumericLimits<float>::Max();
    for (const FOverlapResult& O : Overlaps)
    {
        AGroundWeaponModule* Module = Cast<AGroundWeaponModule>(O.GetActor());
        if (!Module) { continue; }
        const float DistSq = FVector::DistSquared(Origin, Module->GetActorLocation());
        if (DistSq < BestDistSq) { BestDistSq = DistSq; BestModule = Module; }
    }

    if (BestModule)
    {
        BestModule->RequestEquip(ControlledUnit);
    }
}

void AUnitController::OnCycleWeaponInput(const FInputActionValue& Value)
{
    CurrentInputTensor.FCU_Station_Step = true;
}

void AUnitController::OnVerticalThrustInput(const FInputActionValue& Value)
{
    CurrentInputTensor.Vertical_CMD = Value.Get<bool>();
}

void AUnitController::OnBoostInput(const FInputActionValue& Value)
{
    CurrentInputTensor.Override_Power = Value.Get<bool>();
}

//==============================================================================
//                          network replication
//==============================================================================

bool AUnitController::Server_SendUnitInput_Validate(FInputTensor NewInput)
{
    return FMath::Abs(NewInput.Throttle_CMD) <= 1.1f && FMath::Abs(NewInput.Lateral_CMD) <= 1.1f && FMath::Abs(NewInput.MouseYaw_CMD) <= 1.1f;
}

void AUnitController::Server_SendUnitInput_Implementation(FInputTensor NewInput)
{
    if (ControlledUnit) { ControlledUnit->InputTensor = NewInput; } // End if (apply input)
}
