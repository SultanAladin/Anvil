#include "ImpactProjectile.h"

#include "Components/StaticMeshComponent.h"
#include "GameFramework/ProjectileMovementComponent.h"
#include "Kismet/GameplayStatics.h"

AImpactProjectile::AImpactProjectile()
{
    PrimaryActorTick.bCanEverTick = false;

    ProjectileMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ProjectileMesh"));
    SetRootComponent(ProjectileMesh);

    ProjectileMesh->SetSimulatePhysics(false);
    ProjectileMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    ProjectileMesh->SetCollisionObjectType(ECC_WorldDynamic);
    ProjectileMesh->SetNotifyRigidBodyCollision(true);

    ProjectileMesh->SetGenerateOverlapEvents(false);

    ProjectileMesh->OnComponentHit.AddDynamic(this, &AImpactProjectile::OnProjectileHit);

    Movement = CreateDefaultSubobject<UProjectileMovementComponent>(TEXT("Movement"));
    Movement->UpdatedComponent = ProjectileMesh;
    Movement->bAutoActivate = false;
    Movement->ProjectileGravityScale = 0.0f;
    Movement->bRotationFollowsVelocity = true;
    Movement->InitialSpeed = 0.0f;
    Movement->MaxSpeed = 25000.0f;

    InitialLifeSpan = LifeTime_s;
}

void AImpactProjectile::BeginPlay()
{
    Super::BeginPlay();
    InitialLifeSpan = LifeTime_s;
}

void AImpactProjectile::SetInitialVelocity(const FVector& Velocity_cms)
{
    if (!Movement) { return; }
    Movement->InitialSpeed = Velocity_cms.Size();
    Movement->Velocity = Velocity_cms;
    Movement->Activate(true);
}

void AImpactProjectile::SetIgnoredActor(AActor* ActorToIgnore)
{
    if (!ProjectileMesh || !ActorToIgnore) { return; }
    ProjectileMesh->IgnoreActorWhenMoving(ActorToIgnore, true);
}

void AImpactProjectile::OnProjectileHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
    if (!OtherActor || OtherActor == GetOwner()) { return; }

    if (ImpactDecals.Num() > 0)
    {
        const int32 Index = FMath::RandHelper(ImpactDecals.Num());
        if (ImpactDecals[Index])
        {
            UGameplayStatics::SpawnDecalAtLocation(GetWorld(), ImpactDecals[Index], FVector(ImpactDecalSize_cm), Hit.ImpactPoint, Hit.ImpactNormal.Rotation(), 10.0f);
        }
    }

    Destroy();
}
