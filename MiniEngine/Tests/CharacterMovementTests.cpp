#include "GameFramework/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/CapsuleComponent.h"
#include "PhysicsWorld.h"
#include "../Model/Animation.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace
{
    using namespace PARS;
    using namespace Math;
    using namespace physx;

    void Check(bool value, const char* message)
    {
        if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
    }
    bool Near(float a, float b, float tolerance = 0.015f) { return std::abs(a - b) < tolerance; }

    // A character without GPU resources, exercising the production components.
    class TestCharacter : public Pawn
    {
    public:
        TestCharacter()
        {
            capsule = &CreateDefaultSubobject<CapsuleComponent>("Capsule");
            SetRootComponent(*capsule);
            movement = &CreateDefaultSubobject<CharacterMovementComponent>("Movement");
            movement->SetUpdatedComponent(*capsule);
        }
        void Tick(float dt) override
        {
            Pawn::Tick(dt);
            observedPosition = GetActorLocation();
            observedDelta = dt;
        }
        CapsuleComponent* capsule;
        CharacterMovementComponent* movement;
        Vector3 observedPosition = Vector3(kZero);
        float observedDelta = 0;
    };

    struct Fixture
    {
        World world;
        TestCharacter* character;
        PlayerController* player;
        Fixture()
        {
            Check(world.InitializePhysics(false), "Initialize shared physics scene");
            Check(world.GetPhysicsWorld()->CreateStaticBox(PxTransform(PxVec3(0, -.25f, 0)), PxVec3(100, .25f, 100), "Floor") != nullptr, "Create floor");
            character = world.SpawnActor<TestCharacter>();
            character->SetActorLocation(Vector3(0, 3, 0));
            player = world.SpawnActor<PlayerController>();
            Check(player->Possess(*character), "Possess character");
            world.BeginPlay();
            Check(character->capsule->HasPhysicsController(), "BeginPlay creates capsule controller");
            Frames(120);
            Check(character->movement->IsMovingOnGround(), "Gravity lands capsule on shared floor");
            Check(Near(character->GetActorLocation().GetY(), .92f, .03f), "Capsule rests at half height plus contact skin");
        }
        void Frames(int count, float dt = 1.0f / 60)
        {
            for (int i = 0; i < count; ++i) world.Tick(dt);
        }
        ~Fixture() { Check(world.GetPhysicsWorld()->GetErrorCount() == 0, "PhysX reported no errors"); }
    };

    float Travel(int fps, bool run, bool diagonal)
    {
        Fixture f;
        f.player->SetMoveInput(1, diagonal ? 1.0f : 0.0f, run);
        f.Frames(fps * 2, 1.0f / fps);
        const Vector3 position = f.character->GetActorLocation();
        Check(Near(f.character->movement->GetGroundSpeed(), run ? 3.5f : 1.6f), "Reach requested walk/run speed");
        Check(Near(float(Length(f.character->observedPosition - position)), 0), "PostPhysics observes resolved position");
        Check(f.character->movement->IsMovingOnGround(), "Remain grounded during movement");
        return Length(Vector3(position.GetX(), 0, position.GetZ()));
    }

    void TestMovement()
    {
        const float at30 = Travel(30, false, false);
        Check(Near(at30, Travel(60, false, false), .001f) && Near(at30, Travel(144, false, false), .001f), "Fixed movement independent of render FPS");
        Check(Near(at30, Travel(60, false, true), .005f), "Diagonal input cannot exceed straight movement speed");
        Check(Travel(60, true, false) > 6.3f && at30 > 3.0f, "Run covers greater distance than walk");

        Fixture f;
        f.player->SetControlYaw(3.14159265f);
        f.player->SetMoveInput(0, 1, false);
        f.Frames(60);
        Check(float(f.character->GetActorLocation().GetX()) > 1.4f, "D moves camera-right when looking toward -Z");
        Check(float(f.character->GetActorTransform().GetZ().GetX()) > .99f, "Character turns toward travel direction");
        f.player->SetInputEnabled(false);
        f.Frames(30);
        Check(f.character->movement->GetGroundSpeed() < .01f, "Free-camera mode stops input and brakes character");

        f.player->SetInputEnabled(true);
        f.player->SetMoveInput(1, 0, true);
        f.world.SetPhysicsPaused(true);
        const auto paused = f.character->GetActorLocation();
        const uint64_t steps = f.world.GetPhysicsWorld()->GetStepCount();
        f.Frames(30);
        Check(float(Length(f.character->GetActorLocation() - paused)) < .0001f && f.character->observedDelta == 0, "Pause freezes movement and animation time");
        f.world.RequestPhysicsStep();
        f.world.Tick(1.0f / 60);
        Check(f.world.GetPhysicsWorld()->GetStepCount() == steps + 1 && Near(f.character->observedDelta, 1.0f / 60, .0001f), "Single-step advances movement and animation together");
        f.world.SetPhysicsPaused(false);
        f.world.SetPhysicsTimeScale(.25f);
        f.Frames(60);
        Check(f.world.GetPhysicsWorld()->GetStepCount() == steps + 16, "Slow motion scales elapsed time, retaining fixed solver rate");
        f.player->SetMoveInput(std::numeric_limits<float>::quiet_NaN(), 0, false);
        f.Frames(120);
        Check(f.character->movement->GetGroundSpeed() < .01f, "Invalid input does not corrupt movement");
        f.world.SetPhysicsTimeScale(1);
        f.world.Tick(5.0f);
        Check(f.world.GetPhysicsWorld()->GetLastSubsteps() == 8 && Near(f.character->observedDelta, 8.0f / 60, .0001f), "Long stalls discard matching physics and animation time");
        std::puts("PASS: gravity, walk/run, facing, FPS independence, diagonal speed, pause/step/slow motion");
    }

    void TestCollisionAndLifetime()
    {
        Fixture f;
        auto physics = f.world.GetPhysicsWorld();
        physics->CreateStaticBox(PxTransform(PxVec3(0, 1, 2)), PxVec3(10, 1, .25f), "Wall");
        f.player->SetMoveInput(1, 0, true);
        f.Frames(120);
        Check(float(f.character->GetActorLocation().GetZ()) > 1.35f && float(f.character->GetActorLocation().GetZ()) < 1.48f, "Capsule stops before wall");
        Check(f.character->movement->GetGroundSpeed() < .05f, "Blocked speed returns to idle");
        f.player->SetMoveInput(1, 1, false);
        f.Frames(120);
        Check(float(f.character->GetActorLocation().GetX()) < -1.0f && float(f.character->GetActorLocation().GetZ()) < 1.48f, "Diagonal movement slides along wall");

        auto other = f.world.SpawnActor<TestCharacter>();
        other->SetActorLocation(Vector3(3, 2, 0));
        f.Frames(60);
        Check(physics->GetControllerCount() == 2 && other->movement->IsMovingOnGround(), "Multiple actors get independent capsules in one scene");
        const auto oldHandle = f.character->GetHandle();
        f.character->Destroy();
        Check(f.player->GetPawn() == nullptr, "Possession handle stops resolving immediately on destroy");
        Check(f.player->Possess(*other), "Possess another actor before old pawn is reclaimed");
        f.world.CollectGarbage();
        Check(physics->GetControllerCount() == 1 && !f.world.FindActor(oldHandle), "EndPlay releases deleted capsule");
        Check(f.player->GetPawn() == other && other->GetController() == f.player, "Retired pawn cannot unpossess its replacement");
        Check(f.player->Possess(*other) && other->GetController() == f.player, "Controller can possess replacement pawn");
        f.player->Destroy();
        f.world.CollectGarbage();
        Check(other->GetController() == nullptr, "Controller deletion clears possession safely");
        f.world.Clear();
        Check(physics->GetControllerCount() == 0, "World clears all capsules before physics shutdown");
        std::puts("PASS: wall collision/sliding, multiple capsules, possession and deferred destruction");
    }

    void TestPoseBlending()
    {
        GraphNode from{};
        from.xform = Matrix4(kIdentity);
        from.rotation = Quaternion(kIdentity);
        from.scale = XMFLOAT3(1, 1, 1);
        GraphNode to = from;
        to.xform.SetW(Vector4(2, 4, 6, 1));
        to.rotation = Quaternion(Vector3(kYUnitVector), 1.57079633f);
        to.scale = XMFLOAT3(3, 3, 3);
        to.matrixIdx = 17;
        const auto end = to;
        BlendAnimationPoses(&from, &to, 1, .5f);
        Check(Near(to.xform.GetW().GetX(), 1) && Near(to.xform.GetW().GetY(), 2) && Near(to.scale.x, 2), "Blend translation and scale");
        Check(Near((to.rotation * Vector3(kZUnitVector)).GetX(), .70710678f) && to.staleMatrix && to.matrixIdx == 17, "Slerp rotation, preserve hierarchy, invalidate matrix");
        to = end;
        BlendAnimationPoses(&from, &to, 1, 0);
        Check(Near(to.xform.GetW().GetX(), 0) && Near(to.scale.x, 1), "Blend starts at exact displayed pose");
        to = end;
        BlendAnimationPoses(&from, &to, 1, 1);
        Check(Near(to.xform.GetW().GetX(), 2) && Near(to.scale.x, 3), "Blend finishes at exact target pose");
        std::puts("PASS: pose blend endpoints, quaternion/translation/scale interpolation");
    }
}

void RunCharacterMovementTests()
{
    TestMovement();
    TestCollisionAndLifetime();
    TestPoseBlending();
}
