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
            Check(character->capsule->HasPhysicsCollision(), "BeginPlay registers query-only capsule");
            Check(world.GetPhysicsWorld()->GetDynamicBodyCount() == 0, "Movement capsule creates no rigid body to obscure PA in PVD");
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
        Check(physics->GetQueryCapsuleCount() == 2 && other->movement->IsMovingOnGround(), "Multiple actors get independent capsules in one scene");
        const auto oldHandle = f.character->GetHandle();
        f.character->Destroy();
        Check(f.player->GetPawn() == nullptr, "Possession handle stops resolving immediately on destroy");
        Check(f.player->Possess(*other), "Possess another actor before old pawn is reclaimed");
        f.world.CollectGarbage();
        Check(physics->GetQueryCapsuleCount() == 1 && !f.world.FindActor(oldHandle), "EndPlay releases deleted capsule");
        Check(f.player->GetPawn() == other && other->GetController() == f.player, "Retired pawn cannot unpossess its replacement");
        Check(f.player->Possess(*other) && other->GetController() == f.player, "Controller can possess replacement pawn");
        f.player->Destroy();
        f.world.CollectGarbage();
        Check(other->GetController() == nullptr, "Controller deletion clears possession safely");
        f.world.Clear();
        Check(physics->GetQueryCapsuleCount() == 0, "World clears all capsules before physics shutdown");
        std::puts("PASS: wall collision/sliding, multiple capsules, possession and deferred destruction");
    }

    void TestQueryCapsules()
    {
        Fixture f;
        auto* physics = f.world.GetPhysicsWorld();
        auto* capsule = f.character->capsule;
        // The line geometry is in metres, +Y-up, and must not add scene actors.
        std::vector<PxDebugLine> lines;
        physics->GetCapsuleDebugLines(lines);
        Check(!lines.empty(), "Query capsule produces debug lines");
        float minY = 100, maxY = -100, maxRadius = 0;
        const auto centre = f.character->GetActorLocation();
        for (const auto& line : lines)
            for (const auto& point : {line.pos0, line.pos1})
            {
                minY = PxMin(minY, point.y); maxY = PxMax(maxY, point.y);
                maxRadius = PxMax(maxRadius, std::sqrt(point.x * point.x + point.z * point.z));
            }
        Check(Near(maxY - minY, 1.8f, .001f) && Near((minY + maxY) * .5f, centre.GetY(), .001f) &&
            Near(maxRadius, .3f, .001f), "Wire capsule dimensions and +Y axis match collision geometry");
        physics->SetCapsuleDebugVisible(false); physics->GetCapsuleDebugLines(lines);
        Check(lines.empty() && capsule->HasPhysicsCollision(), "Hiding the outline preserves movement collision");
        physics->SetCapsuleDebugVisible(true);

        // A 20 cm ledge is reachable, while a tall wall remains blocking.
        auto* ledge = physics->CreateStaticBox(PxTransform(PxVec3(0, .1f, 2)), PxVec3(2, .1f, 1), "Step");
        f.player->SetMoveInput(1, 0, false); f.Frames(90);
        Check(float(f.character->GetActorLocation().GetZ()) > 1.8f &&
            Near(f.character->GetActorLocation().GetY(), 1.12f, .015f), "Query movement climbs a low step");
        f.Frames(120);
        Check(Near(f.character->GetActorLocation().GetY(), .92f, .015f) && f.character->movement->IsMovingOnGround(),
            "Leaving a ledge returns to the floor");
        f.player->SetMoveInput(0, 0, false); f.Frames(30); ledge->release();

        auto* ceiling = physics->CreateStaticBox(PxTransform(PxVec3(0, 2.5f, 0)), PxVec3(2, .25f, 2), "Ceiling");
        f.character->SetActorLocation(Vector3(0, .92f, 0));
        Check(capsule->Move(Vector3(0, 5, 0), 1.0f / 60).ceiling && float(f.character->GetActorLocation().GetY()) < 1.34f,
            "Upward query movement cannot tunnel through a ceiling");
        ceiling->release();
        f.character->SetActorLocation(Vector3(0, .7f, 0));
        capsule->Move(Vector3(kZero), 1.0f / 60);
        Check(float(f.character->GetActorLocation().GetY()) > .919f, "Teleport penetration recovers from the floor");
        f.character->SetActorLocation(Vector3(0, 10, 0));
        Check(capsule->Move(Vector3(0, -50, 0), 1.0f / 60).grounded &&
            Near(f.character->GetActorLocation().GetY(), .92f, .001f), "Long downward sweep cannot tunnel through the floor");

        auto* other = f.world.SpawnActor<TestCharacter>();
        other->SetActorLocation(Vector3(0, .92f, 2)); f.Frames(1);
        capsule->Move(Vector3(0, 5, 10), 1.0f / 60); // Horizontal pass must stop at the other capsule.
        Check(float(f.character->GetActorLocation().GetZ()) < 1.4f && physics->GetDynamicBodyCount() == 0,
            "Two characters block each other without CCT proxy actors");
        Check(capsule->SetPhysicsEnabled(false) && physics->GetQueryCapsuleCount() == 1,
            "Disabling query collision removes its debug geometry and other-character blocker");
        Check(capsule->SetPhysicsEnabled(true) && physics->GetDynamicBodyCount() == 0,
            "Re-enabling movement never creates a rigid capsule");
        std::puts("PASS: query-only capsule, wire outline, steps, ceiling, depenetration, long sweeps and character collision");
    }

    void TestSkeletonHierarchy()
    {
        // Two scene roots, nested children and siblings. Joint indices need not
        // be in this order; non-joint armatures must remain in the parent chain.
        GraphNode graph[7]{};
        graph[0].hasChildren = graph[0].hasSibling = true;
        graph[1].hasChildren = graph[1].hasSibling = true;
        graph[2].hasSibling = true;
        graph[4].hasChildren = true;
        std::vector<int32_t> parents;
        Check(BuildSceneGraphParents(graph, 7, parents), "Recover skeleton hierarchy with multiple scene roots");
        Check(parents == std::vector<int32_t>({-1, 0, 1, 1, 0, 4, -1}), "Preserve helper nodes and nested sibling parents");
        Check(!BuildSceneGraphParents(graph, 6, parents) && parents.empty(), "Reject a truncated scene hierarchy");
        graph[0].hasChildren = graph[0].hasSibling = false;
        Check(!BuildSceneGraphParents(graph, 7, parents) && parents.empty(), "Reject unreachable trailing nodes");
        std::vector<GraphNode> deep(96);
        for (size_t i = 0; i + 1 < deep.size(); ++i) deep[i].hasChildren = true;
        Check(BuildSceneGraphParents(deep.data(), static_cast<uint32_t>(deep.size()), parents) && parents.back() == 94,
            "Hierarchy inspection does not truncate deep chains");
        Check(BuildSceneGraphParents(nullptr, 0, parents) && parents.empty(), "Empty skeleton is valid");
        Check(!BuildSceneGraphParents(nullptr, 1, parents), "Reject missing node storage");
        std::puts("PASS: skeleton hierarchy, helper nodes, multiple roots and malformed data");
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
    TestQueryCapsules();
    TestPoseBlending();
    TestSkeletonHierarchy();
}
