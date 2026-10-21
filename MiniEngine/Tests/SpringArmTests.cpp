#include "GameFramework/World.h"
#include "GameFramework/SpringArmComponent.h"
#include "PhysicsWorld.h"
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
        if (!value)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    bool Near(Vector3 a, Vector3 b, float tolerance = .001f)
    {
        return float(Length(a - b)) < tolerance;
    }

    class CameraRig : public Actor
    {
    public:
        CameraRig()
        {
            auto& root = CreateDefaultSubobject<SceneComponent>("Root");
            SetRootComponent(root);
            arm = &CreateDefaultSubobject<SpringArmComponent>("SpringArm");
            Check(arm->AttachToComponent(root), "Attach spring arm to the actor root");
        }
        void FixedTick(float deltaTime) override
        {
            if (move)
            {
                SetActorLocation(GetActorLocation() + Vector3(deltaTime, 0, 0));
            }
            Actor::FixedTick(deltaTime);
        }
        SpringArmComponent* arm;
        bool move = false;
    };

    struct Fixture
    {
        World world;
        CameraRig* rig;
        PhysicsWorld* physics;
        Fixture()
        {
            Check(world.InitializePhysics(false), "Initialize spring arm physics scene");
            physics = world.GetPhysicsWorld();
            rig = world.SpawnActor<CameraRig>();
            rig->SetActorLocation(Vector3(0, 2, 0));
            world.BeginPlay();
            world.Tick(0);
        }
        ~Fixture()
        {
            Check(physics->GetErrorCount() == 0, "Spring arm queries produce no PhysX errors");
        }
    };

    void TestAttachmentAndInput()
    {
        Fixture f;
        auto& arm = *f.rig->arm;
        Check(Near(arm.GetCameraLocation(), Vector3(0, 2, -4.5f)), "Unobstructed arm starts at its full length");
        auto settings = arm.GetSettings();
        settings.cameraLagSpeed = settings.armReturnSpeed = 0;
        Check(arm.SetSettings(settings), "Configure immediate follow");
        Check(arm.SetControlRotation(DirectX::XM_PIDIV2, 0), "Set world-space control rotation");
        f.rig->SetActorTransform(AffineTransform(Matrix3::MakeYRotation(DirectX::XM_PI), Vector3(2, 3, 4)));
        arm.SetRelativeLocation(Vector3(0, .55f, 0));
        f.world.Tick(0);
        Check(Near(arm.GetCameraLocation(), Vector3(-2.5f, 3.55f, 4)), "Attached pivot moves with actor without inheriting movement-facing yaw");
        f.rig->move = true;
        f.world.Tick(1.0f / 60);
        Check(Near(arm.GetCameraLocation(), f.rig->GetActorLocation() + Vector3(-4.5f, .55f, 0)),
            "Post-physics component tick follows this frame's resolved actor position");
        Check(f.physics->GetDynamicBodyCount() == 0, "Spring arm owns no physics proxy bodies");
        const auto camera = arm.GetCameraLocation();
        const float nan = std::numeric_limits<float>::quiet_NaN();
        arm.UpdateArm(nan);
        arm.UpdateArm(-1);
        Check(!arm.SetControlRotation(nan, 0) && Near(camera, arm.GetCameraLocation()), "Invalid camera inputs preserve the last valid view");
        Check(arm.SetControlRotation(0, DirectX::XM_PIDIV2) &&
                float(LengthSquare(Cross(arm.GetViewDirection(), Vector3(kYUnitVector)))) > 1e-6f,
            "Extreme pitch preserves a non-degenerate camera basis");
        settings.probeRadius = 0;
        Check(!arm.SetSettings(settings), "Reject a degenerate camera probe");
        std::puts("PASS: spring arm attachment, world control rotation, post-physics timing and input validation");
    }

    void TestCollisionFiltering()
    {
        Fixture f;
        auto* self = f.physics->CreateDynamicBox(PxTransform(PxVec3(0, 2, -.3f)), PxVec3(.5f), 1, "Own ragdoll body");
        self->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        self->userData = f.rig;
        auto* capsule = f.physics->CreateQueryCapsule(PxVec3(0, 2, 0), .3f, .9f);
        capsule->owner = f.rig;
        auto* trigger = f.physics->CreateStaticBox(PxTransform(PxVec3(0, 2, -1)), PxVec3(.4f), "Trigger");
        PxShape* shape = nullptr;
        trigger->getShapes(&shape, 1);
        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
        shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
        f.world.Tick(0);
        Check(!f.rig->arm->IsCollisionFixApplied() && std::abs(f.rig->arm->GetCurrentArmLength() - 4.5f) < .001f,
            "Ignore the owning capsule, owning ragdoll bodies and triggers");
        f.physics->CreateStaticBox(PxTransform(PxVec3(0, 2, -2)), PxVec3(2, 2, .1f), "Wall");
        f.world.Tick(0);
        Check(f.rig->arm->IsCollisionFixApplied() && std::abs(f.rig->arm->GetCurrentArmLength() - 1.68f) < .003f,
            "Sphere radius and collision padding keep the camera in front of the wall");
        auto* other = f.physics->CreateQueryCapsule(PxVec3(0, 2, -1), .3f, .9f);
        f.world.Tick(0);
        Check(std::abs(f.rig->arm->GetCurrentArmLength() - .48f) < .003f, "Other characters' query-only capsules still obstruct the camera");
        f.physics->DestroyQueryCapsule(other);
        f.physics->DestroyQueryCapsule(capsule);
        auto* dynamic = f.physics->CreateDynamicBox(PxTransform(PxVec3(0, 2, -.8f)), PxVec3(.1f), 1, "Moving obstacle");
        dynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        f.world.Tick(0);
        Check(std::abs(f.rig->arm->GetCurrentArmLength() - .48f) < .003f, "Dynamic obstacles participate in camera sweeps");
        std::puts("PASS: spring arm static/dynamic obstacles, self filtering, triggers and query-only capsules");
    }

    void TestRecoveryAndOverlap()
    {
        Fixture f;
        auto& arm = *f.rig->arm;
        auto* wall = f.physics->CreateStaticBox(PxTransform(PxVec3(0, 2, -2)), PxVec3(2, 2, .1f), "Wall");
        f.world.Tick(1.0f / 60);
        wall->setGlobalPose(PxTransform(PxVec3(0, 2, -1)));
        f.world.Tick(1.0f / 144);
        Check(std::abs(arm.GetCurrentArmLength() - .68f) < .003f, "An approaching wall retracts the arm in the same frame");
        wall->release();
        f.world.Tick(1.0f / 60);
        Check(arm.GetCurrentArmLength() > .68f && arm.GetCurrentArmLength() < 2, "Removing obstruction restores distance smoothly");
        float previous = arm.GetCurrentArmLength();
        for (int i = 0; i < 120; ++i)
        {
            f.world.Tick(1.0f / 60);
            Check(arm.GetCurrentArmLength() >= previous && arm.GetCurrentArmLength() <= 4.5001f, "Restoration is monotonic without overshoot");
            previous = arm.GetCurrentArmLength();
        }
        Check(std::abs(previous - 4.5f) < .001f, "Restored arm reaches its configured length");
        auto* overlap = f.physics->CreateStaticBox(PxTransform(PxVec3(0, 2, 0)), PxVec3(.1f), "Overlapping pivot");
        f.world.Tick(0);
        Check(arm.GetCurrentArmLength() == 0 && Near(arm.GetCameraLocation(), f.rig->GetActorLocation()) &&
                std::abs(float(Length(arm.GetViewDirection())) - 1) < .001f,
            "Initial overlap collapses the arm without a degenerate look direction");
        overlap->release();
        auto settings = arm.GetSettings();
        settings.targetArmLength = 0;
        Check(arm.SetSettings(settings), "Allow a zero-length arm");
        f.world.Tick(0);
        Check(Near(arm.GetCameraLocation(), f.rig->GetActorLocation()), "Zero-length arm remains finite");
        std::puts("PASS: spring arm immediate retraction, smooth recovery and initial overlap");
    }

    float LagAt(int fps)
    {
        Fixture f;
        auto settings = f.rig->arm->GetSettings();
        settings.cameraLagSpeed = 4;
        settings.maxCameraLagDistance = 10;
        settings.armReturnSpeed = 0;
        Check(f.rig->arm->SetSettings(settings), "Configure camera lag");
        f.rig->SetActorLocation(Vector3(2, 2, 0));
        f.world.Tick(1.0f / fps);
        Check(float(f.rig->arm->GetCameraLocation().GetX()) > 0 && float(f.rig->arm->GetCameraLocation().GetX()) < 1,
            "Camera follows target displacement gradually");
        for (int frame = 1; frame < fps; ++frame)
        {
            f.world.Tick(1.0f / fps);
        }
        return f.rig->arm->GetCameraLocation().GetX();
    }

    void TestLagAndGrazing()
    {
        const float at30 = LagAt(30);
        Check(std::abs(at30 - LagAt(60)) < 1e-5f && std::abs(at30 - LagAt(144)) < 1e-5f,
            "Settling after the same target displacement is independent of render FPS");
        Fixture f;
        auto& arm = *f.rig->arm;
        auto* edge = f.physics->CreateStaticBox(PxTransform(PxVec3(.18f, 2, -2)), PxVec3(.03f, 1, .1f), "Grazing edge");
        f.world.Tick(0);
        Check(arm.IsCollisionFixApplied() && arm.GetCurrentArmLength() < 2,
            "Sphere probe catches edges that a center ray would miss");
        edge->release();
        auto settings = arm.GetSettings();
        settings.maxCameraLagDistance = 10;
        arm.SetSettings(settings);
        arm.ResetLagState();
        f.world.Tick(0);
        f.physics->CreateStaticBox(PxTransform(PxVec3(1, 2, 0)), PxVec3(.05f, 2, 10), "Lag crossing wall");
        f.rig->SetActorLocation(Vector3(2, 2, 0));
        f.world.Tick(0);
        Check(arm.IsCollisionFixApplied() && float(arm.GetCameraLocation().GetX()) >= 1.249f,
            "Lagged target cannot drag the camera through a wall behind the current pivot");
        f.rig->SetActorLocation(Vector3(100, 2, 0));
        f.world.Tick(0);
        Check(Near(arm.GetCameraLocation(), Vector3(100, 2, -4.5f)), "Teleport discards stale lag and obstruction history");
        f.rig->SetActorLocation(Vector3(102, 2, 0));
        arm.ResetLagState();
        f.world.Tick(0);
        Check(Near(arm.GetCameraLocation(), Vector3(102, 2, -4.5f)), "Explicit reset restores the camera when changing viewing modes");
        std::puts("PASS: spring arm camera lag, frame-rate stability, grazing collisions and reset");
    }
}

void RunSpringArmTests()
{
    TestAttachmentAndInput();
    TestCollisionFiltering();
    TestRecoveryAndOverlap();
    TestLagAndGrazing();
}
