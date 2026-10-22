#include "../Core/GameFramework/Actor.h"
#include "../Core/GameFramework/SceneComponent.h"
#include "../Core/GameFramework/CapsuleComponent.h"
#include "../Core/GameFramework/Pawn.h"
#include "../Model/Animation.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

using namespace PARS;
using namespace Math;

namespace
{
    void Check(bool value, const char* message)
    {
        if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
    }
    bool Near(Vector3 a, Vector3 b) { return float(Length(a - b)) < 1e-4f; }

    class LifecycleProbe : public ActorComponent
    {
    public:
        LifecycleProbe(Actor& owner, const std::string& name, std::vector<std::string>& events)
            : ActorComponent(owner, name), m_Events(events) {}
    protected:
        void BeginPlay() override { m_Events.push_back(GetName() + ":begin"); }
        void EndPlay() override { m_Events.push_back(GetName() + ":end"); }
        void TickComponent(float) override { m_Events.push_back(GetName() + ":tick"); }
    private:
        std::vector<std::string>& m_Events;
    };

    void TestHierarchy()
    {
        Actor actor("Hierarchy");
        Check(!actor.SetActorLocation(Vector3(1, 2, 3)), "Actor without root cannot store an independent location");
        auto& root = actor.CreateDefaultSubobject<SceneComponent>("Root");
        auto& child = actor.CreateDefaultSubobject<SceneComponent>("Child");
        auto& leaf = actor.CreateDefaultSubobject<SceneComponent>("Leaf");
        Check(actor.SetRootComponent(root), "Set root");
        Check(child.AttachToComponent(root) && leaf.AttachToComponent(child), "Attach descendants");
        Check(!root.AttachToComponent(child), "Root cannot be attached below a child");
        Check(!child.AttachToComponent(leaf), "Reject cycles");
        Check(!child.AttachToComponent(child), "Reject self attachment");
        Actor other;
        auto& foreign = other.CreateDefaultSubobject<SceneComponent>("Foreign");
        Check(!child.AttachToComponent(foreign), "Reject foreign-owner attachment");
        Check(!actor.SetRootComponent(foreign), "Reject foreign root");
        SceneComponent external(actor, "Unowned");
        Check(!actor.SetRootComponent(external) && !child.AttachToComponent(external), "Reject unowned component lifetimes");

        root.SetRelativeTransform(AffineTransform(Matrix3(Quaternion(Vector3(kYUnitVector), 1.57079632679f)) *
            Matrix3::MakeScale(Vector3(2, 3, 4)), Vector3(10, 20, 30)));
        child.SetRelativeLocation(Vector3(1, 2, 3));
        Check(Near(child.GetComponentTransform().GetTranslation(), Vector3(22, 26, 28)), "Parent rotation and non-uniform scale affect child location");
        Check(Near(actor.GetActorLocation(), root.GetComponentTransform().GetTranslation()), "Actor delegates transform to root");

        const AffineTransform wanted(Matrix3::MakeScale(Vector3(1, 2, 1)), Vector3(-3, 4, 5));
        Check(child.SetWorldTransform(wanted), "World-to-relative conversion");
        Check(Near(child.GetComponentTransform() * Vector3(1, 1, 1), wanted * Vector3(1, 1, 1)), "World matrix round trip");
        child.DetachFromParent();
        Check(Near(child.GetComponentTransform().GetTranslation(), Vector3(-3, 4, 5)), "Detach preserves world transform");
        Check(child.AttachToComponent(root, AttachmentRule::KeepWorld), "Reattach preserving world");
        Check(Near(child.GetComponentTransform() * Vector3(1, 1, 1), wanted * Vector3(1, 1, 1)), "Reattach preserves world matrix");
        root.SetRelativeTransform(AffineTransform::MakeScale(0.0f));
        Check(!child.SetWorldTransform(wanted), "Reject inverse of singular parent transform");

        bool duplicateRejected = false;
        try { actor.CreateDefaultSubobject<SceneComponent>("Root"); }
        catch (const std::invalid_argument&) { duplicateRejected = true; }
        Check(duplicateRejected, "Reject duplicate subobject names");
        std::puts("PASS: root ownership, attachments, cycle rejection and transforms");
    }

    void TestLifecycle()
    {
        std::vector<std::string> events;
        {
            Actor actor;
            actor.CreateDefaultSubobject<LifecycleProbe>("A", events);
            auto& b = actor.CreateDefaultSubobject<LifecycleProbe>("B", events);
            actor.Tick(1);
            Check(events.empty(), "No component ticks before BeginPlay");
            actor.BeginPlay();
            actor.BeginPlay();
            Check(events.size() == 2, "BeginPlay exactly once");
            b.SetTickEnabled(false);
            actor.Tick(0.1f);
            Check(events.size() == 3 && events.back() == "A:tick", "Enabled component ticks");
            actor.Tick(std::numeric_limits<float>::quiet_NaN());
            Check(events.size() == 3, "Reject invalid tick delta");
            bool rejected = false;
            try { actor.CreateDefaultSubobject<SceneComponent>("Late"); }
            catch (const std::logic_error&) { rejected = true; }
            Check(rejected, "Construction-only subobjects cannot mutate a running component list");
        }
        Check(events.size() == 5 && events[3] == "B:end" && events[4] == "A:end", "Reverse EndPlay on destruction");
        {
            Actor actor;
            auto& child = actor.CreateDefaultSubobject<SceneComponent>("EarlyChild");
            auto& parent = actor.CreateDefaultSubobject<SceneComponent>("LateParent");
            actor.SetRootComponent(parent);
            child.AttachToComponent(parent);
        }
        Pawn pawn;
        pawn.AddMovementInput(Vector3(1, 0, 0), 2);
        pawn.AddMovementInput(Vector3(0, 0, 1));
        Check(Near(pawn.ConsumeMovementInput(), Vector3(2, 0, 1)), "Pawn accumulates movement input");
        Check(Near(pawn.ConsumeMovementInput(), Vector3(kZero)), "Pawn consumes input once");
        auto& capsule = pawn.CreateDefaultSubobject<CapsuleComponent>("Capsule");
        Check(capsule.SetCapsuleSize(0.3f, 0.9f), "Capsule dimensions");
        Check(!capsule.SetCapsuleSize(1, 0.5f) && !capsule.SetCapsuleSize(-1, 1), "Reject impossible capsule geometry");
        Check(std::abs(capsule.GetCylinderHalfHeight() - 0.6f) < 1e-6f, "Half height includes end caps");
        std::puts("PASS: component lifecycle, ownership and capsule geometry");
    }

    void TestPoseSampling()
    {
        GraphNode pose{};
        pose.xform = Matrix4(kIdentity);
        pose.rotation = Quaternion(kIdentity);
        pose.scale = XMFLOAT3(1, 1, 1);
        AnimationSet clip{ 1.0f, 0, 1 };
        AnimationCurve curve{};
        curve.targetPath = AnimationCurve::kTranslation;
        curve.keyFrameFormat = AnimationCurve::kFloat;
        curve.keyFrameStride = 3;
        curve.numSegments = 1;
        curve.rangeScale = 1;
        const float nan = std::numeric_limits<float>::quiet_NaN();
        float keys[] = { 0, 0, 0, 2, 4, 6, nan, nan, nan };
        auto sample = [&](float time) { return EvaluateAnimationPose(clip, &curve, reinterpret_cast<uint8_t*>(keys), &pose, 1, time); };
        Check(sample(0.5f) && Near(Vector3(pose.xform.GetW()), Vector3(1, 2, 3)), "Interpolate translation");
        Check(sample(1.0f) && Near(Vector3(pose.xform.GetW()), Vector3(2, 4, 6)), "Exact last key must not read next poisoned key");
        Check(sample(2.0f) && Near(Vector3(pose.xform.GetW()), Vector3(2, 4, 6)), "Clamp after clip end");
        Check(sample(-1.0f) && Near(Vector3(pose.xform.GetW()), Vector3(kZero)), "Clamp before clip start");
        Check(!sample(nan), "Reject NaN animation time");
        curve.targetNode = 1;
        Check(!sample(0.5f), "Reject pose target outside skeleton");
        curve.targetNode = 0;
        curve.targetPath = AnimationCurve::kScale;
        Check(sample(0.5f) && Near(Vector3(pose.scale), Vector3(1, 2, 3)) && pose.staleMatrix, "Scale interpolation invalidates matrix");
        curve.targetPath = AnimationCurve::kRotation;
        curve.keyFrameStride = 4;
        const float rotationKeys[] = { 0, 0, 0, 1, 0, 1, 0, 0 };
        Check(EvaluateAnimationPose(clip, &curve, reinterpret_cast<const uint8_t*>(rotationKeys), &pose, 1, 0.5f), "Sample quaternion rotation");
        Check(Near(pose.rotation * Vector3(0, 0, 1), Vector3(1, 0, 0)), "Quaternion interpolation takes halfway rotation");
        std::puts("PASS: pose interpolation and clip endpoint safety");
    }
}

void RunWorldTests();
void RunCharacterMovementTests();
void RunPhysicsAssetTests();
void RunRagdollTests();
void RunAnatomicalLimitTests();
void RunActiveRagdollTests();
void RunPelvisDriveTests();
void RunRagdollComparisonTests();
void RunSpringArmTests();

int main(int argc, char** argv)
{
    if (argc == 2 && std::strcmp(argv[1], "--pelvis-only") == 0)
    {
        RunPelvisDriveTests();
        return 0;
    }
    TestHierarchy();
    TestLifecycle();
    TestPoseSampling();
    RunWorldTests();
    RunCharacterMovementTests();
    RunSpringArmTests();
    RunPhysicsAssetTests();
    RunRagdollTests();
    RunAnatomicalLimitTests();
    RunActiveRagdollTests();
    RunRagdollComparisonTests();
    return 0;
}
