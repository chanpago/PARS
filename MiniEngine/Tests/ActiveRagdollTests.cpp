#include "../Model/Ragdoll.h"
#include "../Model/PhysicsPose.h"
#include "../Core/PhysicsWorld.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

using namespace Math;
using namespace PARS;
using namespace physx;
namespace
{
    void Check(bool value, const char* message)
    { if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); } }
    float Difference(Quaternion a, Quaternion b)
    { return 2 * std::acos((std::min)(1.0f, std::abs(XMVectorGetX(XMVector4Dot(a, b))))); }
    Matrix4 Pose(Vector3 position, Quaternion rotation = Quaternion(kIdentity)) { return Matrix4(OrthogonalTransform(rotation, position)); }

    std::vector<float> SampleAccelerationResponse(float massScale, float radius, bool slerp, float maxTorque, bool rotatedFrames = false)
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize acceleration drive world");
        auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -10, 0)), PxVec3(1), "floor");
        Check(floor != nullptr, "Create acceleration test scene");
        PhysicsAsset asset;
        PhysicsBody parent;
        parent.bone = "parent";
        parent.shape = PhysicsShapeType::Sphere;
        parent.radius = radius;
        parent.mass = massScale;
        PhysicsBody child = parent;
        child.bone = "child";
        child.mass *= 2.0f;
        asset.bodies = {parent, child};
        PhysicsConstraint joint;
        joint.parentBone = parent.bone;
        joint.childBone = child.bone;
        joint.twist = PhysicsJointMotion::Free;
        joint.swingY = joint.swingZ = slerp ? PhysicsJointMotion::Free : PhysicsJointMotion::Locked;
        if (rotatedFrames)
        {
            XMStoreFloat4(&joint.parentFrame.rotation, Quaternion(Vector3(kYUnitVector), .7f));
            joint.childFrame.rotation = joint.parentFrame.rotation;
        }
        asset.constraints.push_back(joint);
        ConfigureDefaultDrives(asset);
        for (auto& drive : asset.constraints[0].drives)
        {
            drive.maxTorque = maxTorque;
        }
        const std::vector<std::string> names{parent.bone, child.bone};
        const std::vector<Matrix4> initial{Pose(Vector3(0, 3, 0)), Pose(Vector3(0, 3, 0))};
        Ragdoll ragdoll;
        std::string error;
        Check(ragdoll.Create(world, asset, names, initial, Vector3(kZero), error), "Create acceleration drive pair");
        PxArticulationReducedCoordinate* articulation = nullptr;
        Check(ragdoll.UsesArticulation() && floor->getScene()->getArticulations(&articulation, 1) == 1,
            "Acceleration PA uses an articulation");
        PxArticulationLink* links[2]{};
        Check(articulation->getLinks(links, 2) == 2, "Find acceleration test links");
        PxRigidBody* childBody = nullptr;
        for (auto* body : links)
        {
            body->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
            body->setAngularDamping(0);
            body->setLinearDamping(0);
            if (std::strcmp(body->getName(), "child") == 0)
            {
                childBody = body;
            }
        }
        Check(childBody != nullptr, "Find child inertia for torque bound");
        // 질량과 형상만 바꾸고 외력, 바디 감쇠, 관절 오프셋의 영향을 분리해 응답을 비교한다.
        auto target = initial;
        target[1] = Pose(Vector3(0, 3, 0), Quaternion(Vector3(kXUnitVector), .5f));
        Check(ragdoll.UpdateDriveTargets(target, 0, error), "Set acceleration target without a velocity spike");
        ragdoll.SetDrivesEnabled(true);
        std::vector<float> errors;
        for (int step = 0; step < 60; ++step)
        {
            Check(ragdoll.UpdateDriveTargets(target, 1.0f / 60, error), "Update acceleration target");
            world.Step();
            Check(ragdoll.IsFinite(), "Acceleration drive remains finite");
            errors.push_back(ragdoll.GetMaxDriveError());
            if (step == 0)
            {
                const float velocityBound = maxTorque / childBody->getMassSpaceInertiaTensor().x / 60.0f;
                Check(std::abs(childBody->getAngularVelocity().x) <= velocityBound * 1.02f + 1e-5f,
                    "Acceleration mode keeps its torque limit in Nm");
            }
        }
        auto changed = asset;
        changed.constraints[0].drives[0].naturalFrequency = 8.0f;
        changed.constraints[0].drives[0].dampingRatio = .7f;
        const auto before = childBody->getGlobalPose();
        const auto velocity = childBody->getAngularVelocity();
        Check(ragdoll.ApplyDriveSettings(changed, error) &&
            (childBody->getGlobalPose().p - before.p).magnitude() < 1e-7f &&
            std::abs(childBody->getGlobalPose().q.dot(before.q)) > .999999f &&
            (childBody->getAngularVelocity() - velocity).magnitude() < 1e-7f,
            "Live frequency and ratio edits preserve the physical state");
        const auto beforeBackendSwitch = ragdoll.GetBoneWorld(1);
        auto forceAsset = changed;
        for (auto& drive : forceAsset.constraints[0].drives)
        {
            drive.useAcceleration = false;
            drive.stiffness = 100;
            drive.damping = 10;
        }
        Check(ragdoll.ApplyDriveSettings(forceAsset, error) && !ragdoll.UsesArticulation() &&
            Difference(Quaternion(beforeBackendSwitch), Quaternion(ragdoll.GetBoneWorld(1))) < .001f,
            "Live switch to legacy D6 preserves pose");
        Check(ragdoll.ApplyDriveSettings(changed, error) && ragdoll.UsesArticulation() &&
            world.GetDynamicBodyCount() == 2 && world.GetConstraintCount() == 1,
            "Live switch back to articulation does not duplicate objects");
        ragdoll.Destroy();
        Check(world.GetConstraintCount() == 0 && world.GetDynamicBodyCount() == 0 && world.GetErrorCount() == 0,
            "Acceleration drive releases all bodies and constraints without errors");
        return errors;
    }

    void TestAccelerationDrives()
    {
        for (bool slerp : {false, true})
        {
            const auto reference = SampleAccelerationResponse(1.0f, .1f, slerp, 10000);
            const auto heavy = SampleAccelerationResponse(8.0f, .1f, slerp, 10000);
            const auto larger = SampleAccelerationResponse(1.0f, .4f, slerp, 10000);
            float difference = 0;
            for (size_t i = 0; i < reference.size(); ++i)
            {
                difference = (std::max)(difference, std::abs(reference[i] - heavy[i]));
                difference = (std::max)(difference, std::abs(reference[i] - larger[i]));
            }
            std::printf("acceleration %s response difference=%.6f final error=%.6f\n",
                slerp ? "SLERP" : "twist", difference, reference.back());
            Check(difference < .003f && reference.back() < .005f,
                "Same frequency and ratio produce matching response for different masses and shape inertias");
            const auto limited = SampleAccelerationResponse(1.0f, .4f, slerp, .001f);
            Check(limited.back() > .4f, "Torque capacity still limits an acceleration drive");
        }
        const auto rotated = SampleAccelerationResponse(1.0f, .1f, true, 10000, true);
        std::printf("rotated articulation frame error=%.6f\n", rotated.back());
        Check(rotated.back() < .005f, "Articulation drive target respects rotated parent/child joint frames");
        std::puts("PASS: acceleration drives normalize inertia, preserve torque caps and support live response edits");
    }

    void TestArticulatedArm()
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize articulated arm world");
        auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -5, 0)), PxVec3(10, .1f, 10), "floor");
        PhysicsBody torso;
        torso.bone = "torso";
        torso.shape = PhysicsShapeType::Sphere;
        torso.radius = .12f;
        torso.mass = 25;
        PhysicsBody upper;
        upper.bone = "upper";
        upper.shape = PhysicsShapeType::Capsule;
        upper.radius = .045f;
        upper.halfLength = .21f;
        upper.mass = 2;
        XMStoreFloat4(&upper.rotation, Quaternion(Vector3(kZUnitVector), XM_PIDIV2));
        auto lower = upper;
        lower.bone = "lower";
        lower.halfLength = .16f;
        lower.mass = 1.3f;
        auto hand = torso;
        hand.bone = "hand";
        hand.radius = .055f;
        hand.mass = .5f;
        // PA 배열과 내부 링크 순서를 다르게 만들어 캐시의 DOF 매핑도 함께 검증한다.
        PhysicsAsset asset;
        asset.bodies = {hand, torso, lower, upper};
        const std::vector<std::string> names{"torso", "upper", "lower", "hand"};
        const std::vector<Matrix4> initial{Pose(Vector3(0, 3, 0)), Pose(Vector3(.3f, 2.75f, 0)),
            Pose(Vector3(.3f, 2.3f, 0)), Pose(Vector3(.3f, 2.05f, 0))};
        const auto addJoint = [&](const char* parent, const char* child, Vector3 anchor) {
            PhysicsConstraint joint;
            joint.parentBone = parent;
            joint.childBone = child;
            const auto parentNode = std::find(names.begin(), names.end(), parent) - names.begin();
            const auto childNode = std::find(names.begin(), names.end(), child) - names.begin();
            joint.SetWorldFrame(initial[parentNode] * asset.bodies[asset.FindBody(parent)].GetLocalTransform(),
                initial[childNode] * asset.bodies[asset.FindBody(child)].GetLocalTransform(), Pose(anchor));
            joint.twist = joint.swingY = joint.swingZ = PhysicsJointMotion::Free;
            asset.constraints.push_back(joint);
        };
        addJoint("lower", "hand", Vector3(.3f, 2.1f, 0));
        addJoint("torso", "upper", Vector3(.3f, 3, 0));
        addJoint("upper", "lower", Vector3(.3f, 2.5f, 0));
        ConfigureDefaultDrives(asset);
        Ragdoll ragdoll;
        std::string error;
        Check(ragdoll.Create(world, asset, names, initial, Vector3(kZero), error, false), "Create animated arm PA");
        int owner = 0;
        ragdoll.SetQueryOwner(&owner);
        Check(ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.UsesArticulation(),
            "Animation enters connected-body simulation");
        Check(ragdoll.SetRootSupported(true, error), "Support dynamic torso");
        ragdoll.SetDrivesEnabled(true);
        const auto target = [&](float yaw, float raise) {
            auto result = initial;
            const Matrix4 turn = initial[0] * Pose(Vector3(kZero), Quaternion(Vector3(kYUnitVector), yaw)) * Invert(initial[0]);
            const Matrix4 shoulder = Pose(Vector3(.3f, 3, 0));
            const Matrix4 lift = shoulder * Pose(Vector3(kZero), Quaternion(Vector3(kZUnitVector), raise)) * Invert(shoulder);
            result[0] = turn * initial[0];
            for (size_t i = 1; i < result.size(); ++i)
            {
                result[i] = turn * lift * initial[i];
            }
            return result;
        };
        // 팔 전체를 수평으로 들어 손까지의 지렛팔과 중력 하중을 실제로 만든다.
        const auto horizontal = target(0, XM_PIDIV2);
        Check(ragdoll.UpdateDriveTargets(horizontal, 0, error), "Set horizontal arm target");
        for (int step = 0; step < 240; ++step)
        {
            Check(ragdoll.UpdateDriveTargets(horizontal, 1.0f / 60, error), "Update horizontal arm target");
            world.Step();
        }
        const float holdError = ragdoll.GetMaxDriveError();
        std::printf("articulated arm omega=16 horizontal error=%f\n", holdError);
        Check(holdError < .04f && ragdoll.GetMaxAnchorError() < .0001f,
            "Whole arm holds against gravity at frequency 16 with connected anchors");
        for (int step = 0; step < 180; ++step)
        {
            Check(ragdoll.UpdateDriveTargets(initial, 1.0f / 60, error), "Return to hanging idle arm");
            world.Step();
        }
        float turnError = 0;
        for (int step = 0; step < 360; ++step)
        {
            const float time = step / 60.0f;
            // 정지, 좌우 회전, 정지 구간을 같은 제어기로 처리한다.
            const float phase = PxClamp((time - 1.0f) / 3.0f, 0.0f, 1.0f);
            const float yaw = XM_PI * std::sin(phase * XM_PI) * std::sin(phase * XM_PI);
            Check(ragdoll.UpdateDriveTargets(target(yaw, 0), 1.0f / 60, error), "Update idle turn target");
            world.Step();
            Check(ragdoll.IsFinite(), "Turning articulated arm stays finite");
            turnError = (std::max)(turnError, ragdoll.GetMaxDriveError());
        }
        std::printf("articulated arm omega=16 turn error=%f rest error=%f\n", turnError, ragdoll.GetMaxDriveError());
        Check(turnError < .25f && ragdoll.GetMaxDriveError() < .04f,
            "Idle arm follows torso yaw and settles without increasing natural frequency");
        PxArticulationReducedCoordinate* articulation = nullptr;
        Check(floor->getScene()->getArticulations(&articulation, 1) == 1, "One articulation owns the arm");
        PxArticulationLink* links[4]{};
        articulation->getLinks(links, 4);
        for (const auto* link : links)
        {
            Check(link->userData == &owner, "Actor owner survives animation-to-articulation transition");
        }
        auto* cache = articulation->createCache();
        ragdoll.SetDrivesEnabled(false);
        articulation->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eFORCE);
        for (unsigned i = 0; i < articulation->getDofs(); ++i)
        {
            Check(cache->jointForce[i] == 0, "Passive transition clears persistent compensation torques");
        }
        cache->release();
        Check(ragdoll.SetRootSupported(false, error), "Release the torso support");
        const auto before = ragdoll.GetBoneWorld(1);
        for (int step = 0; step < 30; ++step)
        {
            world.Step();
        }
        Check(float(ragdoll.GetBoneWorld(1).GetW().GetY()) < float(before.GetW().GetY()) - .5f,
            "Unsupported passive articulation falls under gravity");
        Check(ragdoll.SetSimulating(false, Vector3(kZero), error) && ragdoll.GetKinematicBodyCount() == 4,
            "Articulation returns to editable animation bodies");
        Check(ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.UsesArticulation(),
            "Articulation can be recreated after returning to animation");
        ragdoll.Destroy();
        Check(world.GetErrorCount() == 0 && world.GetDynamicBodyCount() == 0 && world.GetConstraintCount() == 0,
            "Articulated arm lifecycle leaves no actors, joints or SDK errors");
        std::puts("PASS: articulated arm gravity, idle turning, ownership and passive/animation lifecycle");
    }

    std::vector<float> SamplePelvisInertiaResponse(float massScale, float sizeScale, Vector3 axis)
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize pelvis inertia world");
        auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -10, 0)), PxVec3(1), "floor");
        PhysicsAsset asset;
        PhysicsBody pelvis;
        pelvis.bone = "pelvis";
        pelvis.shape = PhysicsShapeType::Box;
        pelvis.halfExtents = XMFLOAT3(.12f * sizeScale, .24f * sizeScale, .08f * sizeScale);
        pelvis.mass = 4.0f * massScale;
        PhysicsBody child = pelvis;
        child.bone = "child";
        child.mass = 2.0f * massScale;
        asset.bodies = {pelvis, child};
        PhysicsConstraint joint;
        joint.parentBone = pelvis.bone;
        joint.childBone = child.bone;
        joint.twist = joint.swingY = joint.swingZ = PhysicsJointMotion::Locked;
        asset.constraints.push_back(joint);
        ConfigureDefaultDrives(asset);
        // 중력/접촉/지레팔 효과를 분리하고 질량과 축별 형상 관성에 대한 응답만 비교한다.
        const auto initial = Pose(Vector3(0, 3, 0));
        const std::vector<std::string> names{pelvis.bone, child.bone};
        Ragdoll ragdoll;
        std::string error;
        Check(ragdoll.Create(world, asset, names, {initial, initial}, Vector3(kZero), error),
            "Create articulated pelvis inertia fixture");
        PxArticulationReducedCoordinate* articulation = nullptr;
        Check(floor->getScene()->getArticulations(&articulation, 1) == 1, "Find pelvis articulation");
        PxArticulationLink* links[2]{};
        Check(articulation->getLinks(links, 2) == 2, "Find pelvis inertia links");
        for (auto* link : links)
        {
            link->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
            link->setAngularDamping(0);
            link->setLinearDamping(0);
        }
        ragdoll.SetDrivesEnabled(true);
        Check(ragdoll.SetRootSupported(true, error), "Enable articulated pelvis support");
        const auto target = Pose(Vector3(0, 3, 0), Quaternion(axis, .4f));
        Check(ragdoll.UpdateDriveTargets({target, target}, 0, error), "Set pelvis target without target acceleration spike");
        std::vector<float> errors;
        for (int frame = 0; frame < 120; ++frame)
        {
            Check(ragdoll.UpdateDriveTargets({target, target}, 1.0f / 60.0f, error), "Update inertia-based pelvis control");
            world.Step();
            Check(ragdoll.IsFinite() && ragdoll.GetKinematicBodyCount() == 0, "Pelvis control stays finite and dynamic");
            errors.push_back(ragdoll.GetRootRotationError());
        }
        const auto pose = articulation->getRootGlobalPose();
        const auto angular = articulation->getRootAngularVelocity();
        Check(ragdoll.SetRootSupported(false, error), "Release inertia-based pelvis control");
        Check((pose.p - articulation->getRootGlobalPose().p).magnitude() < 1e-7f &&
            std::abs(pose.q.dot(articulation->getRootGlobalPose().q)) > .999999f &&
            (angular - articulation->getRootAngularVelocity()).magnitude() < 1e-7f,
            "Releasing pelvis support preserves pose and angular velocity");
        links[0]->addTorque(PxVec3(0, .5f, 0), PxForceMode::eIMPULSE);
        world.Step();
        const float releasedSpeed = articulation->getRootAngularVelocity().magnitude();
        for (int frame = 0; frame < 30; ++frame)
        {
            Check(ragdoll.UpdateDriveTargets({target, target}, 1.0f / 60.0f, error), "Update unsupported articulation");
            world.Step();
        }
        Check(releasedSpeed > .001f && articulation->getRootAngularVelocity().magnitude() > releasedSpeed * .9f,
            "Unsupported pelvis does not receive a hidden restoring torque");
        ragdoll.Destroy();
        Check(world.GetErrorCount() == 0 && world.GetDynamicBodyCount() == 0 && world.GetConstraintCount() == 0,
            "Pelvis inertia fixture releases all physics objects");
        return errors;
    }

    void TestPelvisInertiaResponse()
    {
        std::vector<float> reference;
        float maximumDifference = 0;
        for (const auto axis : {Vector3(kXUnitVector), Vector3(kYUnitVector), Vector3(kZUnitVector)})
        {
            for (const auto scales : {PxVec2(1, 1), PxVec2(8, 1), PxVec2(1, 3)})
            {
                const auto response = SamplePelvisInertiaResponse(scales.x, scales.y, axis);
                if (reference.empty())
                {
                    reference = response;
                }
                for (size_t frame = 0; frame < response.size(); ++frame)
                {
                    maximumDifference = (std::max)(maximumDifference, std::abs(reference[frame] - response[frame]));
                }
                Check(response.back() < .005f, "Inertia-based pelvis converges to the stationary target");
            }
        }
        std::printf("pelvis mass/shape/axis response difference=%.6f rad\n", maximumDifference);
        Check(maximumDifference < .003f, "Same pelvis frequency and damping ratio normalize mass and axis inertia");
        std::puts("PASS: articulated pelvis inertia response and support release");
    }

    void TestDynamicPelvis()
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize dynamic pelvis world");
        auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -1, 0)), PxVec3(20, .1f, 20), "floor");
        PhysicsAsset asset;
        PhysicsBody pelvis; pelvis.bone = "pelvis"; pelvis.mass = 6;
        pelvis.shape = PhysicsShapeType::Box; pelvis.halfExtents = XMFLOAT3(.14f, .1f, .12f);
        pelvis.offset = XMFLOAT3(.03f, -.08f, .02f);
        XMStoreFloat4(&pelvis.rotation, Quaternion(Vector3(kZUnitVector), .4f));
        asset.bodies.push_back(pelvis);
        const std::vector<std::string> names{"pelvis"};
        const auto initial = Pose(Vector3(-2, 3, 1), Quaternion(Vector3(kYUnitVector), .7f));
        Ragdoll ragdoll; std::string error;
        Check(ragdoll.Create(world, asset, names, {initial}, Vector3(kZero), error) &&
            ragdoll.SetRootSupported(true, error) && ragdoll.GetKinematicBodyCount() == 0 && world.GetConstraintCount() == 1,
            "Pelvis support adds one force/torque constraint and no kinematic bodies");
        auto* scene = floor->getScene();
        PxActor* actor = nullptr; scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
        auto* body = actor->is<PxRigidDynamic>();
        const auto step = [&](const Matrix4& target) {
            Check(ragdoll.UpdateDriveTargets({target}, 1.0f / 60, error), "Update pelvis target");
            world.Step();
            Check(ragdoll.IsFinite() && ragdoll.GetKinematicBodyCount() == 0, "Pelvis remains finite and dynamic");
        };
        // A distant target must produce bounded acceleration, never a teleport.
        const auto before = body->getGlobalPose();
        const auto distant = Pose(Vector3(10, 3, 1), Quaternion(Vector3(kYUnitVector), .7f));
        Check(ragdoll.UpdateDriveTargets({distant}, 0, error) &&
            (before.p - body->getGlobalPose().p).magnitude() < 1e-7f,
            "Setting a pelvis target does not overwrite the body pose");
        world.Step();
        Check(body->getLinearVelocity().x > .1f && body->getLinearVelocity().x <= 60.0f / 60 * 1.02f,
            "Pelvis linear acceleration obeys the finite force limit");
        for (int i = 0; i < 240; ++i) step(initial);
        Check(ragdoll.GetRootPositionError() < .025f && ragdoll.GetRootRotationError() < .02f,
            "Force/torque support holds a static world target against gravity");
        float positionError = 0, rotationError = 0;
        Matrix4 target = initial;
        for (int i = 0; i < 360; ++i)
        {
            const float t = i / 60.0f;
            target = Pose(Vector3(-2 + .25f * std::sin(t), 3 + .12f * std::sin(t * 1.3f), 1 + .15f * std::sin(t * .8f)),
                Quaternion(Vector3(kYUnitVector), .7f + .4f * std::sin(t)) * Quaternion(Vector3(kXUnitVector), .2f * std::sin(t * 1.2f)));
            step(target);
            if (i > 60)
            {
                const auto actual = ragdoll.GetBoneWorld(0);
                positionError = (std::max)(positionError, float(Length(Vector3(actual.GetW()) - Vector3(target.GetW()))));
                rotationError = (std::max)(rotationError, Difference(Quaternion(actual), Quaternion(target)));
            }
        }
        std::printf("dynamic pelvis moving error=%.6f m / %.6f rad\n", positionError, rotationError);
        Check(positionError < .04f && rotationError < .04f, "Pelvis tracks world translation and rotation including PA offset/rotation");
        for (int i = 0; i < 120; ++i) step(target);
        body->addForce(PxVec3(2, 1, 0), PxForceMode::eVELOCITY_CHANGE);
        body->addTorque(PxVec3(0, 3, 1), PxForceMode::eVELOCITY_CHANGE);
        float pushedPosition = 0, pushedRotation = 0;
        for (int i = 0; i < 180; ++i)
        {
            step(target);
            pushedPosition = (std::max)(pushedPosition, ragdoll.GetRootPositionError());
            pushedRotation = (std::max)(pushedRotation, ragdoll.GetRootRotationError());
        }
        Check(pushedPosition > .02f && pushedRotation > .005f &&
            ragdoll.GetRootPositionError() < .025f && ragdoll.GetRootRotationError() < .02f,
            "Impulses displace and rotate the supported pelvis, then the drives recover");
        // Block the target with a wall: finite motors must yield to contact.
        auto* wall = world.CreateStaticBox(PxTransform(PxVec3(-1, 3, 1)), PxVec3(.1f, 2, 2), "pelvis wall");
        const auto blocked = Pose(Vector3(0, 3, 1));
        for (int i = 0; i < 240; ++i) step(blocked);
        Check(body->getGlobalPose().p.x < -1.15f && ragdoll.GetRootPositionError() > 1,
            "Supported dynamic pelvis is blocked by a static wall instead of following through it");
        const auto releasePose = body->getGlobalPose();
        const auto linear = body->getLinearVelocity(), angular = body->getAngularVelocity();
        Check(ragdoll.SetRootSupported(false, error) && world.GetConstraintCount() == 0 &&
            (releasePose.p - body->getGlobalPose().p).magnitude() < 1e-7f &&
            (linear - body->getLinearVelocity()).magnitude() < 1e-7f &&
            (angular - body->getAngularVelocity()).magnitude() < 1e-7f,
            "Removing pelvis support preserves state and releases its world constraint");
        for (int i = 0; i < 30; ++i) world.Step();
        Check(body->getGlobalPose().p.y < releasePose.p.y - .2f, "Unsupported pelvis falls under gravity");
        Check(ragdoll.SetRootSupported(true, error) && ragdoll.SetRootSupported(true, error) && world.GetConstraintCount() == 1,
            "Repeated enable does not duplicate pelvis support");
        Ragdoll swapped; ragdoll.Swap(swapped);
        Check(!ragdoll.IsActive() && swapped.IsRootSupported(), "Swap transfers support ownership");
        swapped.Destroy();
        Check(world.GetConstraintCount() == 0 && world.GetDynamicBodyCount() == 0 && world.GetErrorCount() == 0,
            "Destroying supported ragdoll releases all PhysX objects without errors");
        wall->release();
        std::puts("PASS: dynamic pelvis force limits, animated targets, impacts, blocked contact and support lifecycle");
    }
}

void RunPelvisDriveTests()
{
    TestPelvisInertiaResponse();
    TestDynamicPelvis();
}

void RunActiveRagdollTests()
{
    PhysicsAsset asset;
    PhysicsBody root; root.bone = "root"; root.shape = PhysicsShapeType::Sphere; root.radius = .08f;
    PhysicsBody child = root; child.bone = "child"; child.mass = 1.5f;
    child.offset = XMFLOAT3(.02f, -.1f, .03f);
    XMStoreFloat4(&child.rotation, Quaternion(Vector3(kZUnitVector), .3f));
    asset.bodies = {root, child};
    const std::vector<std::string> names{"root", "child"};
    const std::vector<Matrix4> initial{Pose(Vector3(0, 3, 0)), Pose(Vector3(0, 2.5f, 0))};
    PhysicsConstraint hinge; hinge.parentBone = "root"; hinge.childBone = "child";
    hinge.SetWorldFrame(initial[0], initial[1] * child.GetLocalTransform(), initial[0]);
    hinge.swingY = hinge.swingZ = PhysicsJointMotion::Locked;
    hinge.twistMin = -.9f; hinge.twistMax = .9f;
    hinge.drives[0] = {true, 720, 18, 25};
    asset.constraints.push_back(hinge);
    const auto target = [&](float angle) {
        auto result = initial;
        result[1] = initial[0] * Pose(Vector3(kZero), Quaternion(Vector3(kXUnitVector), angle)) * Invert(initial[0]) * initial[1];
        return result;
    };
    const auto desired = target(.65f);
    const auto goal = EvaluateJointDriveTarget(hinge, desired[0], desired[1] * child.GetLocalTransform());
    Check(Difference(goal, Quaternion(Vector3(kXUnitVector), .65f)) < .001f, "Drive conversion includes body offsets and both joint frames");
    const Matrix4 global = Pose(Vector3(-2, 1, 4), Quaternion(Vector3(kYUnitVector), .9f));
    Check(Difference(goal, EvaluateJointDriveTarget(hinge, global * desired[0], global * desired[1] * child.GetLocalTransform())) < .001f,
        "Joint drive target is invariant under actor world translation and rotation");
    auto excessive = target(1.5f);
    Check(Difference(EvaluateJointDriveTarget(hinge, excessive[0], excessive[1] * child.GetLocalTransform()), Quaternion(Vector3(kXUnitVector), .9f)) < .001f,
        "Hinge target respects asymmetric twist endpoints");
    PhysicsConstraint cone;
    cone.twist = PhysicsJointMotion::Locked; cone.swingYLimit = .45f; cone.swingZLimit = .7f;
    const auto diagonal = EvaluateJointDriveTarget(cone, Matrix4(kIdentity), Pose(Vector3(kZero), Quaternion(Normalize(Vector3(0, 1, 1)), 1.5f)));
    XMFLOAT4 q; XMStoreFloat4(&q, diagonal);
    const float y = 4 * std::atan2(q.y, 1 + q.w), z = 4 * std::atan2(q.z, 1 + q.w);
    Check(y * y / (.45f * .45f) + z * z / (.7f * .7f) <= 1.0001f && std::abs(q.x) < .0001f,
        "Diagonal target is projected into actual swing ellipse, with locked twist removed");

    PhysicsWorld world;
    Check(world.Initialize(false), "Initialize active ragdoll test world");
    auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -1, 0)), PxVec3(10, .1f, 10), "floor");
    Ragdoll ragdoll; std::string error;
    Check(ragdoll.Create(world, asset, names, initial, Vector3(kZero), error), "Create dynamic twist hinge");
    Check(ragdoll.SetRootSupported(true, error) && ragdoll.GetKinematicBodyCount() == 0, "Support dynamic parent independently of drive mode");
    Check(ragdoll.UpdateDriveTargets(desired, 0, error), "Prepare stationary drive target");
    ragdoll.SetDrivesEnabled(true);
    Check(ragdoll.GetDrivenJointCount() == 1, "Locked swings allow twist-only D6 drive");
    auto incompatible = asset;
    incompatible.constraints[0].swingY = incompatible.constraints[0].swingZ = PhysicsJointMotion::Limited;
    incompatible.constraints[0].useSlerpDrive = true;
    Check(!ragdoll.ApplyDriveSettings(incompatible, error) && ragdoll.GetDrivenJointCount() == 1,
        "Runtime drive edits reject SLERP on live locked axes without changing the hinge");
    for (int i = 0; i < 180; ++i) { ragdoll.UpdateDriveTargets(desired, 1.0f / 60, error); world.Step(); }
    const float heldError = ragdoll.GetMaxDriveError();
    std::printf("active hinge settled error=%.6f rad gap=%.6f limit=%.6f\n", heldError, ragdoll.GetMaxAnchorError(), ragdoll.GetMaxAngularError());
    Check(heldError < .08f && ragdoll.GetMaxAnchorError() < .005f, "PD hinge reaches static target against gravity");
    ragdoll.AddVelocity(Vector3(0, 0, 2));
    float peakError = 0;
    for (int i = 0; i < 180; ++i)
    {
        ragdoll.UpdateDriveTargets(desired, 1.0f / 60, error); world.Step();
        peakError = (std::max)(peakError, ragdoll.GetMaxDriveError());
    }
    Check(peakError > heldError + .02f && ragdoll.GetMaxDriveError() < .08f, "Physical impulse disturbs hinge and PD restores target");
    PxActor* actors[2]; floor->getScene()->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors, 2);
    auto* moving = actors[0]->is<PxRigidDynamic>();
    if (std::strcmp(moving->getName(), "child") != 0) moving = actors[1]->is<PxRigidDynamic>();
    const auto pose = moving->getGlobalPose(); const auto velocity = moving->getLinearVelocity(); const auto angular = moving->getAngularVelocity();
    ragdoll.SetDrivesEnabled(false); ragdoll.SetDrivesEnabled(true);
    Check((pose.p - moving->getGlobalPose().p).magnitude() < 1e-7f &&
        (velocity - moving->getLinearVelocity()).magnitude() < 1e-7f && (angular - moving->getAngularVelocity()).magnitude() < 1e-7f,
        "Drive mode switches preserve physical pose and both velocities");
    auto weak = asset; weak.constraints[0].drives[0].maxTorque = .01f;
    Check(ragdoll.ApplyDriveSettings(weak, error), "Apply lower torque limit without recreating objects");
    float weakPeak = 0;
    for (int i = 0; i < 180; ++i) { ragdoll.UpdateDriveTargets(desired, 1.0f / 60, error); world.Step(); weakPeak = (std::max)(weakPeak, ragdoll.GetMaxDriveError()); }
    Check(weakPeak > .25f, "Low torque cannot hold limb against gravity throughout the interval");
    Check(ragdoll.ApplyDriveSettings(asset, error), "Restore torque limit");
    for (int i = 0; i < 180; ++i) { ragdoll.UpdateDriveTargets(desired, 1.0f / 60, error); world.Step(); }
    Check(ragdoll.GetMaxDriveError() < .08f, "Restored torque wakes and recovers the limb");
    float movingError = 0;
    for (int i = 0; i < 240; ++i)
    {
        const auto movingTarget = target(.4f * std::sin(i / 60.0f));
        ragdoll.UpdateDriveTargets(movingTarget, 1.0f / 60, error); world.Step();
        if (i > 60) movingError = (std::max)(movingError, ragdoll.GetMaxDriveError());
    }
    std::printf("moving hinge peak error=%f\n", movingError);
    Check(movingError < .12f && ragdoll.GetMaxAngularError() < .02f, "Moving target tracks without unlocking hinge swings");
    auto traveling = target(.2f);
    const auto rootMotion = Pose(Vector3(.02f, .01f, -.01f), Quaternion(Vector3(kYUnitVector), .015f));
    for (auto& bone : traveling) bone = rootMotion * bone;
    Check(ragdoll.UpdateDriveTargets(traveling, 1.0f / 60, error), "Move supported root before release");
    world.Step();
    auto* supported = actors[0]->is<PxRigidDynamic>();
    if (std::strcmp(supported->getName(), "root") != 0) supported = actors[1]->is<PxRigidDynamic>();
    const auto rootPose = supported->getGlobalPose();
    const auto rootLinear = supported->getLinearVelocity(), rootAngular = supported->getAngularVelocity();
    Check(rootLinear.magnitude() > .01f && rootAngular.magnitude() > .01f, "Supported dynamic root responds to moving position and rotation targets");
    ragdoll.SetDrivesEnabled(false);
    Check(ragdoll.GetDrivenJointCount() == 0 && ragdoll.IsRootSupported(), "Disabling angular drives alone preserves independent support");
    Check(ragdoll.SetRootSupported(false, error) && ragdoll.GetKinematicBodyCount() == 0, "Release support into full dynamics");
    Check((rootPose.p - supported->getGlobalPose().p).magnitude() < 1e-7f &&
        std::abs(rootPose.q.dot(supported->getGlobalPose().q)) > .99999f &&
        (rootLinear - supported->getLinearVelocity()).magnitude() < 1e-7f &&
        (rootAngular - supported->getAngularVelocity()).magnitude() < 1e-7f,
        "Releasing support preserves the pelvis pose and both animated velocities");
    world.Step(); ragdoll.Destroy();
    // Exercise the separate SLERP solver path with simultaneous XYZ rotation.
    auto ball = asset;
    auto& ballJoint = ball.constraints[0];
    ballJoint.useSlerpDrive = true;
    ballJoint.swingY = ballJoint.swingZ = PhysicsJointMotion::Limited;
    ballJoint.swingYLimit = ballJoint.swingZLimit = 1.2f;
    ballJoint.drives[0] = {true, 720, 18, 25};
    Check(ragdoll.Create(world, ball, names, initial, Vector3(kZero), error) && ragdoll.SetRootSupported(true, error),
        "Create supported three-axis SLERP joint");
    ragdoll.SetDrivesEnabled(true);
    float ballError = 0;
    // The cone and its enclosing solver pyramid must not produce a second,
    // metre-sized overlay that ignores the requested joint visualization scale.
    auto* scene = floor->getScene();
    scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
    scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, .04f);
    world.Step();
    const auto& visualization = scene->getRenderBuffer();
    Check(visualization.getNbLines() > 0, "D6 angular limit visualization is present");
    for (PxU32 i = 0; i < visualization.getNbLines(); ++i)
    {
        const auto& line = visualization.getLines()[i];
        Check((line.pos0 - PxVec3(0, 3, 0)).magnitude() < .06f &&
            (line.pos1 - PxVec3(0, 3, 0)).magnitude() < .06f,
            "Cone plus enclosing pyramid respects joint visualization scale");
    }
    scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 0.0f);
    for (int i = 0; i < 300; ++i)
    {
        const float t = i / 60.0f;
        const auto rotation = Quaternion(Vector3(kZUnitVector), .35f * std::sin(t * 1.3f)) *
            Quaternion(Vector3(kYUnitVector), .3f * std::cos(t)) * Quaternion(Vector3(kXUnitVector), .4f * std::sin(t));
        auto sample = initial;
        sample[1] = initial[0] * Pose(Vector3(kZero), rotation) * Invert(initial[0]) * initial[1];
        Check(ragdoll.UpdateDriveTargets(sample, 1.0f / 60, error), "Update three-axis moving target");
        world.Step();
        if (i > 60) ballError = (std::max)(ballError, ragdoll.GetMaxDriveError());
    }
    std::printf("moving SLERP peak error=%f\n", ballError);
    Check(ballError < .12f && ragdoll.GetMaxAnchorError() < .005f, "SLERP follows combined XYZ rotation with correct target velocity");
    ragdoll.Destroy();
    Check(world.GetConstraintCount() == 0 && world.GetDynamicBodyCount() == 0 && world.GetErrorCount() == 0, "No errors or leaks in active lifecycle");
    std::puts("PASS: active target frames, limit projection, PD hinge tracking, torque limits, impulse recovery and mode preservation");
    world.Shutdown();
    TestDynamicPelvis();
    TestPelvisInertiaResponse();
    TestAccelerationDrives();
    TestArticulatedArm();
}
