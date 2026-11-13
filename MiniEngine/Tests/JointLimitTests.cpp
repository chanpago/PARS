#include "../Model/Ragdoll.h"
#include "../Model/PhysicsPose.h"
#include "../Model/RagdollComparison.h"
#include "../Core/PhysicsWorld.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace Math;
using namespace PARS;
using namespace physx;

namespace
{
    void CheckLimit(bool valid, const char* message)
    {
        if (!valid)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    void TestAsymmetricTargets()
    {
        PhysicsConstraint joint;
        joint.twist = PhysicsJointMotion::Locked;
        joint.swingYLimit = .5f;
        joint.swingZMin = -.2f;
        joint.swingZMax = .55f;
        const float inputs[] = {-1.0f, -.2f, 0.0f, .55f, 1.0f};
        const float expected[] = {-.2f, -.2f, 0.0f, .55f, .55f};
        for (bool articulation : {false, true})
        {
            for (int index = 0; index < 5; ++index)
            {
                const Quaternion raw(Vector3(kZUnitVector), inputs[index]);
                const Quaternion target = articulation ? ClampArticulationDriveTarget(joint, raw) :
                    EvaluateJointDriveTarget(joint, Matrix4(kIdentity), Matrix4(Matrix3(raw), Vector3(kZero)));
                const ComparisonJointRotation result = AnalyzeComparisonJoint(joint, target, articulation);
                CheckLimit(std::abs(result.angles[2] - expected[index]) < 1e-5f && result.excess[2] < 1e-5f,
                    "Targets clamp to distinct Z endpoints in both physics backends");
            }
            const ComparisonJointRotation lower = AnalyzeComparisonJoint(joint, Quaternion(Vector3(kZUnitVector), -.3f), articulation);
            const ComparisonJointRotation upper = AnalyzeComparisonJoint(joint, Quaternion(Vector3(kZUnitVector), .65f), articulation);
            CheckLimit(std::abs(lower.excess[2] - .1f) < 1e-5f && std::abs(upper.excess[2] - .1f) < 1e-5f,
                "Diagnostics measure excess from each Z endpoint independently");
        }

        // 두 축이 각 경계 이내이면 비대칭 D6의 피라미드는 대각선 목표도 허용한다.
        const float y = std::tan(.4f * .25f);
        const float z = std::tan(.5f * .25f);
        const float norm = 1 + y * y + z * z;
        const Quaternion diagonal(XMVectorSet(0, 2 * y / norm, 2 * z / norm, (2 - norm) / norm));
        const Quaternion bounded = EvaluateJointDriveTarget(joint, Matrix4(kIdentity), Matrix4(Matrix3(diagonal), Vector3(kZero)));
        const ComparisonJointRotation result = AnalyzeComparisonJoint(joint, bounded);
        CheckLimit(ComparisonRotationError(diagonal, bounded) < 1e-5f && result.excess[3] == 0,
            "Asymmetric D6 targets and diagnostics do not apply the old centered ellipse");

        joint.swingZ = PhysicsJointMotion::Locked;
        CheckLimit(ComparisonRotationError(ClampArticulationDriveTarget(joint, Quaternion(Vector3(kZUnitVector), .4f)),
            Quaternion(kIdentity)) < 1e-5f, "Locked Z ignores stored minimum and maximum");
        joint.swingZ = PhysicsJointMotion::Free;
        CheckLimit(ComparisonRotationError(ClampArticulationDriveTarget(joint, Quaternion(Vector3(kZUnitVector), .8f)),
            Quaternion(Vector3(kZUnitVector), .8f)) < 1e-5f, "Free Z ignores stored minimum and maximum");
        joint.swingZ = PhysicsJointMotion::Limited;
        joint.swingZMin = .1f;
        joint.swingZMax = .3f;
        CheckLimit(ComparisonRotationError(ClampArticulationDriveTarget(joint, Quaternion(kIdentity)),
            Quaternion(Vector3(kZUnitVector), .1f)) < 1e-5f, "Z bounds need not straddle zero");
    }

    void TestAsymmetricPhysics(bool articulation, bool lockedY)
    {
        PhysicsWorld world;
        CheckLimit(world.Initialize(false), "Initialize asymmetric joint-limit world");
        PxRigidStatic* marker = world.CreateStaticBox(PxTransform(PxVec3(0, -10, 0)), PxVec3(.1f), "scene marker");
        CheckLimit(marker != nullptr, "Create scene marker");
        PxScene& scene = *marker->getScene();
        scene.setGravity(PxVec3(0));

        PhysicsBody parentBody;
        parentBody.bone = "parent";
        parentBody.shape = PhysicsShapeType::Sphere;
        parentBody.radius = .05f;
        parentBody.mass = 10;
        PhysicsBody childBody = parentBody;
        childBody.bone = "child";
        childBody.mass = 1;
        const std::vector<std::string> names{parentBody.bone, childBody.bone};
        const std::vector<Matrix4> pose{
            Matrix4(Matrix3(kIdentity), Vector3(0, 2, 0)),
            Matrix4(Matrix3(kIdentity), Vector3(0, 2.5f, 0))};
        PhysicsConstraint joint;
        joint.parentBone = parentBody.bone;
        joint.childBone = childBody.bone;
        joint.SetWorldFrame(pose[0], pose[1], Matrix4(Matrix3(kIdentity), Vector3(0, 2.25f, 0)));
        joint.twist = PhysicsJointMotion::Locked;
        joint.swingY = lockedY ? PhysicsJointMotion::Locked : PhysicsJointMotion::Limited;
        joint.swingYLimit = .4f;
        joint.swingZMin = -.2f;
        joint.swingZMax = .55f;
        joint.drives[0].useAcceleration = articulation;
        PhysicsAsset asset;
        asset.bodies = {parentBody, childBody};
        asset.constraints = {joint};
        std::string error;

        for (float direction : {-1.0f, 1.0f})
        {
            Ragdoll ragdoll;
            CheckLimit(ragdoll.Create(world, asset, names, pose, Vector3(kZero), error) &&
                ragdoll.UsesArticulation() == articulation, "Create the requested asymmetric joint backend");
            ragdoll.SetCollisionEnabled(false);
            ragdoll.SetDrivesEnabled(false);
            PxRigidBody* parent = nullptr;
            PxRigidBody* child = nullptr;
            if (articulation)
            {
                PxArticulationReducedCoordinate* rig = nullptr;
                CheckLimit(scene.getArticulations(&rig, 1) == 1, "Find asymmetric articulation");
                PxArticulationLink* links[2];
                CheckLimit(rig->getLinks(links, 2) == 2, "Find asymmetric articulation links");
                for (PxArticulationLink* link : links)
                {
                    if (std::strcmp(link->getName(), "parent") == 0)
                    {
                        parent = link;
                    }
                    else
                    {
                        child = link;
                        const PxArticulationLimit limits = link->getInboundJoint()->getLimitParams(PxArticulationAxis::eSWING2);
                        CheckLimit(limits.low == joint.swingZMin && limits.high == joint.swingZMax,
                            "PhysX articulation receives the exact asymmetric Z endpoints");
                    }
                }
            }
            else
            {
                PxActor* actors[2];
                CheckLimit(scene.getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors, 2) == 2, "Find asymmetric D6 bodies");
                for (PxActor* actor : actors)
                {
                    if (std::strcmp(actor->getName(), "parent") == 0)
                    {
                        parent = actor->is<PxRigidBody>();
                    }
                    else
                    {
                        child = actor->is<PxRigidBody>();
                    }
                }
            }
            CheckLimit(parent && child, "Find parent and child for applied torque");
            // 드라이브를 끈 상태에서 외력만 가해 양쪽 물리 한계를 각각 확인한다.
            for (int step = 0; step < 180; ++step)
            {
                const PxVec3 torque = parent->getGlobalPose().q.rotate(PxVec3(0, 0, direction * .1f));
                parent->addTorque(-torque, PxForceMode::eFORCE);
                child->addTorque(torque, PxForceMode::eFORCE);
                world.Step();
            }
            const PxQuat relative = parent->getGlobalPose().q.getConjugate() * child->getGlobalPose().q;
            const Quaternion rotation(XMVectorSet(relative.x, relative.y, relative.z, relative.w));
            const ComparisonJointRotation measured = AnalyzeComparisonJoint(joint, rotation, articulation);
            const float expected = direction < 0 ? joint.swingZMin : joint.swingZMax;
            std::printf("asymmetric %s Y=%s torque=%+.0f Z=%.6f expected=%.6f\n",
                articulation ? "articulation" : "D6", lockedY ? "locked" : "limited", direction, measured.angles[2], expected);
            CheckLimit(std::abs(measured.angles[2] - expected) < .025f && ragdoll.GetMaxAngularError() < .025f,
                "External torque stops at the correct lower or upper Z limit");
        }
        marker->release();
        CheckLimit(world.GetErrorCount() == 0, "Asymmetric limits do not report PhysX errors");
    }
}

void RunJointLimitTests()
{
    TestAsymmetricTargets();
    for (bool articulation : {false, true})
    {
        for (bool lockedY : {false, true})
        {
            TestAsymmetricPhysics(articulation, lockedY);
        }
    }
    std::puts("PASS: asymmetric Z targets, diagnostics and external-torque limits on D6/articulation");
}
