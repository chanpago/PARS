#include "../Model/Ragdoll.h"
#include "../Core/PhysicsWorld.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace PARS;
using namespace Math;
using namespace physx;

namespace
{
    void CheckArmDrive(bool valid, const char* message)
    {
        if (!valid)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    float MeasureArmDeflection(bool articulated, float strength)
    {
        PhysicsWorld world;
        CheckArmDrive(world.Initialize(false), "Initialize arm response scene");
        PxRigidStatic* marker = world.CreateStaticBox(PxTransform(PxVec3(0, -10, 0)), PxVec3(1), "marker");
        CheckArmDrive(marker != nullptr, "Create arm response scene marker");
        PxScene* scene = marker->getScene();
        scene->setGravity(PxVec3(0));
        const std::vector<std::string> names{"Hips", "Spine", "RightArm"};
        const std::vector<Matrix4> pose{
            Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3, 0))),
            Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3.5f, 0))),
            Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(.4f, 3.5f, 0)))};
        PhysicsAsset asset;
        for (size_t index = 0; index < names.size(); ++index)
        {
            PhysicsBody body;
            body.bone = names[index];
            body.shape = PhysicsShapeType::Box;
            body.halfExtents = DirectX::XMFLOAT3(.1f, .1f, .1f);
            body.mass = index == 0 ? 4.0f : 1.0f;
            asset.bodies.push_back(body);
            if (index == 0)
            {
                continue;
            }
            PhysicsConstraint joint;
            joint.parentBone = names[index - 1];
            joint.childBone = names[index];
            joint.SetWorldFrame(pose[index - 1], pose[index], pose[index]);
            for (PhysicsAngularDrive& drive : joint.drives)
            {
                drive.enabled = true;
                drive.useAcceleration = articulated;
                drive.stiffness = 20;
                drive.damping = 2;
                drive.maxTorque = 100;
                drive.naturalFrequency = 16;
                drive.dampingRatio = 1;
            }
            asset.constraints.push_back(joint);
        }
        Ragdoll ragdoll;
        std::string error;
        CheckArmDrive(ragdoll.Create(world, asset, names, pose, Vector3(kZero), error) &&
            ragdoll.UsesArticulation() == articulated, "Create the selected arm drive backend");
        ragdoll.SetDrivesEnabled(true);
        CheckArmDrive(ragdoll.SetRootSupported(true, error), "Support the root during the arm impulse");
        PxRigidBody* arm = nullptr;
        if (articulated)
        {
            PxArticulationReducedCoordinate* articulation = nullptr;
            CheckArmDrive(scene->getArticulations(&articulation, 1) == 1, "Find arm test articulation");
            PxArticulationLink* links[3]{};
            articulation->getLinks(links, 3);
            PxArticulationJointReducedCoordinate* spineJoint = nullptr;
            PxArticulationJointReducedCoordinate* armJoint = nullptr;
            for (PxArticulationLink* link : links)
            {
                const int body = ragdoll.FindPhysicsBody(link);
                if (body == 1)
                {
                    spineJoint = link->getInboundJoint();
                }
                else if (body == 2)
                {
                    armJoint = link->getInboundJoint();
                    arm = link;
                }
            }
            CheckArmDrive(spineJoint && armJoint, "Find independent spine and arm motors");
            const PxArticulationDrive originalArm = armJoint->getDriveParams(PxArticulationAxis::eTWIST);
            const PxArticulationDrive originalSpine = spineJoint->getDriveParams(PxArticulationAxis::eTWIST);
            CheckArmDrive(ragdoll.SetArmDriveStrength(.3f), "Set thirty percent arm strength");
            const PxArticulationDrive weakArm = armJoint->getDriveParams(PxArticulationAxis::eTWIST);
            CheckArmDrive(std::abs(weakArm.stiffness / originalArm.stiffness - .3f) < 1e-4f &&
                std::abs(weakArm.damping / originalArm.damping - std::sqrt(.3f)) < 1e-4f &&
                spineJoint->getDriveParams(PxArticulationAxis::eTWIST).stiffness == originalSpine.stiffness,
                "Arm tracking weakens while preserving damping ratio and spine strength");
        }
        else
        {
            PxActor* actors[4]{};
            const PxU32 count = scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors, 4);
            for (PxU32 index = 0; index < count; ++index)
            {
                PxRigidActor* actor = actors[index]->is<PxRigidActor>();
                if (ragdoll.FindPhysicsBody(actor) == 2)
                {
                    arm = actors[index]->is<PxRigidBody>();
                }
            }
        }
        CheckArmDrive(arm && ragdoll.SetArmDriveStrength(strength), "Select the arm response strength");
        CheckArmDrive(!ragdoll.SetArmDriveStrength(std::numeric_limits<float>::quiet_NaN()) &&
            ragdoll.GetArmDriveStrength() == strength, "Reject invalid tuning without losing the current setting");
        arm->addTorque(PxVec3(0, 0, .3f), PxForceMode::eFORCE, true);
        float peak = 0;
        for (unsigned frame = 0; frame < 30; ++frame)
        {
            CheckArmDrive(ragdoll.UpdateDriveTargets(pose, 1.0f / 60.0f, error), "Update actual arm drive targets");
            world.Update(1.0 / 60.0);
            const Quaternion rotation(PhysicsBoneFrame(ragdoll.GetBoneWorld(2)));
            const float cosine = std::abs(DirectX::XMVectorGetW(rotation));
            peak = (std::max)(peak, 2.0f * std::acos(PxClamp(cosine, 0.0f, 1.0f)));
        }
        CheckArmDrive(ragdoll.SetSimulating(false, Vector3(kZero), error) &&
            ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.GetArmDriveStrength() == strength,
            "Prepared kinematic and dynamic transitions preserve the arm setting");
        CheckArmDrive(ragdoll.IsFinite() && world.GetErrorCount() == 0, "Arm tuning produces finite physics without errors");
        return peak;
    }
}

void RunArmDriveTests()
{
    for (bool articulated : {false, true})
    {
        const float full = MeasureArmDeflection(articulated, 1);
        const float weak = MeasureArmDeflection(articulated, .3f);
        const float passive = MeasureArmDeflection(articulated, 0);
        std::printf("arm backend=%s peak deflection full=%.6f thirty_percent=%.6f passive=%.6f rad\n",
            articulated ? "articulation" : "D6", full, weak, passive);
        CheckArmDrive(weak > full * 1.15f && passive > weak * 1.15f,
            "The same impulse visibly deflects a weaker arm more, and zero strength releases the motor");
    }
    std::puts("PASS: arm tracking strength, independent torso gains, physical response and mode transitions");
}
