#include "../Model/Ragdoll.h"
#include "../Core/PhysicsWorld.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
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

    float ArmElevationDegrees(const Ragdoll& ragdoll, const std::string& parent, const std::string& child)
    {
        PxTransform a(PxIdentity);
        PxTransform b(PxIdentity);
        CheckArmDrive(ragdoll.GetBodyPose(parent, a) && ragdoll.GetBodyPose(child, b), "Read the arm joint poses");
        const float vertical = a.q.rotateInv(b.q.rotate(PxVec3(1, 0, 0))).y;
        return std::asin(PxClamp(vertical, -1.0f, 1.0f)) * 180.0f / PxPi;
    }

    void TestWoundGripLimits(PhysicsWorld& world, Ragdoll& ragdoll)
    {
        PxRigidStatic* marker = world.CreateStaticBox(PxTransform(PxVec3(0, -10, 0)), PxVec3(1), "grip test");
        CheckArmDrive(marker != nullptr, "Read the scene constraint count for the hand grip");
        PxScene* scene = marker->getScene();
        const PxU32 baseline = scene->getNbConstraints();
        PxTransform palm(PxIdentity);
        CheckArmDrive(ragdoll.GetBodyPose("RightHand", palm), "Read the actual hand for the grip");
        const PxTransform far(palm.p + PxVec3(.15f, 0, 0), palm.q);
        ragdoll.DriveWoundGrip(4, 4, 5, palm, far, .5f, true);
        CheckArmDrive(scene->getNbConstraints() == baseline + 1, "A feasible approach can assist the hand");
        ragdoll.DriveWoundGrip(4, 4, 5, palm, far, .5f, false);
        CheckArmDrive(scene->getNbConstraints() == baseline, "An infeasible approach immediately removes the pulling drive");
        const PxTransform edgeOn(palm.p, palm.q * PxQuat(PxHalfPi, PxVec3(1, 0, 0)));
        ragdoll.DriveWoundGrip(4, 4, 5, edgeOn, palm, 1, true);
        ragdoll.DriveWoundGrip(4, 4, 5, palm, palm, 1, false);
        CheckArmDrive(scene->getNbConstraints() == baseline, "A close hand cannot lock with the wrong normal or an infeasible pose");
        ragdoll.DriveWoundGrip(4, 4, 5, palm, palm, 1, true);
        ragdoll.DriveWoundGrip(4, 4, 5, palm, palm, 1, false);
        ragdoll.DriveWoundGrip(4, 4, 5, palm, palm, 1, false);
        world.Update(PhysicsWorld::FixedDeltaTime);
        ragdoll.DriveWoundGrip(4, 4, 5, palm, palm, 1, false);
        CheckArmDrive(scene->getNbConstraints() == baseline + 1, "A held hand tolerates two infeasible ticks and duplicate samples");
        world.Update(PhysicsWorld::FixedDeltaTime);
        ragdoll.DriveWoundGrip(4, 4, 5, palm, palm, 1, true);
        for (unsigned tick = 1; tick <= 3; ++tick)
        {
            world.Update(PhysicsWorld::FixedDeltaTime);
            ragdoll.DriveWoundGrip(4, 4, 5, palm, palm, 1, false);
            CheckArmDrive(scene->getNbConstraints() == baseline + (tick < 3 ? 1 : 0),
                "A valid tick resets the grace period and the third consecutive failure releases the hand");
        }
        ragdoll.DriveWoundGrip(4, 4, 5, palm, palm, 1, true);
        CheckArmDrive(scene->getNbConstraints() == baseline + 1, "The hand can grip again when contact is feasible");
        ragdoll.ReleaseWoundGrip();
        marker->release();
    }

    void TestJointWounds(bool articulated)
    {
        PhysicsWorld world;
        CheckArmDrive(world.Initialize(false), "Create arm wound scene with gravity");
        const std::vector<std::string> names{"Hips", "RightShoulder", "RightArm", "RightForeArm", "RightHand", "LeftArm"};
        const int parents[] = {-1, 0, 1, 2, 3, 0};
        const float origins[] = {0, .10f, .22f, .48f, .72f, -.22f};
        const float centers[] = {0, .06f, .13f, .12f, .06f, -.13f};
        const float masses[] = {15, 1, 4, 1.18f, .45f, 4};
        const float torques[] = {0, 137, 115.4f, 52.1f, 32, 115.4f};
        PhysicsAsset asset;
        std::vector<Matrix4> pose;
        for (size_t i = 0; i < names.size(); ++i)
        {
            pose.emplace_back(OrthogonalTransform(Quaternion(kIdentity), Vector3(origins[i], 3, 0)));
            PhysicsBody body;
            body.bone = names[i];
            body.shape = PhysicsShapeType::Box;
            body.halfExtents = DirectX::XMFLOAT3(.035f, .035f, .035f);
            body.offset.x = centers[i];
            body.mass = masses[i];
            asset.bodies.push_back(body);
            const int parent = parents[i];
            if (parent < 0)
            {
                continue;
            }
            PhysicsConstraint joint;
            joint.parentBone = names[parent];
            joint.childBone = names[i];
            joint.SetWorldFrame(pose[parent] * asset.bodies[parent].GetLocalTransform(), pose[i] * body.GetLocalTransform(), pose[i]);
            joint.swingZMin = -DirectX::XM_PIDIV2;
            joint.swingZMax = DirectX::XM_PIDIV2;
            for (PhysicsAngularDrive& drive : joint.drives)
            {
                drive.enabled = true;
                drive.useAcceleration = articulated;
                drive.stiffness = 150;
                drive.damping = 12;
                drive.maxTorque = torques[i];
            }
            asset.constraints.push_back(joint);
        }
        Ragdoll ragdoll;
        std::string error;
        CheckArmDrive(ragdoll.Create(world, asset, names, pose, Vector3(kZero), error, false), "Create standing wound test rig");
        CheckArmDrive(ragdoll.FindWoundJoint(0) < 0 && ragdoll.FindWoundJoint(1) == 1 &&
            ragdoll.FindWoundJoint(2) == 1 && ragdoll.FindWoundJoint(3) == 2 && ragdoll.FindWoundJoint(4) == 3,
            "Map the shoulder and upper arm, forearm and hand to the lift, elbow and wrist joints");
        std::vector<RagdollBodyImpulse> contacts(1);
        RagdollBodyImpulse& hit = contacts.front();
        hit.body = 2;
        hit.fromProjectile = true;
        hit.newProjectileHit = true;
        hit.impulse = PxVec3(0, 0, 1);
        hit.impactContactId = 42;
        ragdoll.UpdateWounds(contacts);
        CheckArmDrive(ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.UsesArticulation() == articulated &&
            ragdoll.HasWound(), "The first kinematic hit survives the switch to the dynamic rig");
        ragdoll.SetDrivesEnabled(true);
        CheckArmDrive(ragdoll.SetRootSupported(true, error), "Support the torso with both arms held horizontally");
        const std::function<void(unsigned)> advance = [&](unsigned steps) -> void
        {
            for (unsigned i = 0; i < steps; ++i)
            {
                ragdoll.UpdateWounds(contacts);
                hit.newProjectileHit = false;
                CheckArmDrive(ragdoll.UpdateDriveTargets(pose, static_cast<float>(PhysicsWorld::FixedDeltaTime), error), "Drive raised arms");
                world.Update(PhysicsWorld::FixedDeltaTime);
            }
            ragdoll.UpdateWounds(contacts);
        };
        advance(21);
        CheckArmDrive(ragdoll.GetWoundStrength(1) == .10f && ragdoll.GetWoundStrength(2) == 1.0f &&
            ragdoll.GetWoundStrength(4) == 1.0f, "Only the struck shoulder is weak at full hand reach");
        advance(299);
        CheckArmDrive(ragdoll.GetWoundStrength(1) == .10f, "Five-second hold includes the last full fixed step");
        const float upperHitShoulder = ArmElevationDegrees(ragdoll, names[1], names[2]);
        const float upperHitElbow = ArmElevationDegrees(ragdoll, names[2], names[3]);
        advance(46);
        const float recovering = ragdoll.GetWoundStrength(1);
        CheckArmDrive(recovering > .45f && recovering < .65f, "Recover strength smoothly after the five-second hold");
        advance(105);
        CheckArmDrive(!ragdoll.HasWound(), "Continued projectile contact cannot renew the original wound");
        const float healthyShoulder = ArmElevationDegrees(ragdoll, names[1], names[2]);
        const float healthyElbow = ArmElevationDegrees(ragdoll, names[2], names[3]);
        hit.body = 3;
        hit.newProjectileHit = true;
        advance(300);
        const float forearmHitShoulder = ArmElevationDegrees(ragdoll, names[1], names[2]);
        const float forearmHitElbow = ArmElevationDegrees(ragdoll, names[2], names[3]);
        std::printf("arm wound %s degrees (shoulder, elbow) healthy=(%.1f, %.1f) upper hit=(%.1f, %.1f) forearm hit=(%.1f, %.1f)\n",
            articulated ? "articulation" : "D6", healthyShoulder, healthyElbow, upperHitShoulder, upperHitElbow, forearmHitShoulder, forearmHitElbow);
        CheckArmDrive(upperHitShoulder < healthyShoulder - 5.0f && forearmHitElbow < healthyElbow - 3.0f &&
            std::abs(forearmHitShoulder - healthyShoulder) < 5.0f, "Upper-arm and forearm hits droop at different joints under gravity");
        hit.newProjectileHit = true;
        advance(200);
        CheckArmDrive(ragdoll.GetWoundStrength(2) == .10f, "A fresh hit on the same forearm renews its hold");
        hit.body = 5;
        hit.newProjectileHit = true;
        advance(212);
        CheckArmDrive(ragdoll.GetWoundStrength(2) == 1.0f && ragdoll.GetWoundStrength(4) == .10f,
            "The other arm has an independent recovery timer");
        world.ResetClock();
        ragdoll.UpdateWounds({});
        CheckArmDrive(!ragdoll.HasWound() && ragdoll.IsFinite() && world.GetErrorCount() == 0, "Reset wounds with the simulation clock");
        TestWoundGripLimits(world, ragdoll);

        ragdoll.Destroy();
        const std::vector<std::string> torso{"Hips", "Spine", "Chest", "Neck", "Head", "RightUpLeg"};
        for (size_t i = 0; i < torso.size(); ++i)
        {
            asset.bodies[i].bone = torso[i];
            if (parents[i] >= 0)
            {
                asset.constraints[i - 1].parentBone = torso[parents[i]];
                asset.constraints[i - 1].childBone = torso[i];
            }
        }
        CheckArmDrive(ragdoll.Create(world, asset, torso, pose, Vector3(kZero), error), "Create a torso with a leg on the pelvis");
        CheckArmDrive(ragdoll.FindWoundJoint(0) == 0 && ragdoll.FindWoundJoint(1) == 0 &&
            ragdoll.FindWoundJoint(2) == 1 && ragdoll.FindWoundJoint(3) == 2 && ragdoll.FindWoundJoint(4) == 3 &&
            ragdoll.FindWoundJoint(5) < 0, "Map hips, spine, chest, neck and head wounds to their local joints");
        ragdoll.SetDrivesEnabled(true);
        const float deltaTime = static_cast<float>(PhysicsWorld::FixedDeltaTime);
        const unsigned lastImpactStep = static_cast<unsigned>(.32 / PhysicsWorld::FixedDeltaTime);
        const std::vector<RagdollBodyImpulse> noContacts;
        hit.impulse = PxVec3(0, 0, 80);
        for (const size_t body : std::array<size_t, 4>{5, 0, 1, 2})
        {
            hit.body = body;
            hit.newProjectileHit = true;
            // 이 테스트 골격에서 골반은 첫 척추 관절, 나머지 바디는 자신의 부모 관절에 반응한다.
            const size_t joint = body == 0 ? 0 : body - 1;
            for (unsigned step = 0; step <= lastImpactStep + 1; ++step)
            {
                ragdoll.UpdateImpactReaction(step == 0 ? contacts : noContacts, deltaTime);
                // 골반은 인접 척추로 감쇠되어 전달되고, 직접 맞은 관절은 0.32초 직전까지 약해진다.
                if (step == 0 || (body != 0 && step == lastImpactStep))
                {
                    CheckArmDrive(ragdoll.GetImpactReaction().GetJointDriveScale(joint) < 1.0f,
                        "The leg, pelvis and spine retain the transient projectile response");
                }
                advance(1);
            }
            CheckArmDrive(ragdoll.GetImpactReaction().GetJointDriveScale(joint) == 1.0f &&
                ragdoll.GetWoundStrength(joint) == 1.0f, "Leg, pelvis and spine regain full drive after the impact response");
        }
        CheckArmDrive(ragdoll.HasWound(), "The chest wound remains available for hand contact");
        hit.body = 3;
        hit.newProjectileHit = true;
        advance(21);
        CheckArmDrive(ragdoll.GetWoundStrength(2) == .10f && ragdoll.GetWoundStrength(1) == 1.0f,
            "A neck wound retains its local drive reduction while the chest stays at full strength");
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
        TestJointWounds(articulated);
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
