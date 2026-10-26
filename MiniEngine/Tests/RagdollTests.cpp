#include "../Model/Ragdoll.h"
#include "../Model/PhysicsPose.h"
#include "../Core/PhysicsWorld.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

using namespace Math;
using namespace PARS;
namespace
{
    void Check(bool value, const char* message)
    { if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); } }
    bool Near(const Matrix4& a, const Matrix4& b)
    {
        return float(Length(Vector3(a.GetW()) - Vector3(b.GetW()))) < .0001f &&
            float(Length(Vector3(a.GetX()) - Vector3(b.GetX()))) < .0001f &&
            float(Length(Vector3(a.GetY()) - Vector3(b.GetY()))) < .0001f;
    }
    GraphNode Node(Vector3 position, float scale = 1)
    {
        GraphNode result{};
        result.xform = Matrix4(Matrix3::MakeScale(scale), position);
        result.rotation = Quaternion(kIdentity); result.scale = XMFLOAT3(scale, scale, scale);
        return result;
    }

    void TestCenterOfMass()
    {
        using namespace physx;
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize COM scene");
        auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -1, 0)), PxVec3(10, .5f, 10), "COM floor");
        Check(floor != nullptr, "Create COM floor");
        auto& scene = *floor->getScene();
        const std::vector<std::string> names{"hip", "leg"};
        const std::vector<Matrix4> pose{
            Matrix4(OrthogonalTransform(Quaternion(Vector3(kZUnitVector), XM_PIDIV2), Vector3(1, 3, 0))),
            Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(3, 3, 0)))};
        PhysicsBody hip, leg;
        hip.bone = names[0]; hip.shape = PhysicsShapeType::Sphere; hip.radius = .1f; hip.mass = 3;
        leg = hip; leg.bone = names[1]; leg.mass = 1;
        PhysicsAsset asset; asset.bodies = {hip, leg};
        PhysicsConstraint joint; joint.parentBone = names[0]; joint.childBone = names[1];
        joint.SetWorldFrame(pose[0], pose[1], pose[1]);
        asset.constraints = {joint};
        std::string error;
        Vector3 center(kZero);
        float totalMass = -1;
        Ragdoll empty;
        Check(!empty.GetCenterOfMass(center, totalMass) && totalMass == 0, "Empty ragdoll has no COM");
        for (bool articulation : {false, true})
        {
            auto configured = asset;
            if (articulation) ConfigureDefaultDrives(configured);
            Ragdoll ragdoll;
            Check(ragdoll.Create(world, configured, names, pose, Vector3(kZero), error) &&
                ragdoll.UsesArticulation() == articulation, "Create COM fixture on D6/articulation");
            Check(ragdoll.GetCenterOfMass(center, totalMass) && std::abs(totalMass - 4) < 1e-5f &&
                float(Length(center - Vector3(1.5f, 3, 0))) < 1e-4f,
                "COM is weighted by each body's mass and excludes the floor");
            PxRigidBody* heavy = nullptr;
            if (articulation)
            {
                PxArticulationReducedCoordinate* rig = nullptr;
                Check(scene.getArticulations(&rig, 1) == 1, "Find COM articulation");
                PxArticulationLink* links[2];
                Check(rig->getLinks(links, 2) == 2, "Find COM links");
                for (auto* link : links) if (std::strcmp(link->getName(), "hip") == 0) heavy = link;
            }
            else
            {
                PxActor* actors[2];
                Check(scene.getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors, 2) == 2, "Find COM bodies");
                for (auto* actor : actors) if (std::strcmp(actor->getName(), "hip") == 0) heavy = actor->is<PxRigidBody>();
            }
            Check(heavy != nullptr, "Find the heavier COM body");
            heavy->setCMassLocalPose(PxTransform(PxVec3(1, 0, 0)));
            Check(ragdoll.GetCenterOfMass(center, totalMass) &&
                float(Length(center - Vector3(1.5f, 3.75f, 0))) < 1e-4f,
                "A rotated body's local COM offset affects the world COM");
            heavy->setMass(7);
            Check(ragdoll.GetCenterOfMass(center, totalMass) && std::abs(totalMass - 8) < 1e-5f &&
                float(Length(center - Vector3(1.25f, 3.875f, 0))) < 1e-4f,
                "COM reads current PhysX mass rather than cached PA mass");
        }
        Check(world.GetErrorCount() == 0, "COM queries and local mass frames have no PhysX errors");
        std::puts("PASS: whole-body COM, unequal masses and rotated local mass frames on D6/articulation");
    }

    void TestMovementBlocking()
    {
        using namespace physx;
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize PA movement queries");
        auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -.5f, 0)), PxVec3(10, .5f, 10), "Floor");
        auto* wall = world.CreateStaticBox(PxTransform(PxVec3(0, 1, 2)), PxVec3(10, 2, .1f), "Wall");
        auto* trigger = world.CreateStaticBox(PxTransform(PxVec3(0, .5f, .5f)), PxVec3(2, 1, .05f), "Trigger");
        Check(floor && wall && trigger, "Create PA movement obstacles");
        PxShape* triggerShape = nullptr;
        trigger->getShapes(&triggerShape, 1);
        triggerShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
        triggerShape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
        PhysicsBody root, limb;
        root.bone = "root"; root.shape = PhysicsShapeType::Box; root.halfExtents = XMFLOAT3(.1f, .15f, .1f);
        limb.bone = "limb"; limb.shape = PhysicsShapeType::Capsule; limb.radius = .04f; limb.halfLength = .08f;
        limb.offset = XMFLOAT3(.06f, 0, .02f);
        XMStoreFloat4(&limb.rotation, Quaternion(Vector3(kYUnitVector), XM_PIDIV2));
        const std::vector<std::string> names{"root", "limb"};
        const std::vector<Matrix4> pose{Matrix4(Matrix3(kIdentity), Vector3(0, .15f, 0)),
            Matrix4(Matrix3(kIdentity), Vector3(0, .55f, 0))};
        PhysicsConstraint joint; joint.parentBone = names[0]; joint.childBone = names[1];
        joint.SetWorldFrame(pose[0] * root.GetLocalTransform(), pose[1] * limb.GetLocalTransform(), pose[1]);
        for (bool articulation : {false, true})
        {
            PhysicsAsset asset; asset.bodies = {root, limb}; asset.constraints = {joint};
            if (articulation) ConfigureDefaultDrives(asset);
            Ragdoll ragdoll; std::string error;
            Check(ragdoll.Create(world, asset, names, pose, Vector3(kZero), error), "Create PA movement bodies");
            const auto forward = ragdoll.ConstrainHorizontalMovement(pose, Vector3(0, 0, 4));
            Check(float(forward.GetZ()) > 1.65f && float(forward.GetZ()) < 1.77f,
                "PA box and rotated capsule stop at the wall without using the wider movement capsule");
            const auto diagonal = ragdoll.ConstrainHorizontalMovement(pose, Vector3(3, 0, 4));
            Check(std::abs(float(diagonal.GetX()) - 3) < .001f && float(diagonal.GetZ()) < 1.77f,
                "PA movement slides tangentially along the wall");
            auto target = pose;
            for (auto& bone : target) bone.SetW(bone.GetW() + Vector4(0, 0, 1.78f, 0));
            Check(float(Length(ragdoll.ConstrainHorizontalMovement(target, Vector3(0, 0, .02f)))) < 1e-5f,
                "A blocked target cannot keep advancing while physical bodies lag behind it");
            Check(std::abs(float(ragdoll.ConstrainHorizontalMovement(target, Vector3(0, 0, -.2f)).GetZ()) + .2f) < 1e-5f,
                "Movement can back out of an overlapping target instead of sticking to the wall");
            Check(std::abs(float(ragdoll.ConstrainHorizontalMovement(pose, Vector3(1, 0, 0)).GetX()) - 1) < 1e-5f,
                "Ground contact and own PA bodies do not block horizontal movement");
        }
        Check(world.GetErrorCount() == 0, "PA movement queries produce no PhysX errors");
        std::puts("PASS: PA geometry wall blocking, sliding, target lead prevention and self/ground/trigger filtering");
    }

    void CheckBodyMaterials(physx::PxRigidStatic& floor, const PhysicsAsset& asset)
    {
        using namespace physx;
        auto& scene = *floor.getScene();
        std::vector<PxActor*> dynamics(scene.getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC));
        scene.getActors(PxActorTypeFlag::eRIGID_DYNAMIC, dynamics.data(), static_cast<PxU32>(dynamics.size()));
        std::vector<PxRigidActor*> bodies;
        for (auto* actor : dynamics) bodies.push_back(static_cast<PxRigidActor*>(actor));
        std::vector<PxArticulationReducedCoordinate*> articulations(scene.getNbArticulations());
        scene.getArticulations(articulations.data(), static_cast<PxU32>(articulations.size()));
        for (auto* articulation : articulations)
        {
            std::vector<PxArticulationLink*> links(articulation->getNbLinks());
            articulation->getLinks(links.data(), static_cast<PxU32>(links.size()));
            for (auto* link : links) bodies.push_back(link);
        }
        PxShape* floorShape = nullptr;
        PxMaterial* floorMaterial = nullptr;
        Check(floor.getShapes(&floorShape, 1) == 1 && floorShape->getMaterials(&floorMaterial, 1) == 1,
            "Read floor material");
        Check(floorMaterial->getStaticFriction() == .6f && floorMaterial->getDynamicFriction() == .5f &&
            floorMaterial->getRestitution() == .1f && floorMaterial->getFrictionCombineMode() == PxCombineMode::eAVERAGE,
            "Per-body overrides do not mutate the shared floor material");
        size_t checked = 0;
        for (auto* body : bodies)
        {
            const int index = body->getName() ? asset.FindBody(body->getName()) : -1;
            if (index < 0) continue;
            const auto& expected = asset.bodies[index].material;
            PxShape* shape = nullptr;
            PxMaterial* material = nullptr;
            Check(body->getShapes(&shape, 1) == 1 && shape->getMaterials(&material, 1) == 1, "Read PA body material");
            Check(material->getStaticFriction() == expected.staticFriction && material->getDynamicFriction() == expected.dynamicFriction &&
                material->getRestitution() == expected.restitution &&
                static_cast<int>(material->getFrictionCombineMode()) == static_cast<int>(expected.frictionCombine),
                "Each runtime shape retains its own PA friction, restitution and combine mode");
            ++checked;
        }
        Check(checked == asset.bodies.size(), "Inspect every body on both physics backends");
    }
}

void RunRagdollTests()
{
    TestCenterOfMass();
    TestMovementBlocking();
    // A skin root ignores the .01 armature above it. The mesh supplies that
    // scale exactly once, as in the imported Mixamo skeleton and DefaultVS.
    PhysicsSkeleton skeleton{{"armature", "hip", "helper", "leg", "toe", "mesh"},
        {-1, 0, 1, 2, 3, 0}, {-1, 5, -1, 5, 5, -1}};
    std::vector<GraphNode> pose{Node(Vector3(kZero), .01f), Node(Vector3(0, 200, 0)),
        Node(Vector3(0, -30, 0)), Node(Vector3(0, -30, 0)), Node(Vector3(0, -20, 0)), Node(Vector3(kZero))};
    pose[1].skeletonRoot = true;
    const Matrix4 component(OrthogonalTransform(Quaternion(Vector3(kYUnitVector), .7f), Vector3(3, 1, -2)));
    std::string error;
    std::vector<Matrix4> initial;
    Check(EvaluatePhysicsPose(skeleton, pose, component, initial, error), "Evaluate skin-space world pose");
    Check(float(Length(Vector3(initial[1].GetW()) - Vector3(component * Vector3(0, 2, 0)))) < 1e-5f,
        "Centimetres convert once and component location/rotation apply once");
    std::vector<int> nodes{1, 3};
    const Matrix4 motion(OrthogonalTransform(Quaternion(Vector3(kZUnitVector), .6f), Vector3(-2, 4, 1)));
    std::vector<Matrix4> targets{motion * initial[1], motion * initial[3]};
    std::vector<GraphNode> finalPose;
    Check(OverridePhysicsPose(skeleton, pose, component, nodes, targets, finalPose), "Convert physical world bones back to skin-local pose");
    std::vector<Matrix4> reconstructed;
    Check(EvaluatePhysicsPose(skeleton, finalPose, component, reconstructed, error) && Near(reconstructed[1], targets[0]) &&
        Near(reconstructed[3], targets[1]) && Near(reconstructed[4], motion * initial[4]),
        "Physics-to-render round trip preserves scale and helper/unsimulated child attachment");
    PhysicsAsset asset;
    PhysicsBody upper; upper.bone = "hip"; upper.radius = .12f; upper.halfLength = .18f;
    XMStoreFloat4(&upper.rotation, Quaternion(Vector3(kZUnitVector), XM_PIDIV2));
    PhysicsBody lower = upper; lower.bone = "leg"; lower.shape = PhysicsShapeType::Box; lower.halfExtents = XMFLOAT3(.3f, .1f, .1f);
    PhysicsBody toe; toe.bone = "toe"; toe.shape = PhysicsShapeType::Sphere; toe.radius = .08f;
    asset.bodies = {upper, lower, toe};
    for (int child = 1; child < 3; ++child)
    {
        PhysicsConstraint joint; joint.parentBone = asset.bodies[child - 1].bone; joint.childBone = asset.bodies[child].bone;
        const int a = child == 1 ? 1 : 3, b = child == 1 ? 3 : 4;
        Matrix4 pivot = PhysicsBoneFrame(initial[b]);
        joint.SetWorldFrame(PhysicsBoneFrame(initial[a]) * asset.bodies[child - 1].GetLocalTransform(),
            PhysicsBoneFrame(initial[b]) * asset.bodies[child].GetLocalTransform(), pivot);
        asset.constraints.push_back(joint);
    }
    asset.constraints[0].twist = PhysicsJointMotion::Locked;
    asset.constraints[0].swingY = PhysicsJointMotion::Free;
    PhysicsWorld physics;
    Check(physics.Initialize(false), "Initialize ragdoll world");
    auto* floor = physics.CreateStaticBox(physx::PxTransform(physx::PxVec3(0, -.25f, 0)), physx::PxVec3(100, .25f, 100), "RagdollFloor");
    Check(floor != nullptr, "Create ragdoll floor");
    const auto baseMaterialCount = floor->getScene()->getPhysics().getNbMaterials();
    Ragdoll ragdoll;
    Check(ragdoll.Create(physics, asset, skeleton.names, initial, Vector3(kZero), error) &&
        ragdoll.GetBodyCount() == 3 && ragdoll.GetJointCount() == 2, "Create all three PA shapes and D6 joints");
    Check(Near(ragdoll.GetBoneWorld(0), PhysicsBoneFrame(initial[1])) && ragdoll.GetMaxAnchorError() < .0001f,
        "Runtime bodies preserve animated bone offsets and joint anchors");
    Check(!ragdoll.Create(physics, asset, skeleton.names, initial, Vector3(kZero), error) && ragdoll.GetBodyCount() == 3,
        "Reject recreation without leaking or destroying live bodies");
    for (int i = 0; i < 360; ++i)
    {
        physics.Step();
        Check(ragdoll.IsFinite() && ragdoll.GetMaxAnchorError() < .08f, "Ragdoll remains finite and joints stay connected during fall");
    }
    Check(float(ragdoll.GetPosition().GetY()) < 1 && float(ragdoll.GetPosition().GetY()) > -.1f, "Ragdoll falls and rests on floor");
    Check(ragdoll.SetSimulating(false, Vector3(kZero), error) && !ragdoll.IsSimulating() &&
        ragdoll.GetKinematicBodyCount() == 3 && physics.GetConstraintCount() == 2,
        "Return the same bodies and joints to kinematic animation");
    Check(ragdoll.UpdateKinematicPose(initial, true, error), "Restore animation pose immediately");
    for (int i = 0; i < 120; ++i) physics.Step();
    Check(Near(ragdoll.GetBoneWorld(0), PhysicsBoneFrame(initial[1])), "Kinematic bones resist gravity");
    auto moved = initial;
    for (auto& bone : moved) bone = motion * bone;
    Check(ragdoll.UpdateKinematicPose(moved, false, error), "Set animation targets in rotated/transformed world space");
    physics.Step();
    Check(Near(ragdoll.GetBoneWorld(0), PhysicsBoneFrame(moved[1])) &&
        Near(ragdoll.GetBoneWorld(1), PhysicsBoneFrame(moved[3])) && Near(ragdoll.GetBoneWorld(2), PhysicsBoneFrame(moved[4])),
        "Fixed step moves every shape to its animated target, preserving PA offsets");
    Check(ragdoll.SetSimulating(true, Vector3(1, 0, 0), error) && ragdoll.IsSimulating() &&
        ragdoll.GetKinematicBodyCount() == 0 && physics.GetDynamicBodyCount() == 3 && physics.GetConstraintCount() == 2,
        "Switch existing kinematic objects to dynamic simulation without duplicates");
    const auto beforeFall = ragdoll.GetPosition();
    for (int i = 0; i < 30; ++i) physics.Step();
    Check(float(ragdoll.GetPosition().GetY()) < float(beforeFall.GetY()) - .2f, "Gravity resumes after kinematic-to-dynamic transition");
    ragdoll.Destroy();
    Check(!ragdoll.IsActive() && ragdoll.GetJointCount() == 0 && physics.GetDynamicBodyCount() == 0 &&
        physics.GetConstraintCount() == 0, "Release joints before bodies without leaving objects in the scene");
    for (int cycle = 0; cycle < 5; ++cycle)
    {
        Check(ragdoll.Create(physics, asset, skeleton.names, initial, Vector3(kZero), error, false) &&
            ragdoll.GetKinematicBodyCount() == 3, "Register PA directly in kinematic mode");
        Check(ragdoll.SetSimulating(true, Vector3(1, 0, 0), error), "Restart ragdoll");
        physics.Step(); ragdoll.Destroy();
    }

    // Reproduce the old editor bug: changing a collider without rebasing its
    // body-local joints makes articulation reconstruction move the skinned bones.
    auto editedBody = asset.bodies[1];
    editedBody.offset = XMFLOAT3(.17f, -.08f, .12f);
    XMStoreFloat4(&editedBody.rotation, Quaternion(Vector3(kYUnitVector), .7f) * Quaternion(Vector3(kZUnitVector), -.9f));
    editedBody.shape = PhysicsShapeType::Capsule;
    auto uncorrected = asset;
    ConfigureDefaultDrives(uncorrected);
    uncorrected.bodies[1] = editedBody;
    Check(ragdoll.Create(physics, uncorrected, skeleton.names, initial, Vector3(kZero), error, false),
        "Create uncorrected collider edit reproduction");
    ragdoll.SetCollisionEnabled(false);
    Check(ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.UsesArticulation(),
        "Start articulation with stale collider-local joint frames");
    Check(float(Length(Vector3(ragdoll.GetBoneWorld(1).GetW()) - Vector3(initial[3].GetW()))) > .01f,
        "Stale joint frames reproduce the bone displacement before the first physics step");
    ragdoll.Destroy();

    const std::vector<int> bodyNodes{1, 3, 4};
    for (bool articulation : {false, true})
    {
        auto edited = asset;
        if (articulation)
        {
            ConfigureDefaultDrives(edited);
        }
        Check(edited.UpdateBody(1, editedBody), "Rebase both joints of an edited middle collider");
        auto editedToe = edited.bodies[2];
        editedToe.shape = PhysicsShapeType::Box;
        editedToe.material.staticFriction = .1f;
        editedToe.material.dynamicFriction = .05f;
        editedToe.material.frictionCombine = PhysicsFrictionCombine::Minimum;
        editedToe.offset = XMFLOAT3(-.04f, .03f, .08f);
        XMStoreFloat4(&editedToe.rotation, Quaternion(Vector3(kXUnitVector), .5f));
        Check(edited.UpdateBody(2, editedToe), "Rebase an edited end collider after its parent was edited");
        Check(ragdoll.Create(physics, edited, skeleton.names, initial, Vector3(kZero), error, false),
            "Apply edited colliders to a kinematic character");
        CheckBodyMaterials(*floor, edited);
        ragdoll.SetCollisionEnabled(false);
        Check(ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.UsesArticulation() == articulation,
            "Start the edited PA using the requested D6/articulation backend");
        CheckBodyMaterials(*floor, edited);
        Check(ragdoll.GetMaxAnchorError() < .0001f, "Edited colliders retain coincident anchors at simulation start");
        std::vector<Matrix4> editedTargets;
        for (size_t i = 0; i < bodyNodes.size(); ++i)
        {
            const auto& bone = initial[bodyNodes[i]];
            Check(Near(ragdoll.GetBoneWorld(i), PhysicsBoneFrame(bone)),
                "Collider translation, rotation and primitive edits do not change initial physics bone poses");
            editedTargets.push_back(ragdoll.GetBoneWorld(i) * Invert(PhysicsBoneFrame(bone)) * bone);
        }
        Check(OverridePhysicsPose(skeleton, pose, component, bodyNodes, editedTargets, finalPose) &&
                EvaluatePhysicsPose(skeleton, finalPose, component, reconstructed, error),
            "Reconstruct skinning after starting simulation with edited colliders");
        for (size_t i = 0; i < initial.size(); ++i)
        {
            Check(Near(initial[i], reconstructed[i]),
                "Edited PA preserves skinning, imported scale and helper bones when simulation begins");
        }
        Check(ragdoll.SetSimulating(false, Vector3(kZero), error), "Return material overrides to kinematic mode");
        CheckBodyMaterials(*floor, edited);
        ragdoll.Destroy();
        Check(floor->getScene()->getPhysics().getNbMaterials() == baseMaterialCount,
            "Destroy and backend rebuild release custom materials without leaks");
    }
    std::puts("PASS: collider edits preserve joint anchors and skinning on D6/articulation simulation start");

    PhysicsMaterialSettings slippery;
    slippery.staticFriction = .1f;
    slippery.dynamicFriction = .05f;
    slippery.frictionCombine = PhysicsFrictionCombine::Minimum;
    auto* normal = physics.CreateDynamicBody(physx::PxTransform(physx::PxVec3(-4, .1f, -2)),
        physx::PxBoxGeometry(.15f, .1f, .15f), 1, "NormalMaterial");
    auto* ice = physics.CreateDynamicBody(physx::PxTransform(physx::PxVec3(-4, .1f, 2)),
        physx::PxBoxGeometry(.15f, .1f, .15f), 1, "SlipperyMaterial", slippery);
    Check(normal && ice, "Create independent contact-friction probes");
    const auto angularLocks = physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_X |
        physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y | physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z;
    normal->setRigidDynamicLockFlags(angularLocks);
    ice->setRigidDynamicLockFlags(angularLocks);
    for (int step = 0; step < 60; ++step) physics.Step();
    const float normalStart = normal->getGlobalPose().p.x, iceStart = ice->getGlobalPose().p.x;
    normal->setLinearVelocity(physx::PxVec3(2, 0, 0));
    ice->setLinearVelocity(physx::PxVec3(2, 0, 0));
    for (int step = 0; step < 60; ++step) physics.Step();
    const float normalDistance = normal->getGlobalPose().p.x - normalStart, iceDistance = ice->getGlobalPose().p.x - iceStart;
    Check(normalDistance < .8f && iceDistance > 1.2f && iceDistance > normalDistance + .7f,
        "Minimum-combined low friction slides farther on the unchanged default floor");
    std::printf("PASS: contact material sliding distances: default=%.3f m, slippery=%.3f m\n", normalDistance, iceDistance);
    normal->release();
    ice->release();
    Check(floor->getScene()->getPhysics().getNbMaterials() == baseMaterialCount, "Contact probes release custom materials");
    Check(physics.GetErrorCount() == 0, "No PhysX errors during create, simulate, destroy and restart");
    std::puts("PASS: ragdoll skin/world conversion, imported scale, all shapes, D6 fall, floor contact and repeated lifecycle");
}
