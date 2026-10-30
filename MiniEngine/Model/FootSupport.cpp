#include "FootSupport.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsWorld.h"
#include "PhysicsPose.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

using namespace Math;
using namespace physx;

namespace
{
    const char* const FootNames[] = {"LeftFoot", "RightFoot"};
    const char* const ToeNames[] = {"LeftToeBase", "RightToeBase"};
    const char* const HipNames[] = {"LeftUpLeg", "RightUpLeg"};
    const char* const KneeNames[] = {"LeftLeg", "RightLeg"};
    bool Matches(const std::string& name, const char* leaf)
    {
        const auto separator = name.find_last_of(':');
        return name.compare(separator == std::string::npos ? 0 : separator + 1,
            std::string::npos, leaf) == 0;
    }

    bool SameGeometry(const PARS::PhysicsBody& a, const PARS::PhysicsBody& b)
    {
        return a.bone == b.bone && a.shape == b.shape && a.radius == b.radius && a.halfLength == b.halfLength &&
            std::memcmp(&a.offset, &b.offset, sizeof(a.offset)) == 0 &&
            std::memcmp(&a.rotation, &b.rotation, sizeof(a.rotation)) == 0 &&
            std::memcmp(&a.halfExtents, &b.halfExtents, sizeof(a.halfExtents)) == 0;
    }

    int FindAncestor(const PARS::PhysicsSkeleton& skeleton, int node, const char* name)
    {
        for (size_t depth = 0; node >= 0 && size_t(node) < skeleton.parents.size() && depth < skeleton.parents.size(); ++depth)
        {
            node = skeleton.parents[node];
            if (node >= 0 && size_t(node) < skeleton.names.size() && Matches(skeleton.names[node], name))
            {
                return node;
            }
        }
        return -1;
    }

    float Extent(const PARS::PhysicsBody& body, const Matrix4& world, Vector3 direction)
    {
        const float x = std::abs(float(Dot(Vector3(world.GetX()), direction)));
        if (body.shape == PARS::PhysicsShapeType::Capsule)
        {
            return body.radius + body.halfLength * x;
        }
        if (body.shape == PARS::PhysicsShapeType::Sphere)
        {
            return body.radius;
        }
        return body.halfExtents.x * x +
            body.halfExtents.y * std::abs(float(Dot(Vector3(world.GetY()), direction))) +
            body.halfExtents.z * std::abs(float(Dot(Vector3(world.GetZ()), direction)));
    }

    PxVec3 ToPx(Vector3 point)
    {
        return PxVec3(float(point.GetX()), float(point.GetY()), float(point.GetZ()));
    }

}

bool PARS::FootSupport::Prepare(PARS::SkeletalMeshComponent& mesh)
{
    const auto* asset = mesh.GetPhysicsAsset();
    const auto* model = mesh.GetSkeletalMesh().get();
    const auto& ragdoll = mesh.GetRagdoll();
    if (!asset || !model || !ragdoll.IsActive())
    {
        m_Snapshot.message = u8"발바닥을 표시할 물리 바디가 없습니다.";
        return false;
    }

    std::array<int, 2> indices{-1, -1};
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        for (size_t body = 0; body < asset->bodies.size(); ++body)
        {
            if (Matches(asset->bodies[body].bone, FootNames[side]))
            {
                indices[side] = static_cast<int>(body);
            }
        }
    }
    bool unchanged = m_Initialized && m_Model == model;
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        const int index = indices[side];
        unchanged = unchanged && index == m_Feet[side].bodyIndex &&
            (index < 0 || SameGeometry(asset->bodies[index], m_Feet[side].source));
    }
    if (unchanged)
    {
        return true;
    }

    PARS::PhysicsSkeleton skeleton;
    if (!mesh.BuildPhysicsSkeleton(skeleton, m_Snapshot.message))
    {
        return false;
    }
    const std::vector<GraphNode> reference(model->m_SceneGraph.get(), model->m_SceneGraph.get() + model->m_NumNodes);
    std::vector<Matrix4> referenceWorld;
    if (!PARS::EvaluatePhysicsPose(skeleton, reference, Matrix4(kIdentity), referenceWorld, m_Snapshot.message))
    {
        return false;
    }

    m_Feet = {};
    m_Settings = {};
    m_ContactWorld = nullptr;
    const Vector3 up(kYUnitVector);
    bool found = false;
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        const int index = indices[side];
        if (index < 0 || size_t(index) >= ragdoll.GetBodyCount() || size_t(index) >= ragdoll.GetBoneNodes().size())
        {
            continue;
        }
        const int node = ragdoll.GetBoneNodes()[index];
        if (node < 0 || size_t(node) >= referenceWorld.size())
        {
            continue;
        }

        auto& sole = m_Feet[side];
        sole.bodyIndex = index;
        sole.source = asset->bodies[index];
        auto& leg = sole.leg;
        leg.ankleNode = node;
        leg.kneeNode = FindAncestor(skeleton, node, KneeNames[side]);
        leg.hipNode = FindAncestor(skeleton, leg.kneeNode, HipNames[side]);
        if (leg.hipNode >= 0 && leg.kneeNode >= 0 &&
            size_t(leg.hipNode) < referenceWorld.size() && size_t(leg.kneeNode) < referenceWorld.size())
        {
            const Vector3 hip(referenceWorld[leg.hipNode].GetW());
            const Vector3 knee(referenceWorld[leg.kneeNode].GetW());
            const Vector3 ankle(referenceWorld[node].GetW());
            leg.thighLength = float(Length(knee - hip));
            leg.calfLength = float(Length(ankle - knee));
            leg.available = std::isfinite(leg.thighLength) && std::isfinite(leg.calfLength) &&
                leg.thighLength > 1e-5f && leg.calfLength > 1e-5f;
        }
        const Matrix4 body = PARS::PhysicsBoneFrame(referenceWorld[node]) * sole.source.GetLocalTransform();
        Vector3 forward(body.GetX());
        for (size_t toe = 0; toe < skeleton.names.size(); ++toe)
        {
            if (Matches(skeleton.names[toe], ToeNames[side]))
            {
                forward = Vector3(referenceWorld[toe].GetW()) - Vector3(referenceWorld[node].GetW());
            }
        }
        forward.SetY(0);
        forward = float(LengthSquare(forward)) > 1e-8f ? Normalize(forward) : Vector3(kZUnitVector);
        const Vector3 right = Normalize(Cross(up, forward));

        // 기준 자세에서 발바닥 방향과 높이를 정한다. 이후에는 실제 발의 기울기까지 따라간다.
        const Vector3 center = Vector3(body.GetW()) - up * Extent(sole.source, body, up);
        sole.soleInBody = Invert(body) * Matrix4(right, up, forward, center);
        m_Settings[side].halfWidth = Max(.005f, Extent(sole.source, body, right));
        m_Settings[side].halfLength = Max(.005f, Extent(sole.source, body, forward));
        found = true;
    }
    m_Model = model;
    m_Initialized = found;
    m_Snapshot.message = found ? "" : u8"LeftFoot / RightFoot 물리 바디를 찾을 수 없습니다.";
    return found;
}

void PARS::FootSupport::Reset()
{
    m_Feet = {};
    m_Settings = {};
    m_Snapshot = {};
    m_Model = nullptr;
    m_Initialized = false;
    m_Contacts = {};
    m_ContactWorld = nullptr;
}

void PARS::FootSupport::UpdateContacts(const Ragdoll& ragdoll, const PhysicsWorld& physics)
{
    const uint64_t step = physics.GetStepCount();
    if (m_ContactWorld != &physics || m_ClockRevision != physics.GetClockRevision() ||
        m_RagdollRevision != ragdoll.GetContactRevision())
    {
        m_Contacts = {};
        m_ContactWorld = &physics;
        m_ClockRevision = physics.GetClockRevision();
        m_RagdollRevision = ragdoll.GetContactRevision();
        m_ContactStep = step;
        ++m_ContactRevision;
        // 생성/순간이동/모드 전환 직후의 이전 스텝 접촉은 사용하지 않는다.
        return;
    }
    if (step == m_ContactStep)
    {
        return;
    }
    m_ContactStep = step;
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        auto& current = m_Contacts[side];
        const auto previous = current;
        current.leftGround = current.landed = false;
        const int index = m_Feet[side].bodyIndex;
        current.hasSample = index >= 0 && size_t(index) < ragdoll.GetBodyCount();
        PhysicsGroundContact ground;
        current.grounded = current.hasSample && ragdoll.GetBodyGroundContact(size_t(index), ground);
        current.position = ground.position;
        current.normal = ground.normal;
        current.pointCount = ground.pointCount;
        if (previous.hasSample && current.hasSample)
        {
            current.leftGround = previous.grounded && !current.grounded;
            current.landed = !previous.grounded && current.grounded;
            if (current.leftGround)
            {
                ++current.takeoffCount;
                current.lastTakeoffStep = step;
            }
            if (current.landed)
            {
                ++current.landingCount;
                current.lastLandingStep = step;
            }
        }
    }
}

bool PARS::FootSupport::SetBufferScale(float scale)
{
    if (!std::isfinite(scale) || scale < 1.01f || scale > 10.0f)
    {
        return false;
    }
    m_BufferScale = scale;
    return true;
}

void PARS::FootSupport::Update(SkeletalMeshComponent& mesh, const PhysicsWorld& physics)
{
    const auto sampleNumber = m_Snapshot.sampleNumber + 1;
    const int previousCandidate = m_Snapshot.candidates.support;
    m_Snapshot = {};
    auto& sample = m_Snapshot;
    sample.sampleNumber = sampleNumber;
    sample.completedPhysicsStep = physics.GetStepCount();

    // 여기서 COM을 계산한다. 계산은 Articulation의 함수를 사용
    auto& ragdoll = mesh.GetRagdoll();
    Vector3 center(kZero);
    sample.hasCenterOfMass = ragdoll.GetCenterOfMass(center, sample.totalMass);
    if (sample.hasCenterOfMass)
    {
        sample.centerOfMass = ToPx(center);
    }
    Vector3 velocity(kZero);
    sample.hasCenterOfMassVelocity = ragdoll.GetCenterOfMassVelocity(velocity);
    if (sample.hasCenterOfMassVelocity)
    {
        sample.centerOfMassVelocity = ToPx(velocity);
    }
    if (!Prepare(mesh))
    {
        m_ContactWorld = nullptr;
        m_Contacts = {};
        return;
    }
    UpdateContacts(ragdoll, physics);
    sample.contactRevision = m_ContactRevision;

    // 양발 아래 지지영역 A를 만들고, COM의 위치를 비교해서 골반 지지계수 w를 계산하는부분
    std::vector<PxVec3> supportPoints;
    std::array<std::vector<PxVec3>, 2> footPoints;
    supportPoints.reserve(8);
    double supportHeight = 0;
    const PxVec3 down(0, -RayLength, 0);
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        const auto& sole = m_Feet[side];
        if (sole.bodyIndex < 0 || size_t(sole.bodyIndex) >= ragdoll.GetBodyCount())
        {
            continue;
        }
        const auto& settings = m_Settings[side];
        auto& foot = sample.feet[side];
        foot.available = true;
        foot.boneNode = ragdoll.GetBoneNodes()[sole.bodyIndex];
        foot.leg = sole.leg;
        foot.contact = m_Contacts[side];
        const Matrix4 bodyWorld = ragdoll.GetBoneWorld(sole.bodyIndex) * sole.source.GetLocalTransform();
        const Quaternion rotation = Quaternion(Vector3(kZUnitVector), XMConvertToRadians(settings.rotation.z)) *
            Quaternion(Vector3(kYUnitVector), XMConvertToRadians(settings.rotation.y)) *
            Quaternion(Vector3(kXUnitVector), XMConvertToRadians(settings.rotation.x));

        // 실제 발 위치에 맞춰 사각형을 배치
        const Matrix4 world = bodyWorld * sole.soleInBody * Matrix4(OrthogonalTransform(rotation, Vector3(settings.offset)));
        foot.soleInBody = sole.soleInBody * Matrix4(OrthogonalTransform(rotation, Vector3(settings.offset)));
        foot.soleInBone = sole.source.GetLocalTransform() * foot.soleInBody;
        foot.soleWorld = world;
        const float w = settings.halfWidth, l = settings.halfLength;
        const Vector3 local[] = {Vector3(-w, 0, -l), Vector3(w, 0, -l), Vector3(w, 0, l), Vector3(-w, 0, l)};
        for (size_t i = 0; i < foot.corners.size(); ++i)
        {
            foot.corners[i] = ToPx(Vector3(world * local[i]));
            PxRaycastHit hit;

            // 사각형 아래로 레이를 쏨 20cm
            foot.projected[i] = physics.Raycast(foot.corners[i], down, hit, &mesh.GetOwner());
            foot.distances[i] = foot.projected[i] ? hit.distance : RayLength;
            foot.projections[i] = foot.projected[i] ? hit.position : foot.corners[i] + down;
            if (foot.projected[i])
            {
                supportPoints.push_back(hit.position);
                footPoints[side].push_back(hit.position);
                supportHeight += hit.position.y;
            }
        }
    }
    sample.projectedPointCount = supportPoints.size();

    // 모인 지지점으로 영역 A를 만들고 COM을 투영
    sample.hull = BuildFootSupportHull(std::move(supportPoints));
    if (sample.hasCenterOfMass && sample.projectedPointCount > 0)
    {
        // Flat-ground approximation: vertical projection to the mean valid hit height.
        sample.projectedCenterOfMass = PxVec3(sample.centerOfMass.x,
            float(supportHeight / sample.projectedPointCount), sample.centerOfMass.z);
        sample.hasComProjection = true;
        sample.comInsideHull = IsInsideFootSupportHull(sample.hull, sample.projectedCenterOfMass);
    }

    // A와 완충 영역 B를 기준으로 지지 계수를 계산
    // A안 1
    // A와 B사이 0~1
    // B바깥 0
    // 여기서 저장한 weight를 다음 updateautomaticrootsupport에서 읽어서 골반 지지 강도에 적용
    sample.balance = EvaluateFootSupportBalance(sample.hull, sample.hasComProjection
        ? sample.projectedCenterOfMass : PxVec3(std::numeric_limits<float>::quiet_NaN()), m_BufferScale);
    if (sample.hasComProjection)
    {
        sample.candidates = EvaluateFootSupportCandidates(sample.hull,
            sample.projectedCenterOfMass, footPoints, previousCandidate);
    }
}
