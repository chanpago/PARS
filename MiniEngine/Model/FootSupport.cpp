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

// 발 지지 계산과 다리 IK에 사용할 바디·본 연결, 기준 길이, 발바닥 로컬 좌표계를 준비한다.
// 최초 준비 이후에는 저장한 정보를 재사용한다. 모델 교체와 PA 변경은 호출자가 알려 준다.
bool PARS::FootSupport::Prepare(PARS::SkeletalMeshComponent& mesh)
{
    const PARS::PhysicsAsset* asset = mesh.GetPhysicsAsset();
    const Model* model = mesh.GetSkeletalMesh().get();
    const PARS::Ragdoll& ragdoll = mesh.GetRagdoll();
    if (!asset || !model || !ragdoll.IsActive())
    {
        m_Snapshot.message = u8"발바닥을 표시할 물리 바디가 없습니다.";
        return false;
    }

    // 초기화에서 발PA, 다리길이를 계산한다. 만약 실패하면 아래계산을 수행한다.
    if (m_Initialized)
    {
        return true;
    }

    // 좌우 발 본에 연결된 PA 바디를 찾는다. 바디 인덱스와 스켈레톤의 본 인덱스는 서로 다르다.
    // idices배열에 왼발,오른발의 물리바디 인덱스를 저장하기 위해 선언한다
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
    // 본 계층과 모델에 저장된 기준 자세를 사용한다. 현재 애니메이션 프레임이나 물리 자세는 사용하지 않는다.
    // 컴포넌트 변환을 단위행렬로 두어 캐릭터의 월드 위치·방향을 제외한 기준 본 변환을 구한다.
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

    // 최초 준비, 모델·PA 교체 또는 발바닥 설정 초기화 요청에 따라 기본값을 만든다.
    // 다음 접촉 갱신에서도 이전 구성의 이력이 이어지지 않도록 한다.
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
        // PA 바디 인덱스를 실제 스켈레톤의 발목 본 인덱스로 변환한다.
        const int node = ragdoll.GetBoneNodes()[index];
        if (node < 0 || size_t(node) >= referenceWorld.size())
        {
            continue;
        }

        auto& sole = m_Feet[side];
        sole.bodyIndex = index;
        sole.source = asset->bodies[index];
        // 발목에서 부모 계층을 따라 무릎·고관절을 찾아 두 뼈 IK의 연결 정보를 저장한다.
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
            // 기준 자세의 두 점 사이 거리(m)다. 종아리 길이는 발 들기 높이의 상한에도 사용한다.
            leg.thighLength = float(Length(knee - hip));
            leg.calfLength = float(Length(ankle - knee));
            leg.available = std::isfinite(leg.thighLength) && std::isfinite(leg.calfLength) &&
                leg.thighLength > 1e-5f && leg.calfLength > 1e-5f;
        }
        // 본의 스케일·전단을 제거한 프레임에 PA의 로컬 오프셋·회전을 적용해 기준 바디 자세를 구한다.
        const Matrix4 body = PARS::PhysicsBoneFrame(referenceWorld[node]) * sole.source.GetLocalTransform();
        // 발목에서 발가락 본으로 향하는 방향을 발의 앞쪽으로 사용한다. 본이 없으면 바디 X축을 사용한다.
        Vector3 forward(body.GetX());
        for (size_t toe = 0; toe < skeleton.names.size(); ++toe)
        {
            if (Matches(skeleton.names[toe], ToeNames[side]))
            {
                forward = Vector3(referenceWorld[toe].GetW()) - Vector3(referenceWorld[node].GetW());
            }
        }
        // 기준 발바닥은 수평으로 잡는다. 앞 방향이 퇴화하면 +Z를 사용하고, +Y와 외적해 오른쪽을 구한다.
        forward.SetY(0);
        forward = float(LengthSquare(forward)) > 1e-8f ? Normalize(forward) : Vector3(kZUnitVector);
        const Vector3 right = Normalize(Cross(up, forward));

        if (leg.available)
        {
            const Vector3 hip(referenceWorld[leg.hipNode].GetW());
            const Vector3 toKnee = Vector3(referenceWorld[leg.kneeNode].GetW()) - hip;
            const Vector3 toAnkle = Vector3(referenceWorld[node].GetW()) - hip;
            const Vector3 axis = Normalize(float(LengthSquare(toAnkle)) > 1e-12f ? toAnkle : toKnee);
            // 고관절~발목 축에 평행한 성분을 빼면 무릎이 그 축에서 어느 쪽으로 굽었는지 남는다.
            Vector3 bend = toKnee - axis * Dot(axis, toKnee);
            if (float(LengthSquare(bend)) < 1e-6f * (leg.thighLength + leg.calfLength) * (leg.thighLength + leg.calfLength))
            {
                // 기준 다리가 펴져 있으면 발가락의 앞 방향을 무릎 굽힘 기준으로 저장한다.
                bend = forward - axis * Dot(axis, forward);
            }
            if (float(LengthSquare(bend)) > 1e-8f)
            {
                // 고관절에서 굽힘 방향으로 한 단위 떨어진 점을 고관절 로컬로 변환한다.
                // 고관절 위치가 원점으로 상쇄되어 로컬 방향만 남으며, IK의 대체 무릎 방향으로 사용한다.
                leg.referenceBendInHip = ToPx(Vector3(Invert(PhysicsBoneFrame(referenceWorld[leg.hipNode])) * (hip + Normalize(bend))));
            }
        }

        // 기준 자세에서 발바닥 방향과 높이를 정한다. 이후에는 실제 발의 기울기까지 따라간다.
        // 바디 형상의 수직 반범위만큼 내려 기준 발바닥 중심을 잡는다. 실제 접촉점은 별도로 수집한다.
        const Vector3 center = Vector3(body.GetW()) - up * Extent(sole.source, body, up);
        // 바디의 역변환을 곱해 발바닥 좌표계를 바디 로컬로 저장한다.
        // 이후 실제 바디 월드 변환 * soleInBody로 현재 발바닥 월드 변환을 얻는다.
        sole.soleInBody = Invert(body) * Matrix4(right, up, forward, center);
        // PA 형상의 좌우·앞뒤 반범위로 레이 투영용 사각형의 기본 크기를 정한다.
        m_Settings[side].halfWidth = Max(.005f, Extent(sole.source, body, right));
        m_Settings[side].halfLength = Max(.005f, Extent(sole.source, body, forward));
        found = true;
    }
    // 한쪽이라도 발바닥 정보를 준비했으면 성공이다. 각 다리의 본 연결·길이 유효성은 leg.available로 따로 구분한다.
    m_Initialized = found;
    m_Snapshot.message = found ? "" : u8"LeftFoot / RightFoot 물리 바디를 찾을 수 없습니다.";
    return found;
}

void PARS::FootSupport::OnPhysicsAssetChanged(const PhysicsAsset& asset)
{
    // 같은 PA 재적용이나 드라이브 변경만으로 사용자가 조절한 발바닥 설정을 지우지 않는다.
    // 바디 순서·형상이 달라졌거나 빠진 발이 추가됐을 수 있을 때만 다음 갱신에서 다시 찾는다.
    for (const Sole& sole : m_Feet)
    {
        const int index = sole.bodyIndex;
        if (index < 0 || size_t(index) >= asset.bodies.size() || !SameGeometry(asset.bodies[index], sole.source))
        {
            ResetSoles();
            return;
        }
    }
}

void PARS::FootSupport::Reset()
{
    m_Feet = {};
    m_Settings = {};
    m_Snapshot = {};
    m_Initialized = false;
    m_Contacts = {};
    m_ContactWorld = nullptr;
    ResetFootstep();
}

void PARS::FootSupport::ResetFootstep()
{
    m_FootstepPlanner.Reset();
    m_HasIdleReference = false;
    m_Snapshot.footstep = {};
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

    // 새 물리 스텝인지 확인
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

        // 각 발의 실제 물리 접촉을 읽음
        // 물리엔진에 저장된 접촉 결과에서 바닥으로 인정되는 접촉을 가져옴
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

void PARS::FootSupport::UpdateFootstep(SkeletalMeshComponent& mesh)
{
    const AnimInstance& animation = mesh.GetAnimInstance();

    // 애니메이션 전환 중이 아니고 현재 클립이름이 idle일때만 true
    const bool idle = !animation.IsBlending() && animation.GetAnimationName(animation.GetCurrentAnimationIndex()) == "Idle";

    if (m_IdleReferenceRevision != m_Snapshot.contactRevision || !idle)
    {
        m_HasIdleReference = false;
        m_IdleReferenceRevision = m_Snapshot.contactRevision;
    }

    // 필요할 때 idle 자세의 양발 본 변환을 저장
    if (idle && !m_HasIdleReference && m_Snapshot.feet[0].available && m_Snapshot.feet[1].available)
    {
        PhysicsSkeleton skeleton;
        std::vector<Matrix4> reference;
        std::string error;
        if (mesh.BuildPhysicsSkeleton(skeleton, error) &&
            EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(kIdentity), reference, error))
        {
            m_HasIdleReference = true;
            for (size_t side = 0; side < m_Feet.size(); ++side)
            {
                const int node = m_Snapshot.feet[side].boneNode;
                if (node < 0 || size_t(node) >= reference.size())
                {
                    m_HasIdleReference = false;
                    break;
                }
                // 충격으로 변한 실제 발 간격 대신 최초 유효한 Idle 목표 자세를 저장한다.
                m_IdleFootBones[side] = PhysicsBoneFrame(reference[node]);
            }
        }
    }

    // 플래너에 넘길 현재 상태를 FootstepInput에 모음
    // mesh에서 정보를 가져온다.
    FootstepInput input;
    input.active = mesh.IsActiveRagdoll();
    input.automaticSupport = mesh.UsesAutomaticRootSupport();
    input.idleAnimation = idle;
    input.rootReleased = mesh.GetRootSupportRelease() != RootSupportRelease::None;
    input.outsideBufferReleased = mesh.GetRootSupportRelease() == RootSupportRelease::OutsideBuffer;
    input.hasCom = m_Snapshot.hasCenterOfMass;
    input.hasVelocity = m_Snapshot.hasCenterOfMassVelocity;
    input.com = m_Snapshot.centerOfMass;
    input.velocity = m_Snapshot.centerOfMassVelocity;
    input.hull = m_Snapshot.hull;
    input.zone = m_Snapshot.balance.zone;
    input.projectionHeight = m_Snapshot.projectedCenterOfMass.y;
    input.blueFoot = m_Snapshot.candidates.step;
    input.completedStep = m_Snapshot.completedPhysicsStep;
    input.revision = m_Snapshot.contactRevision;

    // 캐릭터의 의도된 수평 방향을 사용한다. 충격에 의한 골반의 순간 회전은 기준 방향에 섞지 않는다.
    const Matrix4 component(mesh.GetComponentTransform());
    PxVec3 forward = ToPx(Vector3(component.GetZ()));
    forward.y = 0;
    input.hasReference = m_HasIdleReference && forward.isFinite() && forward.normalize() > 1e-6f;
    input.supportRight = PxVec3(forward.z, 0, -forward.x);
    std::array<PxVec3, 2> referenceFeet{PxVec3(0), PxVec3(0)};

    // idle자세의 발 간격을 현재 캐릭터 방향에 맞춰 준비함
    if (input.hasReference)
    {
        for (size_t side = 0; side < m_Feet.size(); ++side)
        {
            const Matrix4 sole = m_IdleFootBones[side] * m_Snapshot.feet[side].soleInBone;
            referenceFeet[side] = ToPx(Vector3(sole.GetW()));
            input.hasReference = input.hasReference && referenceFeet[side].isFinite();
        }
    }

    // 양발의 중점을 계산
    const PxVec3 midpoint = (referenceFeet[0] + referenceFeet[1]) * .5f;

    // 각 발에서 중점을 빼면 원래 그 발이 중앙에서 얼마나 옆이나 앞뒤로 떨어져 있었는지 알 수 있음
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        const FootSoleSample& foot = m_Snapshot.feet[side];
        FootstepFootInput& target = input.feet[side];
        target.available = foot.available;
        target.hasContactSample = foot.contact.hasSample;
        target.grounded = foot.contact.grounded;
        target.position = ToPx(Vector3(foot.soleWorld.GetW()));
        target.normal = foot.contact.normal;
        target.calfLength = foot.leg.available ? foot.leg.calfLength : 0;
        const PxVec3 offset = referenceFeet[side] - midpoint;
        target.stanceOffset = input.supportRight * offset.x + forward * offset.z;
    }

    // 균형을 회복하기위해 발을 어디에 딛을지 결정하고 발딛기의 진행 상태를 관리
    /*
    1. 현재 COM과 속도로 미래 COM을 예측
    2. 예측 COM이 A 밖이면 발딛기가 필요하다고 판단
    3. 전달받은 움직일 발 후보와 각 발의 착지 후보가 유효하면, 움직일 발과 착지 목표를 정함
    4. 이후 실제 발의 이동·접촉을 확인하면서 목표와 이륙·공중·착지 상태를 갱신
    */
    m_FootstepPlanner.Update(input, m_FootstepSettings);
    m_Snapshot.footstep = m_FootstepPlanner.GetSnapshot();
}

void PARS::FootSupport::Update(SkeletalMeshComponent& mesh, const PhysicsWorld& physics)
{
    const uint64_t sampleNumber = m_Snapshot.sampleNumber + 1;                          // 디버그용 번호
    const int previousCandidate = m_Snapshot.candidates.support;                        // 0: 왼발이 지지발 후보, 1: 오른발이 지지발 후보, -1: 유효한 지지발 후보가없음
    m_Snapshot = {};                                                                    // 이번 프레임 용으로 스냅샷을 비운다
    
    m_Snapshot.sampleNumber = sampleNumber;
    m_Snapshot.completedPhysicsStep = physics.GetStepCount();                           // 물리 스텝을 읽어온다

    // 여기서 COM을 계산한다. 계산은 Articulation의 함수를 사용
    Ragdoll& ragdoll = mesh.GetRagdoll();
    Vector3 center(kZero);
    m_Snapshot.hasCenterOfMass = ragdoll.GetCenterOfMass(center, m_Snapshot.totalMass); // 래그돌에서 질량 중심과 총질량을 가져온다

    // 적법하게 질량 중심계산에 성공했는지 검사한다
    if (m_Snapshot.hasCenterOfMass)
    {
        m_Snapshot.centerOfMass = ToPx(center);
    }

    Vector3 velocity(kZero);
    m_Snapshot.hasCenterOfMassVelocity = ragdoll.GetCenterOfMassVelocity(velocity);      // 현재 프레임의 리지드바디의 질량 * 속도를 전체질량으로 나눠서 가져온다. 
                                                                                         // 속도를 구하는데 이전 프레임의 중심차이 기반을 안쓰는 이유는 
                                                                                         // "예를 들어 한 스텝 도중 충돌해서 멈췄다면, 위치 차이로는 그동안 이동했으니 속도가 나오지만 현재 속도는 이미 0일 수 있는경우" 때문이다.
    if (m_Snapshot.hasCenterOfMassVelocity)
    {
        m_Snapshot.centerOfMassVelocity = ToPx(velocity);
    }

    // 왼발,오른발이 어느 물리바디인지
    // 발바닥의 위치, 방향, 크기
    // 다리 IK에 필요한 본 연결과 다리길이 파악
    if (!Prepare(mesh))
    {
        m_ContactWorld = nullptr;
        m_Contacts = {};
        m_FootstepPlanner.Reset();
        m_HasIdleReference = false;
        return;
    }

    // 좌우 발이 실제로 바닥에 닿아 있는지 확인하고, 발이 떨어지거나 착지한 순간을 기록하는 함수
    // 결과는  m_Contacts[0], m_Contacts[1]에 저장
    UpdateContacts(ragdoll, physics);
    m_Snapshot.contactRevision = m_ContactRevision;

    // 양발 아래 지지영역 A를 만들고, COM의 위치를 비교해서 골반 지지계수 w를 계산하는부분
    std::vector<PxVec3> supportPoints;
    std::array<std::vector<PxVec3>, 2> footPoints;
    supportPoints.reserve(8);
    double supportHeight = 0;
    const PxVec3 down(0, -RayLength, 0);
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        const Sole& sole = m_Feet[side];
        if (sole.bodyIndex < 0 || size_t(sole.bodyIndex) >= ragdoll.GetBodyCount())
        {
            continue;
        }
        const FootSoleSettings& settings = m_Settings[side];
        FootSoleSample& foot = m_Snapshot.feet[side];
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
    m_Snapshot.projectedPointCount = supportPoints.size();

    // 모인 지지점으로 영역 A를 만들고 COM을 투영
    m_Snapshot.hull = BuildFootSupportHull(std::move(supportPoints));
    if (m_Snapshot.hasCenterOfMass && m_Snapshot.projectedPointCount > 0)
    {
        // Flat-ground approximation: vertical projection to the mean valid hit height.
        m_Snapshot.projectedCenterOfMass = PxVec3(m_Snapshot.centerOfMass.x,
            float(supportHeight / m_Snapshot.projectedPointCount), m_Snapshot.centerOfMass.z);
        m_Snapshot.hasComProjection = true;
        m_Snapshot.comInsideHull = IsInsideFootSupportHull(m_Snapshot.hull, m_Snapshot.projectedCenterOfMass);
    }

    // A와 완충 영역 B를 기준으로 지지 계수를 계산
    // A안 1
    // A와 B사이 0~1
    // B바깥 0
    // 여기서 저장한 weight를 다음 updateautomaticrootsupport에서 읽어서 골반 지지 강도에 적용
    m_Snapshot.balance = EvaluateFootSupportBalance(m_Snapshot.hull, m_Snapshot.hasComProjection
        ? m_Snapshot.projectedCenterOfMass : PxVec3(std::numeric_limits<float>::quiet_NaN()), m_BufferScale);
    if (m_Snapshot.hasComProjection)
    {
        // 위에서 계산한 A/B 판정을 전달해 A~B에서도 움직일 발 후보를 만들 수 있게 한다.
        // 발딛기 필요 여부와 반대 발의 실제 지지 여부는 이후 플래너에서 판단한다.
        m_Snapshot.candidates = EvaluateFootSupportCandidates(m_Snapshot.hull,
            m_Snapshot.projectedCenterOfMass, footPoints, m_Snapshot.balance.zone, previousCandidate);
    }

    // COM 예측과 발 접촉 정보를 바탕으로 발딛기 필요 여부, 움직일 발과 착지 목표를 판단한다.
    // 실제 발의 이동·접촉에 따라 진행 상태와 발 이동 목표를 갱신하고 m_Snapshot.footstep에 저장한다.
    // 목표를 실제 관절에 적용하는 IK·물리 제어는 이후 단계에서 수행한다.
    UpdateFootstep(mesh);
}
