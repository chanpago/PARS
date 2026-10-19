#pragma once
#include "Animation.h"
#include "PhysicsAsset.h"

namespace PARS
{
    // 조인트별 메시 노드이며 DefaultVS의 meshWorld * skinJoint * IBM 구성과 일치한다.
    // -1은 스킨에 속하지 않는 노드를 뜻한다. 부모 정보는 깊이 우선 순서를 따른다.
    struct PhysicsSkeleton
    {
        std::vector<std::string> names;
        std::vector<int32_t> parents, meshNodes;
    };
    bool EvaluatePhysicsPose(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& pose,
        const Math::Matrix4& component, std::vector<Math::Matrix4>& world, std::string& error);
    // 물리 목표에는 원래 본의 늘어남도 포함된다. 시뮬레이션하지 않는 본은
    // 애니메이션 로컬 변환을 유지하고 시뮬레이션되는 부모를 따라간다.
    bool OverridePhysicsPose(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& source,
        const Math::Matrix4& component, const std::vector<int>& nodes,
        const std::vector<Math::Matrix4>& targets, std::vector<GraphNode>& result);
    // PA 오프셋을 적용한 뒤, 부모 조인트 좌표계에서 표현한 자식 조인트의 목표 회전이다.
    Math::Quaternion EvaluateJointDriveTarget(const PhysicsConstraint& joint,
        const Math::Matrix4& parentBodyTarget, const Math::Matrix4& childBodyTarget, bool clampToLimits = true);
    // 주요 본 전체를 포함한 프리셋이다(바디 20개 / 제약 19개). 손가락, 발가락,
    // 말단 표시 노드는 제외한다. 질량은 73kg 성인 남성 기준값(de Leva 1996)을 사용한다.
    // 몸통 그룹은 프리셋 콜라이더 부피에 비례해 나누고, 머리/목은
    // Roos 외(2020)의 남성 모델 비율에 따라 나눈다.
    // 관절 제한에는 성인 가동 범위 참고값, 해부학적 기준점 축, 방향별 범위를 표현하는
    // 편향 원뿔을 사용한다. 척추/목/어깨이음의 배분과 결합 운동은
    // 리그에 맞춘 근사이며, 출처 설명은 PhysicsPose.cpp를 참고한다.
    // 애니메이션 샘플이 아니라 가져온 팔을 벌린 기준 포즈가 필요하다.
    bool BuildMixamoPhysicsAsset(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& reference,
        PhysicsAsset& result, std::string& error);
}
