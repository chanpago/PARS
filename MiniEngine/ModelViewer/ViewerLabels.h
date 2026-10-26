#pragma once

#include <string>

namespace ViewerLabels
{
    inline const char* ConstraintLimits(bool articulation)
    {
        return articulation ?
            u8"빨강: X 회전 범위. 초록: Y 회전 범위. 파랑: Z 회전 범위. 각 부채꼴은 해당 축의 단독 회전 범위이며, 노란 바늘은 현재 상대 회전 벡터의 각 성분을 표시합니다. 주황: 떨어진 연결점." :
            u8"빨간 원뿔/호: Y/Z 스윙 허용 범위. 초록 부채꼴: X 트위스트 허용 범위. 노랑: 현재 자식 방향 / 스윙을 제거한 트위스트. 두 제한은 관절 중심을 공유합니다. 주황: 떨어진 연결점.";
    }

    // 화면 표기만 번역한다. 애니메이션 검색과 저장에 쓰는 에셋 이름은 그대로 둔다.
    inline const char* Animation(const std::string& name)
    {
        if (name == "Idle")
        {
            return u8"대기";
        }
        if (name == "Walking")
        {
            return u8"걷기";
        }
        if (name == "Slow Run")
        {
            return u8"달리기";
        }
        return name.c_str();
    }
}
