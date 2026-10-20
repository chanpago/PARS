#pragma once

#include <string>

namespace ViewerLabels
{
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
