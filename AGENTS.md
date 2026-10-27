# 코드 작성 규약

프로젝트에서 새로 작성하거나 수정하는 C++ 코드는 아래 규칙을 따른다.
기준은 `MiniEngine/Model/Components/SkeletalMeshComponent.cpp`와 관련 헤더의 기존 블록 스타일이다.
기존 코드에 섞여 있는 한 줄 조건문이나 축약된 함수는 따라야 할 예외로 취급하지 않는다.

- 들여쓰기는 **공백 4칸**을 사용한다. 들여쓰기에 탭을 사용하거나 탭과 공백을 혼용하지 않는다.
- **Allman 스타일**을 사용한다. 여는 중괄호 `{`는 다음 줄에 놓고, 닫는 중괄호 `}`와 함께 해당 구문과 같은 들여쓰기 깊이로 정렬한다.
- 블록 내부 코드는 한 단계마다 공백 4칸을 추가한다.
- `if`, `else if`, `else`, `for`, `while`, `do`의 본문에는 **한 문장만 있어도 반드시 중괄호를 사용한다.**
- `if (condition) return;`처럼 조건과 실행문을 한 줄에 쓰지 않는다. 조기 반환도 별도의 중괄호 블록으로 작성한다.
- `else if`와 `else`는 앞 블록의 닫는 중괄호 다음 줄에 놓고, 대응하는 `if`와 정렬한다.
- `if (`, `for (`, `while (`, `switch (`처럼 제어문 키워드와 여는 소괄호 사이에 공백을 둔다.
- 함수, 클래스, 구조체, 열거형, 네임스페이스의 중괄호에도 같은 줄바꿈 규칙을 적용한다.
- 짧은 함수와 getter도 한 줄로 축약하지 않고, 중괄호와 본문을 각각 별도 줄에 작성한다.
- 주석은 **한국어로 작성한다.**

```cpp
if (condition)
{
    DoSomething();
}
else if (otherCondition)
{
    return;
}
else
{
    DoOtherThing();
}

for (auto& item : items)
{
    item.Update();
}

bool IsEnabled() const
{
    return m_Enabled;
}
```
