# 설명과 수식 표기 규칙

- 대화, 기술 설명, 문서와 PPT용 설명의 수식은 사용자가 별도 변환 없이 바로 읽을 수 있게 작성한다.
- **렌더링되지 않은 LaTeX 원문을 수식 대신 보여주지 않는다.** `\[ ... \]`, `\( ... \)`, `\boldsymbol`, `\mathbf`, `\boxed`, `\frac` 등의 구분자나 명령어가 그대로 노출되는 표기를 사용하지 않는다. 사용자가 LaTeX 원문 자체를 요청한 경우만 예외로 한다.
- 대화에서는 **일반 텍스트와 유니코드 기호로 읽을 수 있는 수식**을 기본으로 사용한다. 짧은 식은 인라인 코드로, 별도 표시할 식은 `text` 코드 블록으로 작성해 Markdown과 수식 렌더러의 지원 여부에 영향을 받지 않게 한다.
- 한 수식을 의미 없이 여러 줄로 쪼개지 않는다. 복잡한 식은 중간 변수를 정의해 단계별로 나누고, 각 단계의 식은 한 줄에서 읽을 수 있게 작성한다.
- 벡터, 스칼라, 좌표 성분과 연산의 의미를 식 주변에서 설명한다. 내적은 `dot(a, b)`, 스칼라 곱셈은 `*`처럼 구분하고, 연산 순서가 모호하면 괄호를 사용한다. 아래첨자를 렌더링할 수 없으면 `p_raw`, `c_xz`처럼 읽을 수 있는 변수명을 쓴다.
- 실제 PPT나 시각 자료를 만들 때는 수식이 완성된 형태로 표시되도록 수식 개체 또는 렌더링된 벡터 그래픽 등을 사용한다. LaTeX 소스 문자열을 장표 본문에 그대로 넣지 않는다.

표기 예시:

```text
delta = p_raw - s_ground
delta_plane = delta - n * dot(n, delta)
p_raw_xz = c_xz + v_xz * T + s * o_xz
```

위 예시에서 `p_raw`는 보정 전 착지 후보, `s_ground`는 지면 위 지지발 기준점, `n`은 정규화된 지면 법선이다. `c`는 COM 위치, `v`는 COM 속도, `T`는 예측 시간, `s`는 발 배치 배율, `o`는 해당 발의 배치 오프셋이며, `_xz`는 수평 성분을 뜻한다.

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
- AI가 새로 작성하거나 수정하는 C++ 코드에서는 **`auto`, `decltype(auto)` 등 타입 추론에 의존하는 선언을 사용하지 않고, 반드시 실제 타입을 명시한다.** `const auto`, `auto*`, `auto&` 등의 형태도 사용하지 않는다.
- 실제 타입 명시 규칙은 변수 선언, 범위 기반 `for` 문의 요소, 함수와 람다의 매개변수 및 반환 타입에도 적용한다. 기존 코드에서 타입 추론을 사용하고 있더라도 새로 작성하거나 수정하는 코드에서는 이 규칙을 따른다.
- 변수는 **한 선언문에 하나씩 선언하고, 각 선언문을 별도 줄에 작성한다.** 같은 타입이어도 쉼표로 여러 변수를 묶어 선언하지 않으며, 각 변수마다 타입을 명시한다. 멤버 변수와 지역 변수 등 모든 변수 선언에 적용한다.
- 주석은 **한국어로 작성한다.**

```cpp
uint64_t m_ClockRevision = 0;
uint64_t m_RagdollRevision = 0;
uint64_t m_ContactStep = 0;
uint64_t m_ContactRevision = 0;

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

for (Item& item : items)
{
    item.Update();
}

bool IsEnabled() const
{
    return m_Enabled;
}
```

## 클래스 및 구조체 선언 배치

클래스와 구조체를 새로 작성하거나 수정할 때는 아래 배치 규약을 적용한다. 헤더뿐 아니라 `.cpp` 내부의 클래스·구조체, 중첩 타입, 함수 내부 타입, 테스트용 타입에도 같은 규약을 적용한다.

### 종류별 배치 순서와 접근 지정자

선언은 다음 종류별 순서로 묶는다.

1. **타입 선언류**: 중첩 클래스·구조체·열거형, 타입 별칭(`using`, `typedef`).
2. **친구 선언류**: `friend class`, `friend struct` 등의 친구 관계 선언.
3. **생성자·소멸자류**: 일반 생성자, 복사 생성자, 이동 생성자, 소멸자, 생성자 상속 선언(`using Base::Base`).
4. **일반 함수류**: 객체의 동작을 수행하는 함수와 생명주기 함수.
5. **연산자류**: 복사·이동 대입을 포함한 모든 연산자 함수, 비교·산술·인덱싱·호출·형 변환 연산자 함수.
6. **Getter/Setter류**: 객체의 상태나 설정을 조회·설정하는 함수.
7. **정적 멤버 변수류**: 정적 상수와 정적 변수.
8. **인스턴스 멤버 변수류**: 객체마다 저장되는 멤버 변수.

- **종류별 묶음을 먼저 나누고, 각 묶음 안에서 `public:` → `protected:` → `private:` 순서를 반복한다.** 클래스 전체를 접근 지정자별로 한 번씩만 나누는 방식으로 배치하지 않는다.
- 정적 멤버 변수류와 인스턴스 멤버 변수류는 각각 별도 묶음이며, 각 묶음마다 접근 지정자 순서를 적용한다.
- 해당 종류의 선언이 없으면 그 묶음을 생략한다. 멤버가 없는 접근 구간과 불필요한 빈 접근 지정자도 생략한다.
- **구조체도 실제 멤버가 있는 구간에 접근 지정자를 명시한다.** 데이터만 있는 구조체도 변수 선언 앞에 `public:`을 작성한다.
- 기존 선언을 재배치할 때 각 멤버와 중첩 타입의 접근 수준을 보존한다. 배치를 맞추기 위해 공개 범위를 변경하지 않는다.
- 종류를 구분하는 주석을 작성할 때는 한국어를 사용한다.

### 선언의 분류 기준

- 타입 선언류는 해당 타입을 사용하는 생성자·함수·변수보다 앞에 둔다. 중첩 클래스·구조체의 내부 선언에도 동일한 배치 규약을 적용한다.
- 친구 함수와 친구 연산자 함수는 각각 일반 함수류 또는 연산자류에 두며, `friend` 지정은 유지한다.
- `operator=`는 매개변수 타입이나 복사·이동 여부에 관계없이 연산자류에 둔다. 생성자·소멸자류에 섞지 않는다.
- `= default`, `= delete`, `virtual`, `override`, `static`, `const`, `noexcept` 등 기존 선언의 의미를 보존한다. `static` 함수도 역할에 따라 일반 함수류, 연산자류 또는 Getter/Setter류에 분류한다.
- **Getter/Setter 여부는 함수 이름이 아니라 실제 역할로 판단한다.** `Get`, `Set`, `Is`, `Has`, `Can`, `Uses` 등의 접두사만으로 분류하지 않는다.
- 상태 조회나 값·설정 변경이 주된 역할인 함수는 Getter/Setter류에 둔다. 검증이나 계산이 포함되어 있어도 그 역할이 조회·설정이면 같은 기준을 적용한다.
- 리소스를 생성·교체하거나 시뮬레이션 구성을 전환하는 동작 중심 함수는 일반 함수류에 둔다. 예를 들어 `SetPhysicsAsset()`과 `SetSimulating()`은 일반 함수류로 분류한다.
- 생성자 초기화 리스트의 표기 방식은 기존 방식을 유지한다. 배치 규약을 맞추기 위해 원래 없던 생성자나 특수 멤버 함수를 추가하지 않는다.

### 멤버 변수 정렬과 예외

- 인스턴스 멤버 변수는 **각 접근 구간 안에서 실제 정렬 요구량(`alignof`)이 큰 타입부터 배치한다.** `sizeof`만으로 정렬 요구량을 추정하지 않는다.
- 정렬 요구량이 같으면 기존 기능별 묶음과 상대적 순서를 유지한다. 접근 지정자의 `public:` → `protected:` → `private:` 순서는 보존한다.
- 정적 상수와 정적 변수는 인스턴스의 패딩 정렬 대상에 포함하지 않는다. 배열 크기 등에 쓰는 상수는 사용되는 선언보다 앞에 둔다.
- **멤버의 생성·소멸 및 수명 의존관계는 패딩 정렬보다 우선한다.** 실제 멤버 생성 순서는 선언 순서이고 소멸 순서는 그 역순이므로, 참조 대상이 먼저 생성되고 이를 사용하는 멤버보다 나중에 소멸하도록 필요한 순서를 보존한다. 선언 순서를 바꿀 때 초기화 리스트와 기본 멤버 초기화의 의존관계도 함께 확인한다.
- **데이터 배치 자체가 계약인 타입은 변수 재정렬의 예외로 둔다.** GPU·셰이더와 공유하는 데이터, 바이너리 파일에 직접 읽고 쓰는 데이터, 외부 형식과 대응하는 타입은 필요한 멤버 순서와 오프셋을 보존한다.
- 위 고정 배치 타입과 `union`·비트 필드는 의미를 구성하는 순서·비트 배치를 보존하고, 필요한 명시적 패딩과 정렬 지정(`alignas`, `__declspec(align(...))`, `#pragma pack`)도 유지한다.
- 일반 구조체에 순서 기반 집합 초기화가 있으면 **멤버를 정렬하면서 모든 초기화 사용처도 함께 수정해 기존 값과 멤버의 대응을 보존한다.** 중첩 초기화, 배열 초기화, 반환값, `push_back({...})` 등의 사용처를 모두 확인한다. 순서 기반 초기화가 있다는 이유만으로 일반 구조체의 정렬을 생략하지 않는다.
- 조건부로 선언된 멤버를 옮길 때는 해당 전처리 조건을 보존한다. 패딩을 줄이기 위해 멤버의 타입이나 접근 수준을 바꾸지 않는다.
- 모든 변수 선언에는 앞서 정한 **한 선언문당 한 변수** 규칙을 함께 적용한다.

```cpp
#include <cstdint>

class Example
{
    // 타입 선언류
public:
    enum class State : std::uint8_t
    {
        Idle,
        Running
    };

    // 친구 선언류
private:
    friend class ExampleOwner;

    // 생성자·소멸자류
public:
    Example();
    Example(const Example& other);
    Example(Example&& other) noexcept;
    ~Example();

protected:
    explicit Example(std::uint64_t ownerId);

private:
    Example(std::uint64_t ownerId, bool enabled);

    // 일반 함수류
public:
    void Update(float deltaTime);

protected:
    void OnUpdate(float deltaTime);

private:
    bool ValidateState() const;

    // 연산자류
public:
    Example& operator=(const Example& other);
    Example& operator=(Example&& other) noexcept;
    bool operator==(const Example& other) const;

    // Getter/Setter류
public:
    std::uint32_t GetCount() const;
    void SetCount(std::uint32_t count);

protected:
    double GetElapsedTime() const;
    void SetElapsedTime(double elapsedTime);

private:
    bool IsEnabled() const;
    void SetEnabled(bool enabled);

    // 정적 멤버 변수류
public:
    static constexpr std::uint32_t MaximumCount = 64;

private:
    static std::uint32_t s_InstanceCount;

    // 인스턴스 멤버 변수류
public:
    std::uint64_t externalId = 0;

protected:
    std::uint64_t m_OwnerId = 0;

private:
    double m_ElapsedTime = 0.0;
    std::uint32_t m_Count = 0;
    std::uint16_t m_Flags = 0;
    State m_State = State::Idle;
    bool m_Enabled = false;
};
```

