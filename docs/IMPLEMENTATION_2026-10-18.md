# 2026-10-18 액티브 래그돌 구현 및 수정 내역

작성일: 2026-10-18

> 이 문서는 2026-10-18 구현 기록이다. 2026-10-19에 골반 지지를 Kinematic 추종에서 Dynamic 힘·토크 드라이브로 변경했다. 현재 동작은 [액티브 래그돌 계획](../액티브%20래그돌%20계획.md)의 골반 힘·토크 지지 항목을 참고한다.

변경 비교 기준: `7e50399` (`fix: reduce ragdoll solver iterations to 16/4 for Debug performance`)

관련 계획: [액티브 래그돌 구현 계획](../액티브%20래그돌%20계획.md)

현재 구현은 **애니메이션을 목표로 하는 PhysX D6 관절 구동과 물리 포즈 렌더링**까지 연결되어 있다. Idle뿐 아니라 Walk / Run 클립도 수동으로 선택해 추종할 수 있다. 골반 지지를 켜면 골반만 애니메이션의 위치·회전을 직접 따르고, 나머지 바디는 동적 강체로 시뮬레이션한다.

**WASD로 실제 이동하면서 Active Ragdoll을 적용하는 연결은 아직 구현하지 않았다.** 해당 작업은 사용자 요청에 따라 2026-10-19로 미뤘다. 이동 연결을 위한 코드는 이번 변경에 추가하지 않았다.

## 1. 기존 기반과 이번 변경의 범위

비교 기준 커밋에 이미 있던 기능은 다음과 같다.

- DX12 렌더링, PhysX CPU 물리 월드, 고정 1/60초 물리 업데이트.
- 논리 프로세서 수에서 1을 뺀 PhysX 워커 스레드 설정과 명시적인 `World::Tick` 반복문.
- 기본 Mixamo PA의 20개 바디 / 19개 D6 관절, 총 73 kg 질량 분배.
- 바디·관절 편집, 계층과 연결 그래프, 관절 프레임과 Swing / Twist 시각화.
- 무릎·팔꿈치의 Twist 힌지와 잠긴 Swing 두 축.
- 애니메이션을 따르는 Kinematic PA, Passive Ragdoll, 물리 결과의 스키닝 반영.
- 이동용 Query 캡슐과 와이어 표시, Passive Solver 반복 16/4.

이번 변경은 이 기반 위에 Active Ragdoll 모드, 목표 포즈와 D6 드라이브, 골반 처리, 드라이브 편집·저장, 일부 관절 제한 여유, 성능 조정, PVD 표시 수정을 추가한다. 기존 워커 스레드 수나 고정 시간 간격을 다시 바꾸지는 않았다.

## 2. 현재 모드별 동작

`SkeletalMeshComponent`에 `PhysicsAnimationMode`를 두고 다음 세 모드를 구분한다. 아래 바디 개수는 기본 PA 기준이다.

| 모드 | 바디 상태 | 목표 애니메이션 | 관절 드라이브 | 이동 컴포넌트·캡슐 |
|---|---|---|---|---|
| Animation | 20개 Kinematic | 계속 평가 | OFF | 기존 캐릭터 제어 |
| Passive Ragdoll | 20개 Dynamic | 정지 | OFF | 비활성화 |
| Active Ragdoll + 골반 지지 | 골반 1개 Kinematic, 나머지 19개 Dynamic | 고정 스텝마다 평가 | 기본 PA의 19개 관절 구동 | 비활성화 |
| Active Ragdoll + 골반 지지 해제 | 20개 Dynamic | 고정 스텝마다 평가 | 계속 구동 | 비활성화 |

Active의 `Angular drives` 체크만 끄면 Active 모드와 목표 애니메이션 시간은 유지된다. 골반 지지가 켜져 있으면 골반도 계속 애니메이션을 따른다. 따라서 이 체크를 끄는 것과 Passive로 전환하는 것은 동작이 다르다.

## 3. 한 프레임 안에서 실행되는 순서

핵심 진입점은 기존 `ModelViewer::Update()`와 `World::Tick()`이다. 새 동작은 그 안의 고정 물리 스텝과 메시 컴포넌트에 연결했다.

```text
ModelViewer::Update(deltaT)
  World::Tick(deltaT)
    PhysicsWorld::Update(...)
      PhysicsWorld::Step(beforeStep)             // 고정 1/60초
        Actor::FixedTick(...)
          SkeletalMeshComponent::FixedTickComponent(...)
            대기 중인 드라이브 설정 적용
            AnimInstance::UpdateAnimation(...)
            EvaluateAnimationPhysicsPose(...)
            Ragdoll::UpdateDriveTargets(...)
              골반 Kinematic 목표 갱신
              D6 목표 상대 회전·각속도 갱신
        PxScene::simulate(1/60)
        PxScene::fetchResults(true)
    Actor의 일반 Tick
      SkeletalMeshComponent::UpdatePhysicsPose()
  Renderer::UpdateWorldRenderData(...)
    물리 결과 포즈를 DX12 스키닝 데이터에 반영
```

렌더 프레임 하나에 물리 스텝이 여러 번 필요하면 목표 애니메이션과 관절 목표도 각 물리 스텝마다 함께 갱신한다. `fetchResults(true)`가 끝난 뒤 일반 Tick에서 물리 결과를 읽는다.

목표 애니메이션은 `AnimInstance`에, 최종 물리 포즈는 `SkeletalMeshComponent::m_PhysicsPose`에 따로 보관한다. 물리 결과로 목표 애니메이션을 덮어쓰지 않는다. Active 렌더링은 물리 포즈를 선택한다.

임포트된 본의 스케일·늘어남은 `PhysicsBoneFrame()`으로 강체 프레임과 분리하고, 렌더링 포즈를 만들 때 다시 반영한다. PA 바디가 없는 손가락·발가락 등은 물리 부모 아래에서 애니메이션의 로컬 자세를 사용한다.

## 4. 애니메이션을 관절 목표로 변환하는 방법

구현 위치: [PhysicsPose.cpp](../MiniEngine/Model/PhysicsPose.cpp)의 `EvaluateJointDriveTarget()`, [RagdollDrive.cpp](../MiniEngine/Model/RagdollDrive.cpp)의 `UpdateDriveTargets()`.

본의 로컬 회전을 D6 목표에 그대로 넣지 않는다. 본과 바디 사이 오프셋, 부모·자식 관절 프레임을 모두 반영한다.

```text
목표 바디 월드 = PhysicsBoneFrame(애니메이션 본 월드) × PA 바디 로컬 변환
부모 관절 월드 = 부모 목표 바디 월드 × parentFrame
자식 관절 월드 = 자식 목표 바디 월드 × childFrame
목표 상대 회전 = rotation(inverse(부모 관절 월드) × 자식 관절 월드)
```

이 상대 회전은 Constraint 로컬 좌표계 기준이다. X가 Twist이고 Y/Z가 Swing이라는 D6 규칙을 사용한다. 기존 관절 프레임을 이용하므로 월드 축이나 본 로컬 축을 임의로 대입하지 않는다.

목표가 관절 제한 밖이면 다음과 같이 보정한다.

1. 상대 Quaternion을 Twist와 Swing 성분으로 분리한다.
2. Locked 성분은 0으로 만들고, Limited Twist는 `twistMin..twistMax` 안에 제한한다.
3. Swing은 quarter-angle로 얻은 두 성분과 관절 반각을 이용해 타원 내부로 제한한다. 두 축을 각각 범위에 넣는 것만으로 놓칠 수 있는 대각선 초과도 처리한다.
4. 보정한 회전을 다시 합쳐 D6 목표로 전달한다.

UI의 두 오차는 의미가 다르다.

| 표시 | 의미 |
|---|---|
| `Max target error` | 실제 물리 관절과 제한 보정 후 목표 사이의 최대 회전 오차 |
| `Target limit correction` | 원본 애니메이션 목표를 PA 제한 안으로 옮기면서 발생한 최대 보정 각도 |

물리 추종 오차가 작아도 제한 보정이 크면 원본 애니메이션과는 다른 자세가 된다. 두 값을 따로 봐야 한다.

## 5. PhysX 드라이브와 토크 처리

관절 구동은 `PxD6JointDrive(stiffness, damping, maxTorque, false)`로 설정한다. 마지막 인자가 `false`이므로 **힘 기반 PD**이며 가속도 기반 모드가 아니다. `eDRIVE_LIMITS_ARE_FORCES`를 켜서 각 구동 축의 토크 한도를 Nm 기준으로 사용한다.

프로젝트는 목표 회전·각속도와 게인을 전달하고, PhysX가 관절·접촉 제약과 함께 구동 토크를 계산한다. 별도의 수동 PD 토크를 중복으로 더하지 않는다. 개념적으로 자세 오차에는 강성, 목표 각속도와 현재 각속도의 차이에는 감쇠가 작용하며, 실제 해는 PhysX Solver가 결정한다.

- 세 각도 축이 모두 잠겨 있지 않으면 기본 설정은 SLERP를 사용한다. 이 경우 `drives[0]`의 공통 게인을 쓴다.
- 무릎·팔꿈치는 Swing Y/Z가 Locked이므로 SLERP 대신 Twist 드라이브를 사용한다.
- 목표 각속도는 이전 목표와 현재 목표 Quaternion의 차이를 고정 시간으로 나눠 계산하고, 최대 20 rad/s로 제한한다.
- 같은 회전의 `q` / `-q` 표현 때문에 큰 회전으로 해석되지 않도록 부호를 정리한다.
- 현재 SDK의 Solver 규약에 맞춰 SWING_TWIST 각속도 목표는 부호를 반전하고, SLERP는 그대로 전달한다.
- 목표가 변했거나 실제 자세가 목표에서 0.03 rad보다 멀면 필요한 바디를 깨워 정적 목표 추종이 수면 상태 때문에 중단되지 않도록 한다.

기본 게인은 [PhysicsAsset.cpp](../MiniEngine/Model/PhysicsAsset.cpp)의 `ConfigureDefaultDrives()`에서 계산한다.

```text
m = 해당 관절의 자식 바디와 모든 하위 바디의 합산 질량(kg)
stiffness = clamp(m × 1600, 1600, 96000)
damping   = stiffness × 0.005
maxTorque = clamp(m × 32, 32, 1400)
```

몸통 가까운 관절은 더 많은 하위 질량을 지지하므로 손끝 관절과 같은 강도로 구동하지 않는다. 이 식은 **추종을 위한 초기 튜닝값**이다. 사람의 근력 측정 자료로 얻은 값은 아니다. 기존 73 kg 바디 질량 분포는 유지했다.

## 6. 골반과 모드 전환 수정

### Active 골반은 매 스텝 애니메이션을 따른다

중간 구현에서는 지지한 골반이 Active 진입 순간 자세에 머물렀다. 현재는 `UpdateDriveTargets()`가 매 고정 스텝마다 애니메이션 골반의 월드 위치·회전에 PA 바디 오프셋을 반영해 `setKinematicTarget()`을 호출한다.

`SetRootSupported(true)`에서 현재 자세를 넣는 것은 다음 목표 갱신 전까지의 초기값이다. 이후에는 애니메이션 트랙을 계속 따른다. 나머지 바디는 Kinematic으로 바꾸지 않는다.

이는 Kinematic 골반을 통한 위치·회전 지지다. 골반을 Dynamic으로 두고 힘으로 월드 목표를 추종하거나, 스스로 균형을 잡는 제어는 아직 없다.

### Passive에서는 골반도 해제한다

`SkeletalMeshComponent::SetPhysicsMode(PassiveRagdoll)`에서 드라이브를 끄고 `SetRootSupported(false)`를 호출한다. 목표 평가 오류로 Active에서 Passive로 복귀하는 경로도 골반을 해제하며 원래 오류 메시지는 보존한다.

골반을 해제할 때 마지막 시뮬레이션의 선속도·각속도를 저장한 뒤 Dynamic으로 전환하고 복원한다. 전환 순간의 위치·회전도 유지한다. UI에서는 Passive 중 골반 지지 체크를 비활성화한다.

Active ↔ Passive는 기존 바디와 관절을 재사용한다. Passive에서 다시 Active로 바꿔도 골반 지지는 자동으로 켜지지 않으며 사용자가 명시적으로 켤 수 있다.

### 초기 속도와 Animation 복귀

Animation → Active에서는 유효한 이전 Kinematic 목표 이력이 있으면 바디별 애니메이션 선속도·각속도를 전달한다. 텔레포트 등으로 이력이 무효면 캐릭터 이동 속도와 각속도 0을 초기값으로 사용한다.

`Character::SetPhysicsMode()`는 처음 래그돌에 들어갈 때 이동·캡슐의 활성 상태를 저장하고 둘을 비활성화한다. `Stop / return to start`는 기존 Actor 루트 위치에서 Animation과 저장된 이동·캡슐 상태를 복구한다. 넘어진 위치에서 자연스럽게 일어나는 전환이나 자세 블렌딩은 추가하지 않았다.

## 7. PA v3와 에디터 연결

`PhysicsAngularDrive`에 `enabled`, `stiffness`, `damping`, `maxTorque`를 추가했다. 각 `PhysicsConstraint`는 Twist X / Swing Y / Swing Z용 `drives[3]`와 `useSlerpDrive`를 가진다.

PA JSON은 v3로 저장하며 다음 항목을 추가한다.

- `driveUnits: "Nm-radians-seconds"`
- 관절별 `driveMode: "slerp"` 또는 `"swingTwist"`
- 관절별 세 개의 `drives` 설정

v1/v2 파일은 계속 읽을 수 있으며 드라이브 기본값은 OFF이다. 기존 파일을 불러왔다고 자동으로 모터가 켜지지 않는다. v3 로딩은 단위·모드·배열 개수·유한한 비음수 게인을 검사하고, Locked 축을 포함한 SLERP 설정은 거부한다. 잘못된 파일은 기존 PA를 교체하지 않는다.

스켈레탈 메시 뷰어의 Constraint 패널에 `Active Ragdoll drives`를 추가했다. 관절별 활성화, 강성, 감쇠, 최대 토크, SLERP 사용 여부를 편집할 수 있다. 각도 축을 Locked로 바꾸면 유효하지 않은 SLERP 설정도 해제한다.

실행 중인 캐릭터에 적용하는 경로는 다음과 같다.

| 조작 | 적용 범위와 시점 |
|---|---|
| `Use default PD gains` | 현재 캐릭터 PA에 기본 드라이브 설정을 다음 고정 스텝에 적용 |
| `Apply editor drive settings` | 부모·자식 연결이 일치하는 에디터 PA의 드라이브 값과 방식만 다음 고정 스텝에 적용 |
| `Use editor PA` | 래그돌을 멈춘 상태에서 PA 전체 교체 |
| `Save character PA` | 실행 캐릭터에 적용된 PA를 v3로 저장; 적용 대기 중에는 비활성화 |

드라이브 적용은 `QueueDriveSettings()` → `FixedTickComponent()` → `ApplyDriveSettings()` 순서다. 월드가 일시정지되어 있으면 재개하거나 단일 스텝을 실행할 때 적용된다. 라이브 드라이브 적용으로 바디 형상·질량·관절 프레임·각도 제한은 바뀌지 않는다. 최종 검증은 실행 중 PA의 관절 축 설정을 기준으로 수행한다.

## 8. 기본 관절 제한에 추가한 여유

원본 애니메이션 목표가 기존 제한에 잘려 다른 자세로 구동되는 현상을 줄이기 위해 `BuildMixamoPhysicsAsset()`의 일부 값을 늘렸다. 표의 Swing 값은 전체 폭이 아닌 **반각**이며, 단위는 도다.

| 대상 | 항목 | 이전 | 현재 |
|---|---|---:|---:|
| 양쪽 Shoulder | Swing Y 반각 | 17.052221 | 27.052221 |
| 양쪽 Shoulder | Swing Z 반각 | 20.855535 | 30.855535 |
| 양쪽 Hand | Twist 범위 | −5 ~ +5 | −20 ~ +20 |
| 양쪽 Foot | Swing Y 반각 | 34.654347 | 44.654347 |
| 양쪽 Foot | Swing Z 반각 | 27.971366 | 32.971366 |
| 양쪽 Foot | Twist 범위 | −6 ~ +6 | −10 ~ +10 |

관절 프레임과 편향, 무릎·팔꿈치의 Twist 힌지는 유지했다. 이 여유는 임포트 애니메이션을 위한 튜닝이며 새로운 인체 가동 범위 자료에서 가져온 수치가 아니다. 손목 Twist 여유도 별도 전완 회내·회외 관절을 구현한 것은 아니다.

샘플링 비교에서 원본 관절 회전은 그대로이고, 제한으로 인한 보정만 줄어든 것을 확인했다.

| 클립 / 관절 | 최대 목표 보정 이전 → 현재 |
|---|---|
| Idle / 왼쪽·오른쪽 Shoulder | 7.73° / 8.50° → 0.04° / 0.00° |
| Idle / 왼쪽·오른쪽 Hand | 36.91° / 22.81° → 21.91° / 7.81° |
| Walk / 왼쪽·오른쪽 Hand | 12.10° / 12.57° → 0.04° / 0.00° |
| Walk / 왼쪽·오른쪽 Foot | 18.18° / 13.43° → 8.14° / 3.44° |
| Run / 왼쪽·오른쪽 Foot | 40.18° / 41.05° → 30.14° / 30.95° |

따라서 제한 보정이 모두 사라진 상태는 아니다. 특히 일부 손목·발목과 Locked Swing 성분은 원본 애니메이션과 차이가 남는다. 이미 저장된 PA 파일에는 생성 코드의 변경이 자동으로 덮어써지지 않는다.

발목 범위에 의도적인 여유를 추가했으므로 `AnatomicalLimitTests`의 범위 밖 검사도 기존 기준 끝점에서 3° 바깥 대신 15° 바깥을 검사하도록 수정했다. 기존 기준 끝점이 허용되는지와 새 여유를 넘어서는 각도가 거부되는지를 함께 확인한다.

## 9. Debug 성능과 Solver 반복 횟수

현재 `Ragdoll::SetDrivesEnabled()`가 각 바디의 반복 횟수를 설정한다.

| 상태 | 위치 반복 | 속도 반복 |
|---|---:|---:|
| Active 드라이브 ON | 32 | 8 |
| Passive 또는 드라이브 OFF | 16 | 4 |

Active 초기 구현의 48/12에서 32/8로 낮췄다. 이 값은 같은 물리 스텝 안에서 관절과 접촉 제약의 해를 다듬는 반복 횟수다. 물리 시간 자체를 여러 번 진행하거나 스텝 간격을 바꾸는 설정은 아니다.

로컬 Debug 진단에서 목표 평가 비용은 약 0.2 ms였고, 주된 비용은 `simulate/fetchResults`였다. 진단 실행에서 Idle의 전체 물리 처리 구간은 48/12 약 13.98 ms, 32/8 약 8.06 ms, 16/4 약 5.23 ms였다. Run을 16/4로 낮추면 최대 추종 오차가 약 40.8°까지 증가해 현재 값은 32/8로 정했다.

위 수치는 해당 로컬 진단 조건의 관측값이다. 반복 실행과 접촉 상태에 따라 달라지며 전체 렌더 FPS를 뜻하지 않는다. 진단에 사용한 약한 게인 설정은 최종 기본값에 적용하지 않았다. 최종 변경은 반복 횟수 감소이며 고정 시간, 워커 수, 기본 PD 게인은 유지했다.

## 10. PVD 관절 크기와 제한 초과 색상 수정

### 크기를 줄여도 큰 도형이 남던 원인

현재 D6 설정에서 SDK가 Swing 콘과 이를 둘러싼 Pyramid를 함께 그렸다. Pyramid는 반경 1 단위의 일반 디버그 선으로 그려져 PVD의 `Joints scale`과 관절 표시 설정을 따르지 않았다.

`ExtD6Joint.cpp`에서 콘이 있고 Pyramid가 그 콘을 완전히 포함하면 중복 Pyramid 표시를 생략한다. 더 좁은 제한을 나타내거나 독립적으로 사용되는 Pyramid는 유지한다. 실제 Solver 제한은 수정하지 않았다.

### 실제 자세에 따라 제한 초과를 표시한다

기존 각도 시각화 경로는 실제 초과 여부를 전달하지 않았다. 다음 경로에 상태를 연결했다.

```text
D6JointVisualize: 현재 부모·자식 관절 상대 회전으로 범위 초과 계산
  PxConstraintVisualizer: active 인자 전달
  NpPvdSceneClient → PvdUserRenderer
  기존 PVD 이벤트의 active 필드에 기록
```

Twist는 비대칭 최솟값·최댓값, 두 축 Swing은 타원 범위, 한 축 Swing 부채꼴과 Double Cone은 해당 축 범위를 검사한다. 경계의 부동소수점 오차와 동등한 Quaternion 부호도 처리한다. `CmVisualization`의 즉시 디버그 렌더러에도 같은 상태를 전달한다.

허용 범위 안은 초록, 초과하면 빨강이다. Kinematic 바디도 실제 자세로 판정한다. 이는 시각화이며 Kinematic 바디를 관절 제한 안으로 물리적으로 밀어 넣는 기능은 아니다.

엔진 PA 에디터의 **빨간 Swing / 초록 Twist**는 종류를 구분하는 색이다. PVD의 **초록 정상 / 빨강 초과**와 의미가 다르다.

PVD 통신 형식은 그대로지만 로컬 C++ 시각화 인터페이스가 바뀌므로 SDK와 사용 프로그램을 함께 다시 빌드해야 한다. 일반 ModelViewer 빌드가 이 의존성을 처리한다. 이전 녹화 데이터에는 이전 이벤트가 남으므로 변경된 SDK의 표시를 확인할 때는 재연결 후 새 프레임을 기록한다.

### 마지막 요청: 표시 크기 5배

이 PC의 PVD `Joints scale`을 **0.04 → 0.20**으로 변경했다. 관절 각도·질량·토크와 무관한 화면 표시 설정이다.

- 저장 위치: `HKCU\Software\NVIDIA Corporation\PhysXVisualDebugger3`, `jointScale`.
- `Joints`는 All 설정이다. PVD를 다시 실행하면 저장된 설정을 읽는다.
- 다른 PC에서는 PVD 설정에서 `Joints = All`, `Joints scale = 0.20`으로 맞춘다.
- 레지스트리 변경 자체는 Git 커밋에 포함되지 않는다. 이 문서에 값과 재현 방법을 기록한다.
- 변경 전 백업은 Git 제외 경로 `.tools/pvd/joint-scale-backup-20261018-230058-8074614.json`에 있다.

## 11. 검증 결과와 측정 범위

2026-10-18 최종 Solver 32/8 구현에 대해 아래 실행 기록을 확인했다. 이후 이번 정리 작업에서는 문서만 추가·정리했다.

| 검증 | 결과 |
|---|---|
| ModelViewer Debug x64 빌드 | 성공 |
| ActorComponentTests Debug x64 빌드·실행 | 통과 |
| `ModelViewer -ragdolltest` 실제 임포트 모델 통합 검증 | 전체 통과 |
| PhysX Debug Smoke Test | 통과; 각도 도형·색상·PVD 상태 바이트 검사 포함 |
| `git diff --check` | 통과 |

단위 테스트는 목표 프레임 변환, 타원 제한, 정적·이동 힌지, 3축 SLERP, 토크 제한, 외력 후 회복, 모드 전환의 자세·속도 보존을 검사한다. 정적 힌지 오차는 약 0.0232 rad, 이동 힌지 최대 오차는 약 0.0135 rad, 이동 SLERP 최대 오차는 약 0.0327 rad였다.

통합 검증은 실제 20바디 / 19관절 모델로 Idle / Walk / Run, 클립 블렌딩, DX12 물리 포즈 업로드, 골반 트랙 추종, Passive 해제, 외력 반응, 드라이브 라이브 적용, 일시정지·단일 스텝·슬로모션, 30/60/144 FPS 입력, 지면 접촉, 넘어진 자세의 Active 전환, Actor 삭제·자원 해제를 검사한다.

최종 32/8 통합 실행의 주요 결과는 다음과 같다.

| 목표 클립 | 평균 `World::Tick` | 최대 목표 추종 오차 | 최대 관절 앵커 간격 |
|---|---:|---:|---:|
| Idle | 10.7122 ms | 약 2.5° | 약 1.45 mm |
| Walk | 8.49223 ms | 약 8.5° | 약 2.86 mm |
| Run | 8.34188 ms | 약 13.9° | 약 4.90 mm |

측정 조건은 Debug, PVD 미연결, 애니메이션을 따르는 골반을 지지하고 지면 간섭을 피하도록 들어 올린 캐릭터다. 평균 시간은 360스텝, 오차 최댓값은 초기 안정화 구간 뒤를 측정한다. 지면 접촉은 별도 항목에서 검증했다. 이 수치는 렌더링·UI 전체 FPS나 실제 WASD 이동 성능이 아니며, 추종 오차의 기준은 **PA 제한 보정 후 목표**다.

PVD 표시는 디버그 도형과 직렬화된 상태 바이트를 자동 검증했다. 최종 PVD 화면을 직접 열어 엔진과 육안으로 교차 확인하는 항목은 아직 남아 있다. 최종 설정의 Profile / Release 결과도 이번 Debug 결과와 동일하다고 주장하지 않는다.

원본 로그는 Git에서 제외된 로컬 경로에 보관한다.

- `.tools/active-ragdoll/solver32-viewer-build.txt`, `solver32-tests-build.txt`, `solver32-tests.txt`, `solver32-integration.txt`
- `.tools/active-ragdoll/limits-before.txt`, `limits-after.txt`, `limits-comparison.txt`, `active-cost.txt`
- `.tools/pvd/visualization-smoke.txt`

### 검증 재실행

저장소 루트의 PowerShell에서 실행한다. VS 설치 경로가 다르면 `$msbuildPath`를 조정한다.

```powershell
$msbuildPath = 'C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe'
& $msbuildPath .\MiniEngine\ModelViewer\ModelViewer.sln /p:Configuration=Debug /p:Platform=Windows /m /v:minimal /nologo
& $msbuildPath .\MiniEngine\Tests\ActorComponentTests.vcxproj /p:Configuration=Debug /p:Platform=x64 /m /v:minimal /nologo
& .\.tools\actor-component-tests\Debug\ActorComponentTests.exe
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ThirdParty\PhysX\Build.ps1 -Configuration Debug -RunSmokeTest
```

통합 검증은 `MiniEngine/ModelViewer`를 작업 디렉터리로 하여 `MiniEngine/Build/x64/Debug/Output/ModelViewer/ModelViewer.exe -ragdolltest <보고서 절대 경로>`로 실행한다. 프로그램 종료 코드만 보지 말고 보고서에 `PASS: all character ragdoll integration checks`가 있으며 `FAIL:` / `FAILED`가 없는지 확인한다.

## 12. 파일별 책임과 변경 내용

| 파일 | 이번 변경 |
|---|---|
| [SkeletalMeshComponent.h/.cpp](../MiniEngine/Model/Components/SkeletalMeshComponent.cpp) | 모드 구분, 고정 스텝 목표 갱신, 설정 적용 대기, 오류 시 Passive 복귀 |
| [SkeletalMeshPhysics.cpp](../MiniEngine/Model/Components/SkeletalMeshPhysics.cpp) | 모드 전환, 목표와 물리 포즈 분리, 골반 해제, 자동 Locomotion 상태 저장·복원 |
| [Character.h/.cpp](../MiniEngine/Model/GameFramework/Character.cpp) | Character 단위 모드 진입, 이동·캡슐 상태 관리 |
| [PhysicsAsset.h/.cpp](../MiniEngine/Model/PhysicsAsset.cpp) | 드라이브 데이터, PA v3 저장·로드·검증, 하위 질량 기반 기본 게인 |
| [PhysicsPose.h/.cpp](../MiniEngine/Model/PhysicsPose.cpp) | 관절 목표 회전 변환·제한 보정, 기본 PA 관절 범위 여유 |
| [Ragdoll.h/.cpp](../MiniEngine/Model/Ragdoll.cpp) | 드라이브 상태·이력 수명, 애니메이션 속도 전달, 지원 바디를 제외한 외력 적용 |
| [RagdollDrive.cpp](../MiniEngine/Model/RagdollDrive.cpp) — 신규 | D6 모터 설정, 목표·각속도, 골반 지지·해제, 런타임 게인 적용, 오차 계산 |
| [ModelViewer.cpp](../MiniEngine/ModelViewer/ModelViewer.cpp) | Active / Passive 버튼, 골반·드라이브 체크, 진단값·적용·저장 UI |
| [SkeletalMeshViewerConstraints.cpp](../MiniEngine/ModelViewer/SkeletalMeshViewerConstraints.cpp) | 관절별 드라이브 편집 |
| [ActiveRagdollTests.cpp](../MiniEngine/Tests/ActiveRagdollTests.cpp) — 신규 | 단일 관절 구동·좌표·토크·전환·표시 회귀 검증 |
| [ActiveRagdollValidation.cpp](../MiniEngine/ModelViewer/ActiveRagdollValidation.cpp) — 신규 | 실제 임포트 캐릭터의 Active 통합 검증 |
| `ActorComponentTests.cpp`, `PhysicsAssetTests.cpp`, `AnatomicalLimitTests.cpp`, `RagdollValidation.cpp` | 새 테스트 연결, 저장 버전·제한·수명 검증 확장 |
| Model / ModelViewer / ActorComponentTests 프로젝트 및 필터 | 신규 소스를 빌드 대상에 등록 |
| `ThirdParty/PhysX/.../ExtD6Joint.cpp` | 각도 초과 판정, 중복 Pyramid 표시 제거 |
| `PxConstraintDesc.h`, `NpPvdSceneClient.cpp`, `PxPvdUserRenderer.h/.cpp`, `CmVisualization.h/.cpp` | 시각화 상태 인자 전달·기록·색상 적용 |
| [SmokeTest.cpp](../ThirdParty/PhysX/tests/SmokeTest.cpp) | Kinematic 제한 표시, 범위 복귀, Quaternion 부호, 표시 크기, PVD 상태 바이트 검사 |
| [README.PARS.md](../ThirdParty/PhysX/README.PARS.md) | 로컬 SDK 패치와 재빌드·PVD 표시 사용법 기록 |
| [액티브 래그돌 계획](../액티브%20래그돌%20계획.md), 이 문서 | 구현 현황·검증 결과·미구현 범위 정리 |

## 13. 현재 사용 순서와 다음 작업

1. `Character Animation → Ragdoll / PhysicsAsset`에서 캐릭터 PA를 확인한다.
2. 드라이브가 없는 기존 PA라면 `Use default PD gains`를 적용하고 다음 물리 스텝을 진행한다.
3. `Active Ragdoll`을 누른다. Animation에서 진입할 때 골반 지지는 기본적으로 켜져 있다.
4. Idle / Walk / Run 버튼으로 목표 클립을 바꾸고, 물리 자세와 두 오차 표시를 확인한다.
5. `Play`를 끄면 목표 애니메이션만 멈추고 물리는 정지한 목표를 추종한다. `Pause world`는 둘을 멈추며 단일 스텝은 둘을 함께 진행한다.
6. `Passive Ragdoll`은 모터와 골반 지지를 모두 해제한다. `Stop / return to start`는 시작 위치의 Animation 제어로 돌아간다.

다음 구현 대상은 **WASD 실제 이동과 Active Ragdoll 연결**이다. 이동 컴포넌트·Query 캡슐의 활성 정책, 자신의 PA 바디를 이동 쿼리에서 제외하는 처리, 이동 속도에 따른 Idle / Walk / Run 선택, 움직이는 Actor 기준 골반 월드 목표를 함께 연결해야 한다. Passive 전환과 오류 복귀 시 이동을 다시 중단하는 처리도 필요하다.

목표 자세 고스트 표시, 드라이브 강도의 점진적 전환, Dynamic 골반의 월드 구동, 균형 유지, 발 디딤, 일어나기, 일부 관절의 남은 목표 제한 보정은 후속 작업이다. 현재의 골반 지지·클립 추종 검증 완료를 이러한 기능의 완료로 보지 않는다.
