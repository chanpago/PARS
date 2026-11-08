# 양발 기준 키네마틱 루트 정렬 검토 — 2026-11-08

상태: **보고서 제시 후 사용자 커밋 승인(2026-11-08). 승인한 후보의 코드·애셋 해시와 검증 패치가 일치함을 확인했다.** 아래에 기록한 잔여 문제와 검증 범위는 그대로다.

A영역 2초 유지 후 양발의 실제 접지를 확인하고, 현재 양발 발바닥 중점에 애니메이션의 양발 중점을 맞추도록 루트의 X·Y·Z를 보정했다. 일반 충격에서 복귀 전후 발 중점 높이 변화는 **5.225 → 0.001mm**, 수평 이동은 **13.168 → 0.018mm**였다. 골반 상승은 **17.250 → 12.027mm**로 줄었으며, 자세가 Idle로 펴질 때 생기는 상승은 남아 있다.

## 기준과 후보

- 기준 커밋: `10a1249da416bda5833aa547932926fed0a23b4b`.
- 기준 실행 파일: 위 커밋에 [계측 패치](../.tools/foot-root-alignment-20261107/measurement.patch)만 적용해 빌드한 `baseline.exe`. 입력·제어식은 기준 그대로이며, 발·루트 CSV와 선택적 검증만 추가했다.
- 후보: 기준 커밋에 [후보 패치](../.tools/foot-root-alignment-20261107/candidate.patch)를 적용한 현재 작업 트리.
- 후보 패치 SHA-256: `75f6fac538ac45dd0c25617efc4ad0b34d4af4419db1dee6476d515451496cb9`.
- [소스·애셋·실행 파일·CSV 해시 목록](../.tools/foot-root-alignment-20261107/manifest.json). 기존 미추적 문서는 수정하지 않았다.

## 변경한 흐름

1. 기존처럼 완료된 물리 스텝마다 COM의 A영역 연속 유지 시간을 누적한다. A영역 이탈·새 충격·접촉 이력 변경 시 초기화하고 일시정지나 같은 스텝 중복 조회는 누적하지 않는다.
2. 2초를 채워도 실제 양발 접지가 없으면 액티브 래그돌로 기다린다. **양발 접지를 2초 유지하라는 조건은 아니다.** A영역 체류 시간은 계속 누적하고, 양발의 `available`, `hasSample`, `grounded`가 모두 참인 새 스텝에서 전환한다. 레이 적중이나 접촉 유예를 실제 접지 대신 사용하지 않는다.
3. 실제 발바닥 중심 두 개의 평균과, 현재 애니메이션의 발바닥 중심 두 개의 평균을 구한다. 애니메이션 발 본은 PA와 동일하게 스케일·전단을 제거한 강체 좌표계로 변환하고 `soleInBone`을 적용한다.
4. `root += actualSoleMidpoint - animatedSoleMidpoint`를 X·Y·Z에 적용한다. 기존 골반 X·Z 정렬을 대체한다. 발 자료가 없거나 유효하지 않은 보정량이면 오류를 반환한다.
5. 보정된 루트 좌표계에서 현재 물리 자세를 다시 캡처한 뒤 키네마틱으로 전환한다. 기존 0.5초 자세 보간과 보간 도중 재충격 처리는 유지한다. 복원 스텝은 추가하지 않았다.

주요 구현: [전환과 루트 정렬](../MiniEngine/Model/Components/SkeletalMeshRecovery.cpp), [복귀 판정](../MiniEngine/Model/RagdollRecovery.cpp), [접지 대기 검사](../MiniEngine/Tests/RagdollRecoveryTests.cpp), [실제 캐릭터 계측](../MiniEngine/ModelViewer/RagdollImpactValidation.cpp).

## 같은 조건의 실제 캐릭터 검증

- Windows x64 Debug, MSVC v143, PhysX TGS, 중력 `(0,-9.81,0)`, 고정 간격 1/60초, 시간 배율 1, 900스텝/15초.
- 모델·애니메이션: `Assets/Characters/Mixamo/Converted/Idle/Idle.mini`. PA: `Mixamo.physics.json`, 22개 바디, 질량 72.999999kg. PA·애니메이션·물리 파라미터는 변경하지 않았다.
- 기본 y=0 바닥, 반크기 `(15,0.25,15)m`, 정적/동적 마찰 `0.9/0.7`, 반발 0. 기존 PA 발 마찰 `0.6/0.6`, minimum 결합을 유지했다.
- 초기 더미 위치 `(3.5,capsuleHalfHeight,-2.5)`에서 기존 PA 최하단 바닥 정렬을 수행한다. 이동·캡슐 충돌을 끈 Idle 더미, 자동 지지·자동 모드 전환 활성화, 기본 복귀 대기 2초·보간 0.5초.
- 일반 구 충격은 2kg·7.5m/s·반지름 0.15m. 강한 충격 case 3만 기존 시험대로 20kg을 사용한다. 120번째 프레임에 발사하고, case별 재발사 조건도 기준·후보에서 같다.
- 180번째 프레임에서 120번의 일시정지 틱을 넣어 유지 시간이 늘지 않음을 확인했다. 공중 지지 시험으로 캐릭터 검증을 대체하지 않았다.

| 시나리오 | 기준과 후보 결과 | 후보 원본 |
|---|---|---|
| 0: 무충격 Idle | 15초 내내 키네마틱 | [로그](../.tools/foot-root-alignment-20261107/candidate-case-0.txt) |
| 1: 일반 구 충격 | frame 133 액티브 진입, frame 254 복귀, frame 283 보간 종료 | [로그](../.tools/foot-root-alignment-20261107/candidate-case-1.txt) |
| 2: 복귀 완료 후 재충격 | 양쪽 모두 진입·복귀 2회, 복귀 frame 254·419 | [로그](../.tools/foot-root-alignment-20261107/candidate-case-2.txt) |
| 3: 강한 구 충격 | 양쪽 모두 진입 1회·복귀 0회, 자세 붕괴 | [로그](../.tools/foot-root-alignment-20261107/candidate-case-3.txt) |
| 4: 보간 중 재충격 | frame 268 재진입, frame 390 재복귀 | [로그](../.tools/foot-root-alignment-20261107/candidate-case-4.txt) |
| 5: 타이머 누적 중 재충격 | 타이머 초기화 후 frame 318 복귀 | [로그](../.tools/foot-root-alignment-20261107/candidate-case-5.txt) |

모든 후보 복귀에서 직전 실제 양발 접지와 루트 XYZ 보정량을 검사했다. 보정량 계산 오차 최대는 3.77267e-07m였다. 강한 충격의 PASS는 진입·유효 상태·PhysX 오류 없음에 대한 결과이며 균형 회복 성공이 아니다. 강한 충격의 기준·후보 본 CSV는 SHA-256까지 동일하다.

## 일반 충격 복귀 구간 비교

기준과 후보 모두 마지막 액티브 샘플 frame 253에서 보간 완료 frame 283까지 비교했다. 계측은 발목이 아닌 발바닥 좌표를 사용한다.

| 지표 | 기준 | 후보 | 차이 |
|---|---:|---:|---:|
| 양발 중점 높이 변화 mm | 5.224677 | 0.001340 | -5.223337 |
| 양발 중점 수평 이동 mm | 13.168022 | 0.018028 | -13.149994 |
| 골반 상승 mm | 17.250000 | 12.027000 | -5.223000 |
| 전환 첫 스텝의 본 이동 최대 mm | 0.150223 | 0.145000 | -0.005223 |

후보의 루트 보정은 약 `(-24.088,-5.223,+27.533)mm`였다. 첫 스텝 본 이동 최대 0.145mm는 보간·애니메이션 갱신을 포함한 1/60초 샘플 차이이며, 순간 이동량을 별도로 계측한 값은 아니다.

![복귀 구간의 골반과 발 중점 변화](../.tools/foot-root-alignment-20261107/return-comparison.png)

아래는 복귀 보간 30프레임(frame 254~283)의 애니메이션 목표 대비 본 추종 오차다. 각 버전의 루트에 맞춘 애니메이션을 목표로 삼았으므로, 루트 정렬 변경에 따라 위치 목표 자체가 달라진다. 팔다리는 팔·전완·허벅지·종아리 샘플을 합산했다. 최대값 괄호는 발생 시각(초)이다.

| 지표 | 기준 RMS | 후보 RMS | RMS 차이 | 기준 최대 (초) | 후보 최대 (초) |
|---|---:|---:|---:|---:|---:|
| 골반 위치 mm | 10.431256 | 10.689290 | +0.258034 | 17.531000 (4.250) | 17.999700 (4.250) |
| 골반 회전 rad | 0.005368 | 0.005368 | +0.000000 | 0.008569 (4.250) | 0.008569 (4.250) |
| 양발 위치 mm | 20.983149 | 18.278065 | -2.705085 | 46.836600 (4.250) | 35.032300 (4.250) |
| 양발 회전 rad | 0.059642 | 0.059642 | +0.000000 | 0.109502 (4.250) | 0.109502 (4.250) |
| 팔다리 위치 mm | 14.099099 | 15.604872 | +1.505773 | 36.945600 (4.250) | 46.157300 (4.250) |
| 팔다리 회전 rad | 0.045503 | 0.045503 | +0.000000 | 0.163498 (4.250) | 0.163498 (4.250) |

양발 위치 RMS는 줄었지만 골반 위치 RMS는 약 0.258mm, 팔다리 위치 RMS는 약 1.506mm 증가했다. 골반 수평 위치 대신 발 중점을 기준으로 목표를 옮긴 결과이며, 모든 본의 추종 오차가 개선됐다고 판단하지 않는다. 회전 보간은 동일하다.

15초 전체 구간도 제외 없이 집계했다.

| 지표 | 기준 RMS | 후보 RMS | RMS 차이 | 기준 최대 (초) | 후보 최대 (초) |
|---|---:|---:|---:|---:|---:|
| 골반 위치 mm | 16.084736 | 16.090382 | +0.005646 | 52.283400 (4.100) | 52.283400 (4.100) |
| 골반 회전 rad | 0.008867 | 0.008867 | +0.000000 | 0.070038 (2.267) | 0.070038 (2.267) |
| 양발 위치 mm | 16.208248 | 16.098669 | -0.109579 | 72.086600 (2.383) | 72.086600 (2.383) |
| 양발 회전 rad | 0.037778 | 0.037778 | +0.000000 | 0.112894 (3.183) | 0.112894 (3.183) |
| 팔다리 위치 mm | 19.478727 | 19.516960 | +0.038233 | 115.897000 (2.350) | 115.897000 (2.350) |
| 팔다리 회전 rad | 0.033811 | 0.033811 | +0.000000 | 0.263273 (2.333) | 0.263273 (2.333) |

모든 시나리오의 RMS·최대값·발생 시각·본 이름과 복귀별 결과는 [metrics.json](../.tools/foot-root-alignment-20261107/metrics.json)에 있다. [기준 본 CSV](../.tools/foot-root-alignment-20261107/baseline-case-1.txt.csv), [후보 본 CSV](../.tools/foot-root-alignment-20261107/candidate-case-1.txt.csv), [기준 발·루트 CSV](../.tools/foot-root-alignment-20261107/baseline-case-1.txt.feet.csv), [후보 발·루트 CSV](../.tools/foot-root-alignment-20261107/candidate-case-1.txt.feet.csv)를 함께 보관했다. CSV의 유효 숫자 제한 때문에 수십 μm 수준의 수평 이동은 근사값으로 읽어야 한다.

## 화면 확인과 남은 문제

- 실제 ModelViewer GPU 장면과 UI 버퍼를 저장한 뒤 PNG로 디코딩해 확인했다. 일반 충격 후 후보는 서 있는 Idle로 복귀했다. [기준 화면](../.tools/foot-root-alignment-20261107/baseline-case-1-scene.png), [후보 화면](../.tools/foot-root-alignment-20261107/candidate-case-1-scene.png), [후보 UI](../.tools/foot-root-alignment-20261107/candidate-case-1-ui.png).
- 강한 충격 후에는 기준·후보 모두 쓰러진다. [기준 화면](../.tools/foot-root-alignment-20261107/baseline-case-3-scene.png), [후보 화면](../.tools/foot-root-alignment-20261107/candidate-case-3-scene.png). 이 변경으로 강한 충격의 균형 회복을 해결했다고 판단하지 않는다.
- **양발의 중점을 정렬하므로 각 발을 월드에 고정하지는 않는다.** 일반 충격 복귀에서 후보 왼발은 약 `(+25.39,+2.17,-23.81)mm`, 오른발은 `(-25.42,-2.16,+23.79)mm` 이동했다. 실제 스탠스가 Idle과 다르면 서로 반대 방향으로 움직일 수 있다. 왼발의 수평 이동은 기준 약 22.61mm에서 후보 약 34.81mm로 증가하고, 오른발은 약 47.53mm에서 약 34.82mm로 감소했다.
- 실제 물리 발 높이를 그대로 기준으로 삼기 때문에 물리 상태의 바닥 침투도 이어진다. 일반 충격의 복귀 직전 발바닥 Y는 약 `-5.61/-3.06mm`, 보간 종료 후 후보는 `-3.44/-5.22mm`다. 기존 종료 높이는 약 `+1.78/0.001mm`였다. 지면으로 투영하거나 침투를 별도로 보정하지 않았다.
- 마지막 1초의 후보 발바닥 Y 최대-최소는 왼발 약 0.09181mm, 오른발 약 0.00182mm로 기준과 거의 같았다. 정지 화면과 스텝 CSV를 확인했으며 연속 동영상으로 발 떨림·미끄러짐을 시각 판정하지는 않았다.
- 골반은 보간 동안 약 12.03mm 상승한다. 루트 높이 불일치는 줄었지만, 다리 굽힘과 자세 차이까지 고정하는 처리는 포함하지 않았다.
- 경사·단차·움직이는 지면·모션 매칭·이동 중 캐릭터로의 복귀는 이번에 검증하지 않았다. 양발 접지가 없는 A영역 대기는 단위 검사로 확인했으며, 실제 캐릭터에서 접촉을 강제로 제거한 별도 시험은 하지 않았다.

## 빌드·검사와 검증 범위

| 검사 | 결과 | 원본 |
|---|---|---|
| 기준/후보 ModelViewer Debug | 성공 | [기준 빌드](../.tools/foot-root-alignment-20261107/baseline-build.log), [후보 빌드](../.tools/foot-root-alignment-20261107/candidate-build.log) |
| ActorComponentTests Debug 빌드 | 성공 | [빌드](../.tools/foot-root-alignment-20261107/actor-build.log) |
| 복귀 관련 단위 검사 | 291개 통과. 무접지 대기·접지 후 복귀·중복 스텝·A 이탈 추가 검사 포함 | [로그](../.tools/foot-root-alignment-20261107/recovery-tests.log) |
| PhysicsWorldTests 빌드·실행 | 성공. 기존 접촉·시간·발딛기 142개·IK 273개·서기 지지 검사 포함 | [빌드](../.tools/foot-root-alignment-20261107/physics-build.log), [로그](../.tools/foot-root-alignment-20261107/physics-tests.log) |
| 실제 캐릭터 전환 시나리오 0~5 | 각 검사 통과, 강한 충격의 자세 붕괴는 위에 별도 기록 | [후보 로그 모음](../.tools/foot-root-alignment-20261107/) |
| `git diff --check` | 통과 | 패치 확인 |

후보 ModelViewer 빌드에는 수정하지 않은 `ActiveRagdollValidation.cpp`의 기존 코드 페이지 경고 C4819 1개가 있다. ActorComponentTests 전체·PA 전환 전체 시험은 이번 범위에서 재실행하지 않았다.

토크 제어식·상한·PD·PA 설정은 변경하지 않았다. 20kg 강한 충격은 같은 입력으로 검증했으나 **관절별 요구 토크·적용 토크·포화 지속 시간은 이 계측에 없어 확인하지 못했다.** 현재 기준 코드의 articulation feed-forward 토크 제한과 PD 여유 토크 계산은 이미 주석 처리되어 있으므로, PA 상한이 실제로 지켜진다거나 포화 상태의 제동·복구가 검증됐다고 보고하지 않는다. 강한 충격에서 복귀하지 못한 결과도 그대로 남겼다.

요청한 양발 접지 조건과 양발 중점 기반 루트 정렬은 검증한 조건에서 동작한다. 개별 발 고정·골반 상승 완전 제거·강한 충격 복구까지 해결된 후보는 아니다. 이 결과와 위 잔여 문제를 보고한 뒤 사용자가 해당 변경의 커밋을 요청했다. [AGENTS.md](../AGENTS.md)의 검토 승인 절차를 거쳤으며, 승인 후 코드·애셋·물리 설정은 추가로 변경하지 않았다. 원본 로그·CSV·실행 파일은 Git 추적에서 제외된 로컬 `.tools/foot-root-alignment-20261107/`에 보관한다.

## 재현

작업 디렉터리는 `MiniEngine/ModelViewer`. 후보 실행 파일은 `MiniEngine/Build/x64/Debug/Output/ModelViewer/ModelViewer.exe`. 기준과 후보 검증 실행 파일은 증거 폴더에 따로 보관했다.

```text
ModelViewer.exe -physics 1 -bodyimpacttest <절대경로/report.txt> -kinematiccase 1 -validatefootalignment 1 -impactcapture <절대경로/capture-prefix>
```

`kinematiccase` 0~5가 위 표의 시나리오다. 기준 실행에는 `validatefootalignment`를 생략한다. 이 옵션은 후보의 새 불변 조건을 검증하며 물리 입력을 바꾸지 않는다. `run-validation.ps1 -Variant baseline`과 `-Variant candidate`는 같은 6개 시험을 숨김 창으로 실행한다. CSV는 로그와 같은 이름에 `.csv`, `.feet.csv`를 덧붙인다. `analyze.py`는 측정값을 집계하고 `write_review.py`는 이 보고서·해시 목록·비교 그래프를 만든다.

빌드: MSBuild의 `ModelViewer.sln /p:Configuration=Debug /p:Platform=Windows`, `ActorComponentTests.vcxproj` 및 `PhysicsWorldTests.vcxproj /p:Configuration=Debug /p:Platform=x64`. 단위 검사: `.tools/actor-component-tests/Debug/ActorComponentTests.exe --recovery-only`, `.tools/physicsworld-tests/Debug/PhysicsWorldTests.exe`.
