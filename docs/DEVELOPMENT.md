# 개발 및 실행 가이드

[PARS 프로젝트 소개로 돌아가기](../README.md)

저장소 루트에서 실행하는 명령을 기준으로 작성했습니다. 개발 도구 요구 사항은 [README의 빠른 시작](../README.md#빠른-시작)을 참고하세요.

## 빌드

### Visual Studio 구성

기본 솔루션은 [`MiniEngine/ModelViewer/ModelViewer.slnx`](../MiniEngine/ModelViewer/ModelViewer.slnx)입니다.
**Windows** 솔루션 플랫폼은 **x64** 프로젝트에 대응하며, `ModelViewer`가 시작 프로젝트입니다.
Visual Studio가 이전 선택을 기억하고 있다면 **Debug / Windows**와 시작 프로젝트를 직접 선택하세요.

| 구성 | 용도 | PVD 기록 |
| --- | --- | --- |
| Debug | 디버깅 | 지원 |
| Profile | 계측을 포함한 실행 | 지원 |
| Release | 릴리스 실행 | 미지원 |

솔루션 탐색기에는 `Core`, `Model`, `ModelViewer`가 나란히 표시됩니다.
프로젝트 필터는 소스 디렉터리 구조를 따르며, 공유 ImGui 소스는 `Core/ThirdParty/ImGui` 아래에 표시됩니다.

### 명령줄 빌드

Visual Studio Developer Command Prompt에서 실행합니다.

```bat
msbuild MiniEngine\ModelViewer\ModelViewer.slnx -restore -p:RestorePackagesConfig=true -p:Configuration=Debug -p:Platform=Windows -m
```

다른 구성을 빌드하려면 `Configuration`을 `Profile` 또는 `Release`로 바꿉니다.

### 의존성과 첫 빌드

- NuGet이 SDK·런타임 패키지를 복원합니다.
- 빌드가 필요할 때 vcpkg를 준비하고 DirectX 라이브러리를 복원합니다. 별도의 `vcpkg integrate install`은 필요하지 않습니다.
- Windows x64에서는 포함된 PhysX CPU SDK를 구성별로 자동 빌드합니다. 첫 빌드에는 시간이 더 걸립니다.
- 빌드 결과, 다운로드한 도구와 의존성 캐시는 Git에서 제외됩니다.

세부 설정은 [MiniEngine 안내](../MiniEngine/readme.md)와 [PhysX 통합 문서](../ThirdParty/PhysX/README.PARS.md)를 참고하세요.

### 솔루션 호환성과 ARM64

`.slnx`를 열 수 없는 Visual Studio에서는 [`ModelViewer.sln`](../MiniEngine/ModelViewer/ModelViewer.sln)을 사용합니다.
두 솔루션은 같은 프로젝트와 구성을 가리키므로, 솔루션 구성을 변경할 때 함께 갱신해야 합니다.
파일 형식을 바꾸어도 README의 개발 도구 요구 사항은 동일합니다.

네이티브 ARM64 빌드에는 ARM64 C++ 도구와 [`ModelViewer.ARM64.slnx`](../MiniEngine/ModelViewer/ModelViewer.ARM64.slnx)를 사용합니다.
기존 ARM64 뷰어에는 x64용 PhysX 통합이 적용되지 않습니다.

## 실행 옵션

| 인자 | 실행 장면 |
| --- | --- |
| 없음 | x64 기본 물리 장면과 애니메이션 캐릭터 |
| `-physics 0` | 기존 Sponza 장면 |
| `-model <path.gltf>` | 지정한 glTF 모델 |

Sponza와 모델 뷰어 경로에서도 ImGui 테스트 패널을 사용할 수 있습니다.
물리 데모는 미터 단위의 자체 바닥을 사용하며, Sponza와 불러온 모델 메시에는 충돌 형상이 자동으로 생성되지 않습니다.
모델 배치와 glTF 로딩은 [MiniEngine의 glTF 안내](../MiniEngine/readme.md#gltf-20-support)를 참고하세요.

## 조작과 실험

### 캐릭터와 카메라

기본 키는 [README의 기본 사용법](../README.md#기본-사용법)에 정리되어 있습니다.
캐릭터 조작에서는 **WASD**로 이동하고 **Shift**로 달립니다. **Tab**으로 자유 카메라를 전환하면 **WASD / Q·E**가 카메라 이동에 사용됩니다.
메인 화면의 자유 카메라는 이동할 때 마우스 버튼을 누르지 않아도 되며, UI 밖에서 오른쪽 버튼을 누른 채 움직이면 시점을 회전합니다.

캐릭터 조작 중 **R**로 충격 실험용 구를 발사할 수 있습니다.
오른쪽 버튼으로 조준하면서 왼쪽 클릭 또는 **R**을 누르면 카메라가 바라보는 방향으로 발사합니다.
래그돌 비교 장면이나 자유 카메라에서는 캐릭터 조작용 발사가 비활성화됩니다.

**F**를 누르면 플레이어 월드의 물리와 캐릭터 애니메이션이 **0.1배속**으로 진행하고, 다시 누르면 정상 속도로 돌아옵니다.
카메라와 UI는 정상 속도로 조작할 수 있으며, 패널에 포커스가 있어도 단축키가 동작합니다. 텍스트 입력 중에는 단축키가 동작하지 않으며, 스켈레탈 메시 편집 뷰포트에서는 기존 F 키의 카메라 맞춤 동작을 유지합니다.

**G**를 누르면 **더미 리셋** 버튼과 같은 동작으로 더미를 초기 상태로 되돌리고 발사체를 제거합니다.
텍스트 입력 중이거나 스켈레탈 메시 편집 뷰포트 또는 래그돌 비교 장면을 사용하는 동안에는 더미 리셋 단축키가 동작하지 않습니다.

### 물리 애셋 편집

**창 → 스켈레탈 메시 뷰어**에서 본과 강체를 선택하고, 물리 애셋(PA)의 바디·관절·드라이브 설정을 편집합니다.
**PA 파일 저장 / PA 불러오기**로 파일을 관리하고, **저장 후 기본 PA로 적용**으로 기본 애셋에 반영할 수 있습니다.

뷰어의 별도 뷰포트에서는 **우클릭 + WASD**, **Q/E**로 카메라를 이동하고 **F**로 메시를 화면에 맞춥니다.
바디 선택 시 **Space**로 이동·회전·크기 기즈모를 전환합니다.

x64 UI는 한국어로 표시되며 Windows의 맑은 고딕 또는 굴림을 사용합니다.
본 이름과 파일 경로 등 애셋 식별자는 원래 이름을 유지합니다.

### 래그돌 비교

**창 → 래그돌 비교**를 열거나, **스켈레탈 메시 뷰어 / 캐릭터 애니메이션** 창에서 비교를 시작합니다.
키네마틱 기준 더미와 액티브 래그돌 더미가 같은 애니메이션·블렌딩·경로를 따릅니다.

| 실험 | 동작 |
| --- | --- |
| 시뮬레이션 A | 대기 2초 → 걷기 4초 → 달리기 4초 |
| 시뮬레이션 B | 대기 후 3m 직진·유턴 반복 |
| 단일 애니메이션 | 클립·시간·이동 속도를 선택해 비교 |

기본 A/B는 바닥 접촉 조건에서 실행됩니다. 공중 지지는 별도의 추종 실험 옵션입니다.
바디별 이동 오차와 관절의 트위스트·스윙 진단을 확인하고, **HTML 저장**으로 한글 보고서를 내보낼 수 있습니다.
보고서는 CSV 다운로드와 인쇄/PDF 저장을 지원합니다.
**캐릭터 조작으로 돌아가기 (WASD)**를 누르면 비교 장면에서 복귀합니다.

### 상자 낙하와 단일 스텝

**PARS - 물리** 패널에서 한 변 1m, 중심 높이 4m로 준비된 상자를 실험할 수 있습니다.

| 조작 | 동작 |
| --- | --- |
| 낙하 시작 / 다시 낙하 | 낙하 실험 시작·재실행 |
| 상자 초기화 | 시작 자세로 복원 |
| 위로 밀기 + 회전 | 현재 자세에서 충격량과 토크 적용 |
| 느리게 (0.1배, F) | 시뮬레이션 0.1배속과 정상 속도 전환 |
| 월드 일시정지 | 물리 월드 정지 |
| 1/60초 진행 | 일시정지 중 물리 한 스텝 실행 |
| F1 | 물리 패널 숨기기·다시 표시 |

패널에는 상자 중심 높이, 수직 속도와 높이 이력이 표시됩니다.
물리는 렌더링과 독립적인 **60 Hz** 고정 간격으로 실행하며, 지연된 프레임당 최대 **8회**까지 따라잡기 스텝을 수행합니다.

## PVD 디버깅

PhysX Visual Debugger(PVD)는 PC마다 별도로 설치합니다. 설치 절차와 문제 해결은 [PVD 설치 가이드](PVD_SETUP.md)에 정리되어 있습니다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File Tools\Setup-PVD.ps1 -OpenDownloadPage
```

다운로드가 필요한 경우 브라우저에서 NVIDIA 로그인 후 설치 프로그램을 받고 같은 명령을 다시 실행합니다.
스크립트는 설치 파일을 검증하고 `.tools/pvd/app`에 설치합니다.

1. **Debug** 또는 **Profile**로 ModelViewer를 실행합니다.
2. **PARS - 물리 → PVD 실행 / 연결**을 누릅니다. 로컬 `PVD3.exe`를 실행하고 `127.0.0.1:5425`로 연결을 시도합니다.
3. 패널에서 연결 상태를 확인하고, **낙하 시작** 또는 일시정지 중 **1/60초 진행**으로 물리 프레임을 기록합니다.
4. PVD에서 `Ground`, `Falling box`와 캐릭터의 물리 상태를 확인합니다.

PVD를 먼저 실행한 뒤 ModelViewer를 시작해도 자동으로 연결합니다.
디버거를 재시작했다면 **PVD 다시 연결**을 사용합니다. 설치 누락, 실행 실패와 연결 시간 초과는 패널에 표시됩니다.
**Release**에서는 PVD 계측이 비활성화됩니다.

## 테스트

아래는 테스트 실행 방법입니다. 테스트 결과와 검증 기록은 해당 변경의 검토 문서를 참고하세요.
Visual Studio Developer Command Prompt에서 저장소 루트를 기준으로 실행합니다.

### 물리 월드

GPU에 의존하지 않는 `PhysicsWorld` 회귀 테스트입니다.
프레임률 독립성, 바닥 충돌, 지연·잔여 시간 처리, 액터 교체와 종료·재초기화 등을 검사합니다.
같은 실행 파일에는 발 지지·발 디딤·IK와 성능 계측 관련 검사도 포함됩니다.

```bat
msbuild MiniEngine\Tests\PhysicsWorldTests.vcxproj -p:Configuration=Debug -p:Platform=x64 -m
.tools\physicsworld-tests\Debug\PhysicsWorldTests.exe
```

`Debug`를 `Profile` 또는 `Release`로 바꾸어 해당 구성을 검사할 수 있습니다.
PVD가 실행 중일 때 Debug/Profile 실행 파일에 `--require-pvd`를 추가하면 실제 디버거 연결도 검사합니다.

### 캐릭터와 컴포넌트

액터·컴포넌트, 캐릭터 이동, 물리 애셋, 래그돌 제어·비교 관련 테스트입니다.

```bat
msbuild MiniEngine\Tests\ActorComponentTests.vcxproj -p:Configuration=Debug -p:Platform=x64 -m
.tools\actor-component-tests\Debug\ActorComponentTests.exe
```

이 프로젝트는 **Debug / Release** 구성을 제공합니다.
PhysX SDK 자체의 스모크 테스트는 [PhysX 통합 문서](../ThirdParty/PhysX/README.PARS.md#verify)를 참고하세요.
