# 다른 PC에서 PVD 설치하기 — 사용자 / AI 실행 매뉴얼

이 문서는 다른 Windows PC에서 PARS 저장소를 받은 뒤 PhysX Visual Debugger
(PVD)를 설치하고 실제 물리 장면 연결까지 확인하는 절차다.
AI에게 **“`docs/PVD_SETUP.md`를 읽고 이 PC에 PVD를 설치하고 연결을 확인해 줘”**라고
요청하면 된다. 모든 명령은 저장소 루트에서 실행한다.

## 준비 조건과 설치 범위

- Windows x64, Windows PowerShell 5.1 이상.
- NVIDIA 계정 및 다운로드를 위한 인터넷 연결. 로그인, MFA, 약관 확인은 사용자가
  자신의 브라우저에서 진행한다. AI나 스크립트에 비밀번호·쿠키·토큰을 전달할 필요가 없다.
- ModelViewer 실행에는 [루트 README의 개발 도구](../README.md#build-and-run)가 필요하다.
  PVD 설치 자체에는 Visual Studio 빌드가 필요하지 않다.
- 설치 스크립트: [`Tools/Setup-PVD.ps1`](../Tools/Setup-PVD.ps1).
- 설치 위치: `<저장소>/.tools/pvd/app/PVD3.exe`.
  ModelViewer의 **Launch / connect PVD** 버튼이 자동으로 찾는 경로다.
- 설치 프로그램은 Windows 등록 정보를 기록하며, 환경에 따라 UAC 승인이 필요하다.
  UAC 창이 나오면 사용자가 직접 승인한다. 자동 재부팅은 하지 않는다.
- 스크립트와 이 문서는 Git에 포함한다. `.tools/` 전체와 다운로드한 설치 파일은
  Git에 넣지 않는다. PVD는 각 PC에서 한 번씩 설치한다.

## 1. 설치 시도

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tools\Setup-PVD.ps1 -OpenDownloadPage
$LASTEXITCODE
```

스크립트는 설치된 실행 파일의 SHA256, NVIDIA 서명과 필수 DLL을 확인한다.
정상 설치가 있으면 그대로 사용하고 종료한다. 없으면 Windows 다운로드 폴더와
`.tools/pvd/downloads/`에서 `pvd_setup*.exe`를 찾는다.
다운로드 폴더를 다른 드라이브로 옮긴 경우도 Windows 폴더 설정을 따른다.

| 종료 코드 / 출력 | 의미 | AI의 다음 작업 |
| --- | --- | --- |
| `0` / `PVD_READY` | 설치 또는 기존 설치 확인 성공 | 3단계로 이동 |
| `2` / `PVD_DOWNLOAD_REQUIRED` | 지원하는 설치 파일이 아직 없음 | 2단계에서 사용자 로그인·다운로드를 기다림 |
| `1` / 오류 | 경로, 검증, 설치 실패 | 오류와 `.tools/pvd/install.log`를 확인하고 아래 문제 해결 절차 적용 |

`-OpenDownloadPage`는 다운로드가 필요한 경우에만 브라우저를 연다.
브라우저를 열 수 없는 AI 실행 환경에서는 이 옵션을 빼고 실행한 뒤 출력된 링크를
사용자에게 전달한다. 이 옵션을 빼도 설치 파일 탐색과 자동 설치는 동일하게 동작한다.

## 2. NVIDIA 로그인 후 설치 파일 받기

공식 다운로드:
[PhysX Visual Debugger 3.2021.01.29532135](https://developer.nvidia.com/downloads/physx-visual-debugger-3.2021.01.29532135).
링크가 변경되었다면 [NVIDIA PVD 제품 페이지](https://developer.nvidia.com/physx-visual-debugger)의
Download 또는 [NVIDIA 도구 다운로드 목록](https://developer.nvidia.com/tools-downloads)에서
**PhysX Visual Debugger**를 찾는다.

AI는 사용자에게 다음과 같이 안내한다:

> NVIDIA 페이지에서 직접 로그인하고 필요한 약관을 확인해 주세요.
> `pvd_setup.exe` 다운로드가 끝나면 알려 주세요. 다운로드 이후 설치와 확인은 제가 진행할게요.

사용자 로그인 완료 후 브라우저 조작 도구를 쓸 수 있다면 AI가 다운로드를 이어서
진행할 수 있다. 사용할 수 없으면 사용자가 Windows 다운로드 폴더에 파일을 저장한다.
로그인만 완료되었다는 응답을 설치 파일 다운로드 완료로 간주하지 않는다.
파일을 실제로 확인하고, `.crdownload` 등 임시 파일은 실행하지 않는다.

다운로드가 끝나면 같은 스크립트를 다시 실행한다:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tools\Setup-PVD.ps1
$LASTEXITCODE
```

다른 경로에 저장했거나 파일 이름을 바꿨다면 실제 절대 경로를 지정한다:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tools\Setup-PVD.ps1 -InstallerPath "D:\Downloads\pvd_setup.exe"
$LASTEXITCODE
```

스크립트가 SHA256과 NVIDIA Corporation의 유효한 Authenticode 서명을 검증한 후
공식 설치 프로그램을 무인 실행한다. Windows UAC 외에 설치 마법사를 조작할 필요는 없다.
완료 후 실행 파일과 필수 DLL을 확인하고 `.tools/pvd/setup-info.json`에 설치 정보를 기록한다.
설치 파일과 검증이 정상이고 설치 요청을 받은 상태라면 AI는 추가 확인 없이 계속 진행한다.

이 프로젝트에서 검증한 설치 파일:

| 항목 | 값 |
| --- | --- |
| NVIDIA 배포 버전 | `3.2021.01.29532135` |
| 원래 파일 이름 | `pvd_setup.exe` |
| 파일 크기 | `56,184,792` bytes |
| 설치 파일 SHA256 | `FC6B9EC5B3196864A2E03818648C2C1AD1950F943D69958037BEE2299064DEB5` |
| 설치 후 PVD3.exe SHA256 | `2371E7123C9159677C13CC8B070047A763BDB01F96A0F09E313CAA5177A679BB` |

이 배포본은 설치 프로그램 정보에 예전 버전 `3.2016.04.20594560`이 표시되고
PVD3.exe 파일 버전은 `0.0.0.0`으로 표시될 수 있어 SHA256으로 구분한다.
공식 배포 파일이 바뀌면 자동 설치는 중단된다. AI는 검증을 우회하지 말고 공식 출처,
서명과 새 배포 내용을 확인한 뒤 스크립트의 해시와 이 문서를 함께 갱신한다.

## 3. ModelViewer에서 연결 확인

1. `MiniEngine/ModelViewer/ModelViewer.sln` 또는 지원되는 VS의 `.slnx`를 연다.
2. **Debug / Windows** 또는 **Profile / Windows**로 빌드하고 실행한다.
   **Release**는 PVD 계측이 비활성화되어 있으므로 이 검증에 사용하지 않는다.
3. **PARS - Physics** 패널에서 **Launch / connect PVD**를 누른다.
   PVD가 자동으로 실행되고 `127.0.0.1:5425`로 연결을 재시도한다.
4. `PVD: Connected`를 확인하고 **Start drop**으로 상자를 낙하시킨다.
   처음에는 상자만 고정되어 있고 캐릭터 물리는 진행된다.
   **Pause world**를 켠 경우에는 **Step 1/60 s**로 한 프레임씩 기록할 수 있다.
5. PVD에서 `Ground`, `Falling box`와 프레임 증가를 확인한다.
   상자가 낙하한 뒤 바닥에 멈추는지 확인한다.

PVD를 직접 실행하려면 PowerShell에서 아래 명령을 사용한다.
대화형 디버거이므로 창을 표시한다:

```powershell
$pvdApp = (Resolve-Path -LiteralPath '.tools\pvd\app').Path
Start-Process -FilePath (Join-Path $pvdApp 'PVD3.exe') -WorkingDirectory $pvdApp
```

PVD를 재시작했다면 **Reconnect PVD**를 누른다.
게임 실행 후에 PVD를 켜고 연결해도 현재 장면을 볼 수 있으며, 연결 시점부터 기록된다.
기존 PVD 창이 응답하지 않으면 **Restart unresponsive PVD**가 표시된다.
이 버튼은 프로젝트에 설치된 PVD만 종료하고 다시 실행하며, 미저장 기록은 사라진다.
다른 경로의 PVD는 직접 종료한 뒤 다시 연결한다.
설치 스크립트는 설치와 파일 검증까지만 수행한다. `PVD_READY`만으로
실제 디버거 연결과 물리 프레임 기록까지 확인했다고 보고하지 않는다.

## 4. AI용 추가 연결 검사 (선택)

GUI 조작이 어렵거나 연결을 코드로 확인해야 할 때 사용한다.
PVD를 먼저 실행하고 ModelViewer는 종료해서 테스트 연결과 겹치지 않게 한다.
일반 PowerShell에서도 아래 명령으로 VS2022 MSBuild를 찾을 수 있다:

```powershell
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vsPath = & $vswhere -latest -products '*' -version '[17.0,18.0)' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { throw 'Visual Studio 2022 C++ tools were not found.' }
$msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe'
& $msbuild .\MiniEngine\Tests\PhysicsWorldTests.vcxproj /nologo /m /nr:false /p:Configuration=Debug /p:Platform=x64 /verbosity:minimal
if ($LASTEXITCODE -ne 0) { throw 'PhysicsWorldTests build failed.' }
& .\.tools\physicsworld-tests\Debug\PhysicsWorldTests.exe --require-pvd
if ($LASTEXITCODE -ne 0) { throw 'PhysicsWorldTests PVD connection test failed.' }
```

테스트는 물리 계산과 실제 PVD 연결, 일시정지 중 재연결,
캐릭터 충돌 검사 이후 반복 재연결과 물리 재개를 확인한다.
GUI에서 장면을 직접 확인하지 못했다면 최종 보고에 그 점을 구분해서 적는다.

## 문제 해결과 완료 보고

| 증상 | 조치 |
| --- | --- |
| 종료 코드 `2`가 계속 나옴 | 다운로드 완료 여부와 저장 위치를 확인하고 `-InstallerPath`로 지정한다. |
| SHA256 불일치 | 다운로드가 완전히 끝났는지, 위 배포본이 맞는지 확인한다. 검증을 끄지 않는다. |
| 서명 검증 실패 | PC 날짜와 Windows 인증서 신뢰 상태, 네트워크 및 공식 다운로드 출처를 확인한다. |
| UAC 취소 / 접근 거부 | 사용자가 UAC를 승인할 수 있는 환경에서 다시 실행한다. 필요한 경우 사용자 관리자 PowerShell에서 실행한다. |
| PVD가 실행 중이라 설치 중단 | 사용자가 쓰는 디버거 세션을 저장·종료한 뒤 다시 실행한다. AI가 프로세스를 강제 종료하지 않는다. |
| 설치 프로그램 실패 / 파일 누락 | `.tools/pvd/install.log`를 확인한다. 저장소 경로의 쓰기 권한과 보안 프로그램의 차단 여부를 확인한 뒤 재실행한다. |
| `Disconnected` 또는 연결 시간 초과 | Debug/Profile인지, PVD가 실행 중인지, PVD 연결 포트가 `5425`인지 확인하고 Reconnect를 누른다. |
| Connected인데 장면이 비어 있음 | Start drop 또는 Step으로 시뮬레이션 프레임을 진행한다. |
| 다른 위치의 PVD만 설치되어 있음 | 그 PVD를 직접 실행한 뒤 연결하거나, 이 스크립트로 프로젝트 경로에 설치한다. |

AI는 최종 보고에 설치 경로, 설치 스크립트 종료 코드, 실제 수행한 연결 검증 결과를
간단히 적는다. 로그인·다운로드·UAC 또는 실행 환경 때문에 남은 단계가 있다면
해당 단계만 명시한다. 다운로드한 EXE, DLL, 로그, 인증 정보를 커밋하지 않는다.
