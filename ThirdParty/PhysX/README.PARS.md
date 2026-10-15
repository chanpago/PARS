# PhysX integration

PARS vendors the CPU SDK from NVIDIA PhysX **5.6.1** for Windows x64 / VS 2022.
The source is included in Git; PhysX builds do not download dependencies.

- Upstream: https://github.com/NVIDIA-Omniverse/PhysX
- Tag: `107.3-physx-5.6.1`
- Commit: `5ca9f472105a90d70d957c243cb0ef36fe251a9f`
- License: [BSD 3-Clause](LICENSE.md); preserve this notice when distributing binaries.
- Vendored paths: `physx/include`, `physx/source`, `physx/compiler/public`,
  `physx/buildtools/templates`, `physx/pvdruntime/include`, `physx/version.txt`,
  and the upstream root `LICENSE.md`.

## Build

Install **C++ CMake tools for Windows** in Visual Studio Installer (CMake 3.25+).
`Build.ps1` prefers the CMake shipped with the selected Visual Studio installation
and falls back to `cmake.exe` on PATH.

Build ModelViewer normally from either `.sln` or `.slnx`. Its MSBuild target calls
`Build.ps1` as a custom build step before compilation. Its declared source inputs
and library outputs let Visual Studio detect SDK edits and missing libraries.
CMake/MSBuild incrementally builds the SDK using
the same VS installation, v143 toolset version, and Windows SDK as ModelViewer.
The first build of each configuration takes longer while compiling PhysX.

| MiniEngine configuration | PhysX configuration | Runtime |
| --- | --- | --- |
| Debug | debug | `/MDd` |
| Profile | profile | `/MD` |
| Release | release | `/MD` |

Generated projects, object files, libraries, and test executables are under
`.tools/physx-build/vs2022-x64/` and excluded from Git. Upstream CMake also
generates `physx/include/PxConfig.h`, which is ignored. CMake projects and object
files are cached independently of normal solution outputs.
If changing VS installations/toolset versions, remove that build directory before
building again, as CMake binds a build tree to a generator and toolset.

The linked static libraries are PhysX, Common, Foundation, Cooking, Extensions,
CharacterKinematic, and PvdSDK. `PhysX.props` supplies their paths and
`PX_PHYSX_STATIC_LIB` to Core and ModelViewer. Engine code can include
`<PxPhysicsAPI.h>`. Only ModelViewer imports `PhysX.targets` to build the libraries;
Core itself only needs the vendored headers.

The SDK uses CPU physics. CUDA, GPU physics, samples, vehicle libraries, and the
OmniPVD runtime are not built. No PhysX DLL copy is required. The existing ARM64
solution does not import the x64 SDK settings; PhysX integration currently targets
x64 only.

## Verify

From the repository root, run (substitute `Profile` or `Release` as needed):

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ThirdParty\PhysX\Build.ps1 -Configuration Debug -RunSmokeTest
```

The headless smoke test checks SDK/PVD initialization, a dynamic box falling onto
a floor at a fixed 1/60 second step, triangle mesh cooking, and a capsule controller
moving along the floor. It fails on SDK errors/warnings or incorrect final positions.
It is separate from ModelViewer. The engine now has `MiniEngine/Core/PhysicsWorld`
and a visible floor/box test scene, plus `MiniEngine/Tests/PhysicsWorldTests.vcxproj`
for the production wrapper. See the root README for usage. Imported level
colliders, character input and animation integration remain subsequent work.

## PhysX Visual Debugger (PVD)

The standalone PVD application is installed separately from the `PhysXPvdSDK`
library. Use the [PVD setup guide for users and AI agents](../../docs/PVD_SETUP.md)
and `Tools/Setup-PVD.ps1` to install it on each PC. The user completes NVIDIA login
and downloads the installer in their browser; the script validates its SHA256 and
NVIDIA signature and installs it under `.tools/pvd/app`. Installers and binaries
are excluded from Git. A valid existing installation is reused.
Launch `PVD3.exe` to open the debugger. Its default connection port is TCP 5425.
The x64 ModelViewer physics scene connects automatically to `127.0.0.1:5425`
in Debug and Profile. **Launch / connect PVD** in the **PARS - Physics** panel
also starts the local debugger if needed and retries during startup. Connection
failures are displayed in the panel. Use **Reconnect PVD** after a debugger restart
and **Start drop** or **Step** to record simulation frames. Release SDK builds
disable PVD instrumentation.

## Upstream changes

Two local patches are applied:

- `physx/source/compiler/cmake/windows/CMakeLists.txt`: the public-release external
  DLL copy is conditioned on `PX_BUILDSNIPPETS`, unless explicitly requested through
  `PX_COPY_EXTERNAL_DLL`. This avoids requiring freeglut DLLs for a CPU SDK build
  without sample applications.
- `NpPvdSceneClient.cpp` and `NpPvdSceneQueryCollector.h`: reset recorded queries
  and both geometry buffers when a new PVD session connects. Otherwise reconnecting
  after capsule movement leaves previous-session geometry references that trigger
  `isInstanceValid(data)` in `removeObjectRef` on the next simulation step.
  `PhysicsWorldTests --require-pvd` covers repeated reconnects after controller queries.

To update the SDK, copy the paths listed above from the selected upstream commit,
reapply these patches, update the version/commit here, and run the smoke test plus
ModelViewer builds for Debug, Profile, and Release.
