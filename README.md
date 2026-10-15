# PARS

**Procedural Active Ragdoll System**

A C++ / DirectX 12 project built on Microsoft's MiniEngine.

This repository starts from the working engine snapshot with the corrected
ModelViewer solution configuration and automatic dependency setup. It has an
independent Git history beginning with a single initial commit.

## Build and run

1. Install Visual Studio 2022 17.14 or later with **Desktop development with C++**,
   the **MSVC v143** tools, **C++ CMake tools for Windows** (CMake 3.25+), and a
   Windows SDK. Install Git if it is not already available.
2. Clone this repository and open `MiniEngine/ModelViewer/ModelViewer.slnx`.
3. A fresh checkout opens with **Debug / Windows** (x64) and `ModelViewer` as the
   startup project. Press **F5** to build and run.

For Visual Studio 2022 versions that cannot open `.slnx`, open
`MiniEngine/ModelViewer/ModelViewer.sln` instead. This compatibility solution has
the same projects and configurations; keep both solution files in sync when
changing them. The build prerequisites above still apply.

`Core` and `Model` are grouped under the `Engine` solution folder. The main
solution targets Windows x64; for native ARM64 development, open
`MiniEngine/ModelViewer/ModelViewer.ARM64.slnx` with the ARM64 C++ tools installed.

Visual Studio remembers selections for solutions you have already opened. If an
existing checkout still uses an old selection, choose **Debug / Windows** and
set `ModelViewer` as the startup project once; future opens keep that selection.

The first build needs internet access. NuGet restores the SDK/runtime packages,
and the build sets up vcpkg when needed and restores the DirectX libraries.
For Windows x64, it also builds the bundled CPU PhysX SDK automatically; the
first build of each configuration takes longer. See the
[PhysX integration notes](ThirdParty/PhysX/README.PARS.md) for details and the smoke test.
Build outputs, downloaded tools, and dependency caches are excluded from Git.
A DirectX 12 capable GPU and driver are required to run the application.

From a Visual Studio Developer Command Prompt at the repository root:

```bat
msbuild MiniEngine\ModelViewer\ModelViewer.slnx -restore -p:RestorePackagesConfig=true -p:Configuration=Debug -p:Platform=Windows -m
```

See [the MiniEngine guide](MiniEngine/readme.md) for controls, model loading, and
additional build configurations.

On x64, ModelViewer starts with a **PARS - Physics** test scene, paused with a
one-metre box four metres above the floor. Click **Start drop** to watch it fall;
**Drop again** immediately replays the experiment, and **Reset & pause** restores
the starting pose. **Kick up + spin** applies a PhysX impulse and torque so you
can observe flight, rotation and collision from the current pose. Use **Slow
motion (0.25x)**, **Pause**, and **Step 1/60 s** to inspect the motion. Height,
vertical speed and a height history plot make the result visible after it settles.
Physics runs at 60 Hz independently of rendering, with at most eight catch-up
steps per frame after a stall. **F1** hides/reopens the panel.
Use WASD to move and Q/E to move down/up; hold the right mouse button outside the
UI to look around. Movement does not require a mouse button.

Use command-line arguments `-physics 0` for the original Sponza scene, or
`-model <path.gltf>` for the model viewer. These paths retain the ImGui test panel.
The physics demo uses its own metre-scale floor; Sponza and imported model meshes
do not have collision shapes yet. The existing ARM64 viewer does not use PhysX.

To install PVD on another PC, follow the [PVD setup guide for users and AI agents](docs/PVD_SETUP.md).
Run `powershell.exe -NoProfile -ExecutionPolicy Bypass -File Tools\Setup-PVD.ps1 -OpenDownloadPage`;
if a download is needed, sign in to NVIDIA in your browser, download the installer,
then rerun the command. The script validates and installs it under `.tools/pvd/app`.

For live physics debugging, run a **Debug** or **Profile** build and click
**Launch / connect PVD**. The button connects to `127.0.0.1:5425`, or starts the
local `.tools/pvd/app/PVD3.exe` and retries while the debugger opens. A missing
installation, launch failure or connection timeout is shown in the panel.
PVD can also be opened manually before starting ModelViewer for automatic
connection. Use **Reconnect PVD** after restarting the debugger, including while
the demo is paused. **Start drop** or **Step** records simulation frames; inspect
the `Ground` and `Falling box` actors in PVD. **Release** runs physics without
PVD instrumentation.

The production `Core/PhysicsWorld.cpp` also has a GPU-independent regression test.
From a Visual Studio Developer Command Prompt at the repository root:

```bat
msbuild MiniEngine\Tests\PhysicsWorldTests.vcxproj -p:Configuration=Debug -p:Platform=x64 -m
.tools\physicsworld-tests\Debug\PhysicsWorldTests.exe
```

Substitute `Profile` or `Release` to test those builds. With PVD listening, append
`--require-pvd` in Debug/Profile to verify the actual debugger connection as well.
Tests cover frame-rate independence, floor collision, stall limits, fractional
steps, actor replacement, and shutdown/reinitialization.

## Repository layout

- `MiniEngine/Core`: engine foundation, graphics, input, cameras, and utilities.
- `MiniEngine/Model`: model loading and rendering.
- `MiniEngine/ModelViewer`: the application used as the starting point.
- `ThirdParty/ImGui`: Dear ImGui with Win32 and DirectX 12 backends, compiled into
  Core. See [integration notes](ThirdParty/ImGui/README.PARS.md) for version and usage.
- `ThirdParty/PhysX`: PhysX 5.6.1 CPU SDK sources and automatic x64 static-library
  build, including character controllers and collision mesh cooking.
- `Samples`, `Libraries`, `TechniqueDemos`, `Tools`, and `Assets`: the accompanying
  DirectX samples, shared code, and assets retained from the source snapshot.
- `vcpkg` and `vcpkg.json`: dependency integration and automatic setup.

## License and attribution

The project is licensed under the [MIT License](LICENSE).
MiniEngine originates from Microsoft's
[DirectX-Graphics-Samples](https://github.com/microsoft/DirectX-Graphics-Samples).
The original copyright and license notices are preserved. Bundled third-party
components and sample assets retain their respective notices and licenses.
