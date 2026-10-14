# PARS

**Procedural Active Ragdoll System**

A C++ / DirectX 12 project built on Microsoft's MiniEngine.

This repository starts from the working engine snapshot with the corrected
ModelViewer solution configuration and automatic dependency setup. It has an
independent Git history beginning with a single initial commit.

## Build and run

1. Install Visual Studio 2022 17.14 or later with **Desktop development with C++**,
   the **MSVC v143** tools, and a Windows SDK. Install Git if it is not already
   available.
2. Clone this repository and open `MiniEngine/ModelViewer/ModelViewer.slnx`.
3. A fresh checkout opens with **Debug / Windows** (x64) and `ModelViewer` as the
   startup project. Press **F5** to build and run.

`Core` and `Model` are grouped under the `Engine` solution folder. The main
solution targets Windows x64; for native ARM64 development, open
`MiniEngine/ModelViewer/ModelViewer.ARM64.slnx` with the ARM64 C++ tools installed.

Visual Studio remembers selections for solutions you have already opened. If an
existing checkout still uses an old selection, choose **Debug / Windows** and
set `ModelViewer` as the startup project once; future opens keep that selection.

The first build needs internet access. NuGet restores the SDK/runtime packages,
and the build sets up vcpkg when needed and restores the DirectX libraries.
Build outputs, downloaded tools, and dependency caches are excluded from Git.
A DirectX 12 capable GPU and driver are required to run the application.

From a Visual Studio Developer Command Prompt at the repository root:

```bat
msbuild MiniEngine\ModelViewer\ModelViewer.slnx -restore -p:RestorePackagesConfig=true -p:Configuration=Debug -p:Platform=Windows -m
```

See [the MiniEngine guide](MiniEngine/readme.md) for controls, model loading, and
additional build configurations.

ModelViewer also opens a temporary **PARS - ImGui Test** panel. Try its slider,
button, text input, or **Show ImGui demo** checkbox. **F1** hides/reopens the panel.
Hold the right mouse button outside the UI to look around and use WASD to move.

## Repository layout

- `MiniEngine/Core`: engine foundation, graphics, input, cameras, and utilities.
- `MiniEngine/Model`: model loading and rendering.
- `MiniEngine/ModelViewer`: the application used as the starting point.
- `ThirdParty/ImGui`: Dear ImGui with Win32 and DirectX 12 backends, compiled into
  Core. See [integration notes](ThirdParty/ImGui/README.PARS.md) for version and usage.
- `Samples`, `Libraries`, `TechniqueDemos`, `Tools`, and `Assets`: the accompanying
  DirectX samples, shared code, and assets retained from the source snapshot.
- `vcpkg` and `vcpkg.json`: dependency integration and automatic setup.

## License and attribution

The project is licensed under the [MIT License](LICENSE).
MiniEngine originates from Microsoft's
[DirectX-Graphics-Samples](https://github.com/microsoft/DirectX-Graphics-Samples).
The original copyright and license notices are preserved. Bundled third-party
components and sample assets retain their respective notices and licenses.
