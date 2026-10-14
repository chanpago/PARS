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
3. Set `ModelViewer` as the startup project, select **Debug** and **Windows** (x64),
   then press **F5**.

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

## Repository layout

- `MiniEngine/Core`: engine foundation, graphics, input, cameras, and utilities.
- `MiniEngine/Model`: model loading and rendering.
- `MiniEngine/ModelViewer`: the application used as the starting point.
- `Samples`, `Libraries`, `TechniqueDemos`, `Tools`, and `Assets`: the accompanying
  DirectX samples, shared code, and assets retained from the source snapshot.
- `vcpkg` and `vcpkg.json`: dependency integration and automatic setup.

## License and attribution

The project is licensed under the [MIT License](LICENSE).
MiniEngine originates from Microsoft's
[DirectX-Graphics-Samples](https://github.com/microsoft/DirectX-Graphics-Samples).
The original copyright and license notices are preserved. Bundled third-party
components and sample assets retain their respective notices and licenses.
