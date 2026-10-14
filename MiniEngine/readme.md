# MiniEngine by Team Minigraph at Microsoft

## Getting started:
* Install Visual Studio 2022 17.14 or later with **Desktop development with C++**, the **MSVC v143** tools, and a Windows SDK (tested with 10.0.26100.0).
* Clone this repository, then open `ModelViewer/ModelViewer.slnx`.
* A fresh checkout selects configuration `Debug`, solution platform `Windows` (x64), and startup project `ModelViewer` automatically. `Core` and `Model` appear under the `Engine` solution folder.
* Press **F5** to build and run. NuGet restores the SDK/runtime packages. If vcpkg is not already installed, the build downloads a pinned copy into `.tools/vcpkg` and restores DirectXTK12, DirectXTex, and DirectXMesh automatically.

The first build needs internet access and takes longer while tools and libraries are downloaded and built. No separate vcpkg installation, `vcpkg integrate install`, or `VCPKG_ROOT` setting is required. Git must be on `PATH`, or available through Visual Studio's bundled Git. Downloaded tools and libraries are ignored by Git.

`Profile` (instrumented) and `Release` remain available in the main solution. For native ARM64 development, open `ModelViewer/ModelViewer.ARM64.slnx` with the corresponding C++ build tools installed. A DirectX 12 capable GPU and driver are required to run ModelViewer.

Visual Studio keeps your previously selected configuration and startup project in local solution settings. For a checkout you have already opened, select `Debug` / `Windows` and set `ModelViewer` as the startup project once if needed. The defaults above apply when opening a fresh checkout.

From a Visual Studio Developer Command Prompt, the equivalent build from the repository root is:

```bat
msbuild MiniEngine\ModelViewer\ModelViewer.slnx -restore -p:RestorePackagesConfig=true -p:Configuration=Debug -p:Platform=Windows -m
```

Existing vcpkg installations selected through `VCPKG_ROOT` or bundled with Visual Studio are still supported. The automatic fallback uses the revision in `vcpkg/vcpkg-version.txt`; its bundled MSBuild integration files live in `vcpkg/msbuild` and must be updated together with that revision.

## Controls:
* forward/backward/strafe: left thumbstick or WASD (FPS controls)
* up/down: triggers or E/Q
* yaw/pitch: right thumbstick or mouse
* toggle slow movement: click left thumbstick or lshift
* open debug menu: back button or backspace
* navigate debug menu: dpad or arrow keys
* toggle debug menu item: A button or return
* adjust debug menu value: dpad left/right or left/right arrow keys

## glTF 2.0 Support:

Get sample assets from https://github.com/KhronosGroup/glTF-Sample-Models or make your own.

* Place asset folder underneath the ModelViewer folder
* Add to command line "-model [relative path to gltf or glb file]"
* Example:  ModelViewer.exe -model SciFiHelmet/glTF/SciFiHelmet.gltf

Notes:  Some IBL cube maps are provided for physically based rendering in the ModelViewer/Textures folder.  ModelViewer automatically detects cube maps in this folder so you can add your own.  They must follow the same naming convention for diffuse and specular maps.  You can change the active environment map in the tweak menu under the "ModelViewer" category.

DirectXMesh and DirectXTex are used for compiling content the first time it is loaded.  For example, image files will have mip maps generated, encoded to a block compressed format, and then saved as a DDS for subsequent loading.  This means that first-time loading takes a little longer.
