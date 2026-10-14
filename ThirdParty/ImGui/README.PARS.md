# Dear ImGui for PARS

Vendored from [Dear ImGui](https://github.com/ocornut/imgui), tag
[`v1.92.9b-docking`](https://github.com/ocornut/imgui/tree/v1.92.9b-docking),
commit `b48d1afbe8ee8b238e2961dc363a949dd7304e23`.
The upstream files are unmodified and retain their [MIT license](LICENSE.txt).

## Included files

- Dear ImGui core, docking support, and the demo window.
- Win32 platform backend and DirectX 12 renderer backend.
- `misc/cpp/imgui_stdlib.*` for widgets that accept `std::string`.
- The official [Win32/DX12 example](examples/example_win32_directx12/main.cpp)
  and [backend documentation](docs/BACKENDS.md), for reference.

## Build integration

`MiniEngine/Core/Core.vcxproj` compiles the core, demo, both backends, and
`imgui_stdlib.cpp` into `Core.lib`, with precompiled headers disabled for these
upstream sources. `MiniEngine/PropertySheets/Build.props` supplies the include
paths for MiniEngine projects. ModelViewer already links Core, so no separate
download, DLL, or additional ImGui project is needed.

```cpp
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_stdlib.h" // Optional std::string support.
```

## Runtime preview

ModelViewer opts in through `IGameApp::UsesImGui()` and displays a temporary
`PARS - ImGui Test` panel from `ModelViewer::RenderUI()`. It includes a slider,
click counter, text input, FPS display, and a checkbox for the official demo.
Press **F1** to hide or reopen the test panel. Hold the right mouse button outside
the UI to control the camera; use WASD while holding it to move.

`MiniEngine/Core/ImGuiOverlay.*` owns the context, Win32/DX12 backends, and a
shader-visible SRV descriptor heap. GameCore forwards window messages, starts
each ImGui frame before game input, and renders the draw data into the UI overlay
after the existing UI pass. A separate command list keeps backend state changes
isolated from MiniEngine's cached graphics state, and per-frame fences protect
the backend's upload buffers. Shutdown waits for outstanding GPU work.

GameInput uses a shared mouse cursor for opted-in apps and masks keyboard/mouse
input captured by ImGui. Other MiniEngine applications keep their previous mouse
mode unless they override `UsesImGui()`. Preview layout persistence and separate
OS viewport windows are disabled; the upstream files remain unchanged.

The reference example has its own entry point and is deliberately not compiled
into Core or ModelViewer. Do not compile the vendored sources again in ModelViewer.

To update, replace all upstream files together from one tagged revision, keep
`LICENSE.txt`, and update the tag and commit recorded above. Keep core and backend
versions matched.
