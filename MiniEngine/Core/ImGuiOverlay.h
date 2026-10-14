// Copyright (c) 2026 Chanpago. Licensed under the MIT License.
#pragma once

#include <Windows.h>

namespace ImGuiOverlay
{
    void Initialize(HWND window);
    void Shutdown();
    void NewFrame();
    void Render();
    bool HandleMessage(HWND window, UINT message, WPARAM wParam, LPARAM lParam);
    bool WantsMouse();
    bool WantsKeyboard();
}
