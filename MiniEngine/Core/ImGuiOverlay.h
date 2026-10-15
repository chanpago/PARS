// Copyright (c) 2026 Chanpago. Licensed under the MIT License.
#pragma once

#include <Windows.h>
#include <d3d12.h>
#include <cstdint>

namespace ImGuiOverlay
{
    void Initialize(HWND window);
    void Shutdown();
    void NewFrame();
    void Render();
    bool HandleMessage(HWND window, UINT message, WPARAM wParam, LPARAM lParam);
    bool WantsMouse();
    bool WantsKeyboard();
    // The texture must remain alive until UnregisterTexture (GPU idle required).
    uint64_t RegisterTexture(D3D12_CPU_DESCRIPTOR_HANDLE source);
    void UnregisterTexture(uint64_t texture);
}
