// Copyright (c) 2026 Chanpago. Licensed under the MIT License.
#include "pch.h"
#include "ImGuiOverlay.h"
#include "BufferManager.h"
#include "CommandContext.h"
#include "CommandListManager.h"
#include "GraphicsCore.h"
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include <stdexcept>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

namespace
{
    constexpr UINT kFramesInFlight = 3;
    constexpr UINT kDescriptorCount = 256;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> s_DescriptorHeap;
    std::vector<UINT> s_FreeDescriptors;
    UINT s_DescriptorSize = 0;
    UINT s_FrameSlot = 0;
    uint64_t s_FrameFences[kFramesInFlight] = {};
    bool s_Initialized = false;

    void AllocateDescriptor(ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE* cpu,
        D3D12_GPU_DESCRIPTOR_HANDLE* gpu)
    {
        if (s_FreeDescriptors.empty())
            throw std::runtime_error("ImGui SRV descriptor heap exhausted");
        const UINT index = s_FreeDescriptors.back();
        s_FreeDescriptors.pop_back();
        *cpu = s_DescriptorHeap->GetCPUDescriptorHandleForHeapStart();
        *gpu = s_DescriptorHeap->GetGPUDescriptorHandleForHeapStart();
        cpu->ptr += SIZE_T(index) * s_DescriptorSize;
        gpu->ptr += UINT64(index) * s_DescriptorSize;
    }

    void FreeDescriptor(ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE cpu,
        D3D12_GPU_DESCRIPTOR_HANDLE)
    {
        const SIZE_T offset = cpu.ptr - s_DescriptorHeap->GetCPUDescriptorHandleForHeapStart().ptr;
        s_FreeDescriptors.push_back(static_cast<UINT>(offset / s_DescriptorSize));
    }
}

void ImGuiOverlay::Initialize(HWND window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    io.IniFilename = nullptr; // This temporary preview should not persist its layout.
    ImFontConfig fontConfig;
    fontConfig.SizePixels = 18.0f;
    io.Fonts->AddFontDefault(&fontConfig);
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 6.0f;

    D3D12_DESCRIPTOR_HEAP_DESC desc = {};
    desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    desc.NumDescriptors = kDescriptorCount;
    desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ASSERT_SUCCEEDED(Graphics::g_Device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&s_DescriptorHeap)));
    s_DescriptorSize = Graphics::g_Device->GetDescriptorHandleIncrementSize(desc.Type);
    s_FreeDescriptors.clear();
    for (UINT i = kDescriptorCount; i > 0; --i)
        s_FreeDescriptors.push_back(i - 1);
    s_FrameSlot = 0;
    ZeroMemory(s_FrameFences, sizeof(s_FrameFences));

    ImGui_ImplDX12_InitInfo info = {};
    info.Device = Graphics::g_Device;
    info.CommandQueue = Graphics::g_CommandManager.GetGraphicsQueue().GetCommandQueue();
    info.NumFramesInFlight = kFramesInFlight;
    info.RTVFormat = Graphics::g_OverlayBuffer.GetFormat();
    info.SrvDescriptorHeap = s_DescriptorHeap.Get();
    info.SrvDescriptorAllocFn = AllocateDescriptor;
    info.SrvDescriptorFreeFn = FreeDescriptor;
    ASSERT(ImGui_ImplWin32_Init(window));
    ASSERT(ImGui_ImplDX12_Init(&info));
    s_Initialized = true;
}

void ImGuiOverlay::Shutdown()
{
    if (!s_Initialized)
        return;
    Graphics::g_CommandManager.IdleGPU();
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    s_DescriptorHeap.Reset();
    s_FreeDescriptors.clear();
    s_Initialized = false;
}

void ImGuiOverlay::NewFrame()
{
    if (!s_Initialized)
        return;
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

void ImGuiOverlay::Render()
{
    if (!s_Initialized)
        return;
    ImGui::Render();
    ImDrawData* drawData = ImGui::GetDrawData();
    if (drawData->DisplaySize.x <= 0.0f || drawData->DisplaySize.y <= 0.0f)
        return;

    // Protect the backend's upload buffers even when VSync is disabled.
    if (s_FrameFences[s_FrameSlot] != 0)
        Graphics::g_CommandManager.WaitForFence(s_FrameFences[s_FrameSlot]);

    // A separate command list avoids invalidating MiniEngine's cached PSO/root state.
    GraphicsContext& context = GraphicsContext::Begin(L"ImGui Overlay");
    context.TransitionResource(Graphics::g_OverlayBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET, true);
    context.SetRenderTarget(Graphics::g_OverlayBuffer.GetRTV());
    context.SetDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, s_DescriptorHeap.Get());
    ImGui_ImplDX12_RenderDrawData(drawData, context.GetCommandList());
    s_FrameFences[s_FrameSlot] = context.Finish();
    s_FrameSlot = (s_FrameSlot + 1) % kFramesInFlight;
}

bool ImGuiOverlay::HandleMessage(HWND window, UINT message, WPARAM wParam, LPARAM lParam)
{
    return s_Initialized && ImGui_ImplWin32_WndProcHandler(window, message, wParam, lParam) != 0;
}

bool ImGuiOverlay::WantsMouse()
{
    return s_Initialized && ImGui::GetIO().WantCaptureMouse;
}

bool ImGuiOverlay::WantsKeyboard()
{
    return s_Initialized && ImGui::GetIO().WantCaptureKeyboard;
}
