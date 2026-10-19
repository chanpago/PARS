#include "PhysicsDemo.h"
#include "GameFramework/World.h"
#include "BufferManager.h"
#include "Camera.h"
#include "CommandContext.h"
#include "GraphicsCommon.h"
#include "imgui.h"
#include "CompiledShaders/PhysicsDemoVS.h"
#include "CompiledShaders/PhysicsDemoPS.h"
#include <cfloat>
#include <Windows.h>
#include <TlHelp32.h>

using namespace physx;
using namespace Graphics;

namespace
{
    const PxVec3 FloorHalfExtents(6.0f, 0.25f, 6.0f);
    const PxVec3 BoxHalfExtents(0.5f);
    const PxVec3 WallCentres[] = {{6, 0.75f, 0}, {-6, 0.75f, 0}, {0, 0.75f, 6}, {0, 0.75f, -6}, {0, 0.75f, -2.5f}};
    const PxVec3 WallExtents[] = {{.2f, .75f, 6}, {.2f, .75f, 6}, {6, .75f, .2f}, {6, .75f, .2f}, {1.5f, .75f, .25f}};

    DWORD FindPvdProcess()
    {
        HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
        if (snapshot == INVALID_HANDLE_VALUE)
        {
            return 0;
        }
        PROCESSENTRY32W entry = {};
        entry.dwSize = sizeof(entry);
        DWORD found = 0;
        if (Process32FirstW(snapshot, &entry))
        {
            do
            {
                if (_wcsicmp(entry.szExeFile, L"PVD3.exe") == 0)
                {
                    found = entry.th32ProcessID;
                    break;
                }
            } while (Process32NextW(snapshot, &entry));
        }
        CloseHandle(snapshot);
        return found;
    }

    HWND FindPvdWindow(DWORD processId)
    {
        struct Search
        {
            DWORD processId;
            HWND window;
        } search{processId, nullptr};
        EnumWindows([](HWND window, LPARAM parameter) -> BOOL {
            auto& result = *reinterpret_cast<Search*>(parameter);
            DWORD owner = 0;
            GetWindowThreadProcessId(window, &owner);
            wchar_t title[256] = {};
            GetWindowTextW(window, title, _countof(title));
            if (owner == result.processId && wcsstr(title, L"PhysX Visual Debugger"))
            {
                result.window = window;
                return FALSE;
            }
            return TRUE;
        },
            reinterpret_cast<LPARAM>(&search));
        return search.window;
    }

    bool IsPvdUnresponsive(HWND window)
    {
        if (!window)
        {
            return false;
        }
        DWORD_PTR response = 0;
        return SendMessageTimeoutW(window, WM_NULL, 0, 0, SMTO_ABORTIFHUNG | SMTO_BLOCK,
                   250, &response) == 0;
    }

    void ShowPvdWindow(HWND window)
    {
        if (!window)
        {
            return;
        }
        // A stale debugger must not block ModelViewer's message/render thread.
        ShowWindowAsync(window, SW_RESTORE);
        SetForegroundWindow(window);
    }

    std::wstring FindPvdExecutable()
    {
        // Locate the repository from the executable, independent of VS's working directory.
        std::wstring directory(32768, L'\0');
        const DWORD length = GetModuleFileNameW(nullptr, &directory[0], static_cast<DWORD>(directory.size()));
        if (!length || length >= directory.size())
        {
            return {};
        }
        directory.resize(length);
        for (size_t slash = directory.find_last_of(L"\\/"); slash != std::wstring::npos;
             slash = directory.find_last_of(L"\\/"))
        {
            directory.resize(slash);
            const std::wstring path = directory + L"\\.tools\\pvd\\app\\PVD3.exe";
            const DWORD attributes = GetFileAttributesW(path.c_str());
            if (attributes != INVALID_FILE_ATTRIBUTES && !(attributes & FILE_ATTRIBUTE_DIRECTORY))
            {
                return path;
            }
        }
        return {};
    }

    struct Vertex
    {
        PxVec3 position;
        PxVec3 normal;
        PxVec3 color;
    };

    void AppendBox(Vertex*& output, const PxTransform& pose, const PxVec3& halfExtents,
        const PxVec3& color)
    {
        static const PxVec3 normals[] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        static const PxVec3 tangents[] = {
            {0, 1, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0}, {1, 0, 0}};
        static const float corners[6][2] = {
            {-1, -1}, {1, -1}, {1, 1}, {-1, -1}, {1, 1}, {-1, 1}};
        for (unsigned face = 0; face < 6; ++face)
        {
            const PxVec3 bitangent = normals[face].cross(tangents[face]);
            for (const auto& corner : corners)
            {
                const PxVec3 local = normals[face] + tangents[face] * corner[0] + bitangent * corner[1];
                output->position = pose.transform(local.multiply(halfExtents));
                output->normal = pose.q.rotate(normals[face]);
                output->color = color;
                ++output;
            }
        }
    }
}

PhysicsDemo::PhysicsDemo(PARS::World& world)
    : m_GameWorld(world), m_World(*world.GetPhysicsWorld())
{
}

bool PhysicsDemo::Startup()
{
    if (!m_World.IsInitialized())
    {
        return false;
    }
    m_Floor = m_World.CreateStaticBox(PxTransform(PxVec3(0, -0.25f, 0)), FloorHalfExtents, "Ground");
    ResetBox();
    for (size_t i = 0; i < _countof(m_Walls); ++i)
    {
        m_Walls[i] = m_World.CreateStaticBox(PxTransform(WallCentres[i]), WallExtents[i], "Wall");
        if (!m_Walls[i])
        {
            Cleanup();
            return false;
        }
    }
    if (!m_Floor || !m_Box)
    {
        Cleanup();
        return false;
    }

    m_RootSignature.Reset(1);
    m_RootSignature[0].InitAsConstantBuffer(0, D3D12_SHADER_VISIBILITY_VERTEX);
    m_RootSignature.Finalize(L"Physics demo", D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    const D3D12_INPUT_ELEMENT_DESC layout[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};
    m_PSO.SetRootSignature(m_RootSignature);
    m_PSO.SetRasterizerState(RasterizerTwoSided);
    m_PSO.SetBlendState(BlendDisable);
    m_PSO.SetDepthStencilState(DepthStateReadWrite);
    m_PSO.SetInputLayout(_countof(layout), layout);
    m_PSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);
    m_PSO.SetRenderTargetFormat(g_SceneColorBuffer.GetFormat(), g_SceneDepthBuffer.GetFormat());
    m_PSO.SetVertexShader(g_pPhysicsDemoVS, sizeof(g_pPhysicsDemoVS));
    m_PSO.SetPixelShader(g_pPhysicsDemoPS, sizeof(g_pPhysicsDemoPS));
    m_PSO.Finalize();
    m_LinePSO = m_PSO;
    m_LinePSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE);
    m_LinePSO.SetDepthStencilState(DepthStateReadOnly);
    m_LinePSO.Finalize();
    return true;
}

void PhysicsDemo::Cleanup()
{
    m_PvdConnecting = false;
    if (m_Box)
    {
        m_Box->release();
    }
    if (m_Floor)
    {
        m_Floor->release();
    }
    for (auto& wall : m_Walls)
    {
        if (wall)
        {
            wall->release();
        }
        wall = nullptr;
    }
    m_Box = nullptr;
    m_Floor = nullptr;
}

void PhysicsDemo::ResetBox()
{
    if (m_Box)
    {
        m_Box->release();
    }
    m_Box = m_World.CreateDynamicBox(PxTransform(PxVec3(0, 4, 0)), BoxHalfExtents, 10.0f, "Falling box");
    // Hold only the demonstration box; character physics remains active.
    if (m_Box)
    {
        m_Box->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    }
    m_DropReady = true;
    m_HistoryCount = 0;
    m_HistoryOffset = 0;
    RecordHeight();
}

void PhysicsDemo::StartDrop()
{
    ResetBox();
    if (m_Box)
    {
        m_Box->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
        // Clearing eKINEMATIC preserves sleep. Explicitly wake the released box
        // so gravity starts acting on the very next world step.
        m_Box->wakeUp();
    }
    m_DropReady = m_Box == nullptr;
    m_GameWorld.SetPhysicsPaused(false);
}

void PhysicsDemo::KickBox()
{
    if (!m_Box)
    {
        return;
    }
    m_Box->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
    m_DropReady = false;
    // Apply a real impulse and torque; PhysX handles the ensuing flight,
    // rotation and collision with the floor from the current pose.
    m_Box->addForce(PxVec3(0, 60, 0), PxForceMode::eIMPULSE);
    m_Box->addTorque(PxVec3(5, 0, 3), PxForceMode::eIMPULSE);
    m_GameWorld.SetPhysicsPaused(false);
}

void PhysicsDemo::RecordHeight()
{
    if (!m_Box)
    {
        return;
    }
    m_HeightHistory[m_HistoryOffset] = m_Box->getGlobalPose().p.y;
    m_HistoryOffset = (m_HistoryOffset + 1) % HistoryCapacity;
    if (m_HistoryCount < HistoryCapacity)
    {
        ++m_HistoryCount;
    }
}

void PhysicsDemo::Update(float)
{
    UpdatePvdConnection(); // Connection retries must also run while physics is paused.
    if (!m_DropReady && m_World.GetLastSubsteps() && m_Box && !m_Box->isSleeping())
    {
        RecordHeight();
    }
}

void PhysicsDemo::ConnectPvd()
{
    m_PvdConnecting = false;
    m_PvdUnresponsive = false;
    m_PvdMessage.clear();
    m_PvdProcessId = FindPvdProcess();
    const HWND window = FindPvdWindow(m_PvdProcessId);
    if (window && IsPvdUnresponsive(window))
    {
        m_PvdUnresponsive = true;
        m_PvdMessage = "The existing PVD window is not responding. Restart it to restore the connection.";
        return;
    }
    ShowPvdWindow(window);
    // Force a fresh connection: the SDK may retain a stale socket status while paused.
    if (m_World.ConnectPvd(true))
    {
        return;
    }

    if (!m_PvdProcessId)
    {
        const std::wstring executable = FindPvdExecutable();
        if (executable.empty())
        {
            m_PvdMessage = "PVD3.exe was not found in .tools/pvd/app. Open your installed PVD, then retry.";
            return;
        }
        const std::wstring workingDirectory = executable.substr(0, executable.find_last_of(L"\\/"));
        STARTUPINFOW startup = {};
        startup.cb = sizeof(startup);
        startup.dwFlags = STARTF_USESHOWWINDOW;
        startup.wShowWindow = SW_SHOWNORMAL; // This is the interactive debugger requested by the button.
        PROCESS_INFORMATION process = {};
        if (!CreateProcessW(executable.c_str(), nullptr, nullptr, nullptr, FALSE, 0, nullptr,
                workingDirectory.c_str(), &startup, &process))
        {
            m_PvdMessage = "Could not start PVD (Windows error " + std::to_string(GetLastError()) + ").";
            return;
        }
        m_PvdProcessId = process.dwProcessId;
        CloseHandle(process.hThread);
        CloseHandle(process.hProcess);
        m_PvdMessage = "Starting PVD and waiting for its connection port...";
    }
    else
    {
        m_PvdMessage = "PVD is running. Waiting for 127.0.0.1:5425...";
    }

    const uint64_t now = GetTickCount64();
    m_PvdConnecting = true;
    m_PvdNextAttempt = now + 500;
    m_PvdConnectDeadline = now + 30000;
}

void PhysicsDemo::RestartPvd()
{
    // Only the explicit recovery button may terminate a hung debugger, and only
    // the bundled executable we know. Never terminate an arbitrary PVD process.
    const HWND window = FindPvdWindow(m_PvdProcessId);
    if (!window || !IsPvdUnresponsive(window))
    {
        ConnectPvd();
        return;
    }
    HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION | PROCESS_TERMINATE | SYNCHRONIZE,
        FALSE, m_PvdProcessId);
    wchar_t executable[32768] = {};
    DWORD size = _countof(executable);
    const std::wstring bundled = FindPvdExecutable();
    const bool local = process && QueryFullProcessImageNameW(process, 0, executable, &size) &&
        !bundled.empty() && _wcsicmp(executable, bundled.c_str()) == 0;
    bool stopped = false;
    if (local && TerminateProcess(process, 1))
    {
        stopped = WaitForSingleObject(process, 2000) == WAIT_OBJECT_0;
    }
    if (process)
    {
        CloseHandle(process);
    }
    if (stopped)
    {
        ConnectPvd();
    }
    else
    {
        m_PvdMessage = "Could not restart the bundled PVD. Close the unresponsive debugger and retry Connect.";
    }
}

void PhysicsDemo::UpdatePvdConnection()
{
    if (!m_PvdConnecting)
    {
        return;
    }
    const uint64_t now = GetTickCount64();
    if (now >= m_PvdConnectDeadline)
    {
        m_PvdConnecting = false;
        const HWND window = FindPvdWindow(m_PvdProcessId);
        m_PvdUnresponsive = window && IsPvdUnresponsive(window);
        m_PvdMessage = m_PvdUnresponsive ? "PVD is not responding. Restart the debugger and retry." : "PVD did not accept the connection on 127.0.0.1:5425. Enable incoming connections "
                                                                                                      "using PVD's rightmost toolbar button, then retry.";
        return;
    }
    // PVD can temporarily stop pumping window messages while starting OpenGL.
    // Keep retrying during startup instead of treating that as a hung debugger.
    if (now < m_PvdNextAttempt)
    {
        return;
    }
    m_PvdNextAttempt = now + 500;
    if (m_World.ConnectPvd())
    {
        m_PvdConnecting = false;
        m_PvdMessage.clear();
    }
}

void PhysicsDemo::Render(GraphicsContext& context, const Math::Camera& camera,
    const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor)
{
    context.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET);
    context.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, true);
    float background[] = {0.025f, 0.04f, 0.065f, 1.0f};
    context.ClearColor(g_SceneColorBuffer, background);
    context.ClearDepth(g_SceneDepthBuffer);
    context.SetRenderTarget(g_SceneColorBuffer.GetRTV(), g_SceneDepthBuffer.GetDSV());
    context.SetViewportAndScissor(viewport, scissor);
    context.SetRootSignature(m_RootSignature);
    context.SetPipelineState(m_PSO);
    context.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    context.SetDynamicConstantBufferView(0, sizeof(Math::Matrix4), &camera.GetViewProjMatrix());

    // Copy authoritative PhysX position AND rotation after fetchResults().
    alignas(16) Vertex vertices[36 * 7];
    Vertex* output = vertices;
    if (m_Floor)
    {
        AppendBox(output, m_Floor->getGlobalPose(), FloorHalfExtents, PxVec3(0.24f, 0.34f, 0.42f));
    }
    if (m_Box)
    {
        AppendBox(output, m_Box->getGlobalPose(), BoxHalfExtents, PxVec3(1.0f, 0.38f, 0.06f));
    }
    for (size_t i = 0; i < _countof(m_Walls); ++i)
    {
        if (m_Walls[i])
        {
            AppendBox(output, m_Walls[i]->getGlobalPose(), WallExtents[i], PxVec3(.35f, .47f, .5f));
        }
    }
    const UINT count = static_cast<UINT>(output - vertices);
    if (count)
    {
        context.SetDynamicVB(0, count, sizeof(Vertex), vertices);
        context.Draw(count);
    }
}

void PhysicsDemo::RenderCapsules(GraphicsContext& context, const Math::Camera& camera,
    const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor)
{
    std::vector<PxDebugLine> lines;
    m_World.GetCapsuleDebugLines(lines);
    if (lines.empty())
    {
        return;
    }
    std::vector<Vertex> vertices;
    vertices.reserve(lines.size() * 2);
    for (const auto& line : lines)
    {
        const PxVec3 color(float((line.color0 >> 16) & 255) / 255, float((line.color0 >> 8) & 255) / 255, float(line.color0 & 255) / 255);
        vertices.push_back({line.pos0, PxVec3(0, 1, 0), color});
        vertices.push_back({line.pos1, PxVec3(0, 1, 0), color});
    }
    context.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET);
    context.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_READ, true);
    context.SetRenderTarget(g_SceneColorBuffer.GetRTV(), g_SceneDepthBuffer.GetDSV_DepthReadOnly());
    context.SetViewportAndScissor(viewport, scissor);
    context.SetRootSignature(m_RootSignature);
    context.SetPipelineState(m_LinePSO);
    context.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
    context.SetDynamicConstantBufferView(0, sizeof(Math::Matrix4), &camera.GetViewProjMatrix());
    context.SetDynamicVB(0, static_cast<UINT>(vertices.size()), sizeof(Vertex), vertices.data());
    context.Draw(static_cast<UINT>(vertices.size()));
}

void PhysicsDemo::RenderUI()
{
    static bool visible = true;
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
        visible = !visible;
    }
    if (!visible)
    {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(20, 70), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(410, 610), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(390, 520), ImVec2(FLT_MAX, FLT_MAX));
    if (ImGui::Begin("PARS - Physics", &visible))
    {
        const bool ready = m_DropReady;
        bool paused = m_GameWorld.IsPhysicsPaused();
        bool slowMotion = m_GameWorld.GetPhysicsTimeScale() < 1.0f;
        ImGui::TextUnformatted(paused ? "WORLD PAUSED" : ready ? "BOX READY - character active"
                                                               : "SIMULATING");
        if (ImGui::Button(ready ? "Start drop###Drop" : "Drop again###Drop", ImVec2(-1, 32)))
        {
            StartDrop();
        }
        if (ImGui::Button("Reset box"))
        {
            ResetBox();
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!m_Box);
        if (ImGui::Button("Kick up + spin"))
        {
            KickBox();
        }
        ImGui::EndDisabled();
        if (ImGui::Checkbox("Slow motion (0.25x)", &slowMotion))
        {
            m_GameWorld.SetPhysicsTimeScale(slowMotion ? 0.25f : 1.0f);
        }
        if (ImGui::Checkbox("Pause world", &paused))
        {
            m_GameWorld.SetPhysicsPaused(paused);
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!paused);
        if (ImGui::Button("Step 1/60 s"))
        {
            m_GameWorld.RequestPhysicsStep();
        }
        ImGui::EndDisabled();
        ImGui::Separator();
        if (m_Box)
        {
            ImGui::Text("Box centre height: %.3f m", m_Box->getGlobalPose().p.y);
            ImGui::Text("Vertical speed: %.3f m/s", m_Box->getLinearVelocity().y);
            ImGui::Text("Body: %s", m_Box->isSleeping() ? "Sleeping" : "Awake");
            ImGui::PlotLines("##Height", m_HeightHistory, m_HistoryCount,
                m_HistoryCount == HistoryCapacity ? m_HistoryOffset : 0,
                "Height history (m)", 0.0f, FLT_MAX, ImVec2(-1, 64));
        }
        else
        {
            ImGui::TextUnformatted("Box creation failed. Try Reset box.");
        }
        ImGui::Text("%.1f FPS | Physics: 60 Hz | Speed: %.2fx", ImGui::GetIO().Framerate,
            m_GameWorld.GetPhysicsTimeScale());
        ImGui::Text("Steps: %llu | This frame: %u", static_cast<unsigned long long>(m_World.GetStepCount()),
            m_World.GetLastSubsteps());
        ImGui::Text("Dropped stall time: %.2f s", m_World.GetDroppedTime());
        ImGui::Separator();
        bool showCapsule = m_World.IsCapsuleDebugVisible();
        if (ImGui::Checkbox("Movement capsule outline", &showCapsule))
        {
            m_World.SetCapsuleDebugVisible(showCapsule);
        }
        if (!PhysicsWorld::SupportsPvd())
        {
            ImGui::TextWrapped("PVD recording requires a Debug or Profile build.");
        }
        else
        {
            const bool connected = m_World.IsPvdConnected();
            if (ImGui::Checkbox("Show scene queries in PVD", &m_PvdSceneQueries))
            {
                m_World.SetPvdSceneQueries(m_PvdSceneQueries);
            }
            ImGui::TextColored(connected ? ImVec4(0.4f, 0.95f, 0.65f, 1) : ImVec4(1, 0.75f, 0.3f, 1),
                "PVD: %s", connected ? "Connected" : "Disconnected");
            ImGui::BeginDisabled(m_PvdConnecting);
            if (ImGui::Button(m_PvdConnecting ? "Connecting..." : connected ? "Reconnect PVD"
                                                                            : "Launch / connect PVD"))
            {
                ConnectPvd();
            }
            ImGui::EndDisabled();
            if (m_PvdUnresponsive && ImGui::Button("Restart unresponsive PVD"))
            {
                RestartPvd();
            }
            if (connected)
            {
                ImGui::TextWrapped("World updates record the character and rigid bodies in PVD.");
            }
            else if (!m_PvdMessage.empty())
            {
                ImGui::TextWrapped("%s", m_PvdMessage.c_str());
            }
            else
            {
                ImGui::TextWrapped("Connect opens the local PVD application automatically.");
            }
        }
        ImGui::Text("PhysX errors: %u", m_World.GetErrorCount());
        ImGui::TextWrapped("Tab: character / free camera. F1: panels. Units: metres.");
    }
    ImGui::End();
}
