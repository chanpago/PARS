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
#include <cmath>
#include <Windows.h>
#include <TlHelp32.h>
#include <functional>

using namespace physx;
using namespace Graphics;

namespace
{
    const PxVec3 FloorHalfExtents(15.0f, 0.25f, 15.0f);
    // 외곽 벽의 위치와 길이는 바닥 크기에 맞추고 내부 장애물은 기존 배치를 유지한다.
    const PxVec3 WallCentres[] =
    {
        {FloorHalfExtents.x, 0.75f, 0},
        {-FloorHalfExtents.x, 0.75f, 0},
        {0, 0.75f, FloorHalfExtents.z},
        {0, 0.75f, -FloorHalfExtents.z},
        {0, 0.75f, -2.5f}
    };
    const PxVec3 WallExtents[] =
    {
        {.2f, .75f, FloorHalfExtents.z},
        {.2f, .75f, FloorHalfExtents.z},
        {FloorHalfExtents.x, .75f, .2f},
        {FloorHalfExtents.x, .75f, .2f},
        {1.5f, .75f, .25f}
    };

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
        public:
            HWND window;
            DWORD processId;
        } search{nullptr, processId};
        EnumWindows([](HWND window, LPARAM parameter) -> BOOL
            {
            Search& result = *reinterpret_cast<Search*>(parameter);
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
        // 응답하지 않는 디버거가 ModelViewer의 메시지/렌더 스레드를 막지 않도록 한다.
        ShowWindowAsync(window, SW_RESTORE);
        SetForegroundWindow(window);
    }

    std::wstring FindPvdExecutable()
    {
        // VS의 작업 디렉터리에 의존하지 않고 실행 파일 위치에서 저장소를 찾는다.
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
    public:
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
            for (const float(&corner)[2] : corners)
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

bool PhysicsDemo::Startup(MapId map)
{
    if (!m_World.IsInitialized() || !LoadMap(map))
    {
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
    m_OverlayLinePSO = m_LinePSO;
    m_OverlayLinePSO.SetDepthStencilState(DepthStateDisabled);
    m_OverlayLinePSO.SetRenderTargetFormat(g_SceneColorBuffer.GetFormat(), DXGI_FORMAT_UNKNOWN);
    m_OverlayLinePSO.Finalize();
    return true;
}

void PhysicsDemo::Cleanup()
{
    m_PvdConnecting = false;
    ReleaseMap();
}

void PhysicsDemo::ReleaseMap()
{
    for (const MapBox& box : m_MapBoxes)
    {
        box.body->release();
    }
    m_MapBoxes.clear();
}

bool PhysicsDemo::AddMapBox(std::vector<MapBox>& boxes, const PxTransform& pose,
    const PxVec3& halfExtents, const PxVec3& color, const char* name,
    const PARS::PhysicsMaterialSettings& material)
{
    PxRigidStatic* body = m_World.CreateStaticBox(pose, halfExtents, name, material);
    if (!body)
    {
        return false;
    }
    boxes.push_back({body, halfExtents, color});
    return true;
}

bool PhysicsDemo::LoadMap(MapId map)
{
    if (!m_World.IsInitialized() || map >= MapId::Count)
    {
        return false;
    }
    // 새 지형을 모두 준비한 뒤 교체하여 생성 실패 시 기존 맵을 유지한다.
    std::vector<MapBox> boxes;
    const PxVec3 groundColor(.24f, .34f, .42f);
    const PARS::PhysicsMaterialSettings groundMaterial{.9f, .7f, 0.0f};
    bool ready = true;
    if (map == MapId::Cliff)
    {
        ready = AddMapBox(boxes, PxTransform(PxVec3(0, -1, 0)), PxVec3(6, 1, 6),
            PxVec3(.48f, .32f, .20f), "Cliff platform", groundMaterial);
        ready = ready && AddMapBox(boxes, PxTransform(PxVec3(0, -6.25f, 0)), PxVec3(30, .25f, 30),
            groundColor, "Lower ground", groundMaterial);
    }
    else
    {
        ready = AddMapBox(boxes, PxTransform(PxVec3(0, -.25f, 0)), FloorHalfExtents,
            groundColor, "Ground", groundMaterial);
    }
    if (map == MapId::TestYard)
    {
        for (size_t index = 0; index < _countof(WallCentres) && ready; ++index)
        {
            ready = AddMapBox(boxes, PxTransform(WallCentres[index]), WallExtents[index],
                PxVec3(.35f, .47f, .5f), "Wall");
        }
    }
    else if (map == MapId::Stairs)
    {
        for (uint32_t step = 0; step < 8 && ready; ++step)
        {
            const float height = .18f * static_cast<float>(step + 1);
            ready = AddMapBox(boxes, PxTransform(PxVec3(-3, height * .5f, -3 - .6f * step)),
                PxVec3(1.4f, height * .5f, .3f), PxVec3(.30f, .49f, .62f), "Step", groundMaterial);
        }
        const float angle = std::atan2(1.44f, 6.0f);
        const float length = std::sqrt(3.0f * 3.0f + .72f * .72f);
        ready = ready && AddMapBox(boxes,
            PxTransform(PxVec3(3, .72f - .15f * std::cos(angle), -5.8f), PxQuat(angle, PxVec3(1, 0, 0))),
            PxVec3(1.4f, .15f, length), PxVec3(.55f, .43f, .22f), "Ramp", groundMaterial);
        ready = ready && AddMapBox(boxes, PxTransform(PxVec3(-3, .72f, -8.5f)),
            PxVec3(1.4f, .72f, 1), PxVec3(.30f, .49f, .62f), "Stair landing", groundMaterial);
        ready = ready && AddMapBox(boxes, PxTransform(PxVec3(3, .72f, -9.7f)),
            PxVec3(1.4f, .72f, 1), PxVec3(.55f, .43f, .22f), "Ramp landing", groundMaterial);
    }
    if (!ready)
    {
        for (const MapBox& box : boxes)
        {
            box.body->release();
        }
        return false;
    }
    ReleaseMap();
    m_MapBoxes = std::move(boxes);
    m_Map = map;
    return true;
}

const char* PhysicsDemo::GetMapName(MapId map)
{
    switch (map)
    {
    case MapId::TestYard:
        return u8"기존 테스트장";
    case MapId::Flat:
        return u8"평지";
    case MapId::Stairs:
        return u8"계단 · 경사로";
    case MapId::Cliff:
        return u8"절벽 낙하장";
    default:
        return u8"맵 선택";
    }
}

const char* PhysicsDemo::GetMapDescription(MapId map)
{
    switch (map)
    {
    case MapId::TestYard:
        return u8"외곽 벽과 장애물이 있는 기존 공간.\n더미 충격과 균형 복구를 확인합니다.";
    case MapId::Flat:
        return u8"장애물이 없는 넓은 바닥.\n기본 이동과 발 디딤을 확인합니다.";
    case MapId::Stairs:
        return u8"왼쪽은 계단, 오른쪽은 경사로.\n높이가 달라지는 지형을 이동합니다.";
    case MapId::Cliff:
        return u8"아래 바닥까지 6m 높이의 발판.\n가장자리에서의 지지 상실과 낙하를 확인합니다.";
    default:
        return "";
    }
}

Math::Vector3 PhysicsDemo::GetSpawnLocation(float halfHeight) const
{
    return Math::Vector3(2, halfHeight, m_Map == MapId::Cliff ? 2.0f : 0.0f);
}

Math::Vector3 PhysicsDemo::GetDummySpawnOrigin() const
{
    // 절벽 앞쪽 경계(Z=-6)에서 30cm 안쪽에 더미를 배치한다.
    return Math::Vector3(3.5f, 0, m_Map == MapId::Cliff ? -5.7f : -2.5f);
}

void PhysicsDemo::Update(float)
{
    UpdatePvdConnection(); // 물리가 일시정지된 동안에도 연결 재시도를 수행해야 한다.
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
        m_PvdMessage = u8"실행 중인 PVD가 응답하지 않습니다. 다시 시작하여 연결을 복구하세요.";
        return;
    }
    ShowPvdWindow(window);
    // 일시정지 중에는 SDK가 오래된 소켓 상태를 유지할 수 있으므로 새 연결을 강제한다.
    if (m_World.ConnectPvd(true))
    {
        return;
    }

    if (!m_PvdProcessId)
    {
        const std::wstring executable = FindPvdExecutable();
        if (executable.empty())
        {
            m_PvdMessage = u8".tools/pvd/app에 PVD3.exe가 없습니다. 설치된 PVD를 연 뒤 다시 시도하세요.";
            return;
        }
        const std::wstring workingDirectory = executable.substr(0, executable.find_last_of(L"\\/"));
        STARTUPINFOW startup = {};
        startup.cb = sizeof(startup);
        startup.dwFlags = STARTF_USESHOWWINDOW;
        startup.wShowWindow = SW_SHOWNORMAL; // 버튼으로 실행을 요청한 대화형 디버거다.
        PROCESS_INFORMATION process = {};
        if (!CreateProcessW(executable.c_str(), nullptr, nullptr, nullptr, FALSE, 0, nullptr,
                workingDirectory.c_str(), &startup, &process))
        {
            m_PvdMessage = u8"PVD를 실행하지 못했습니다 (Windows 오류 " + std::to_string(GetLastError()) + ").";
            return;
        }
        m_PvdProcessId = process.dwProcessId;
        CloseHandle(process.hThread);
        CloseHandle(process.hProcess);
        m_PvdMessage = u8"PVD 실행 후 연결 포트가 열리기를 기다리는 중...";
    }
    else
    {
        m_PvdMessage = u8"PVD 실행 중. 127.0.0.1:5425 연결 대기 중...";
    }

    const uint64_t now = GetTickCount64();
    m_PvdConnecting = true;
    m_PvdNextAttempt = now + 500;
    m_PvdConnectDeadline = now + 30000;
}

void PhysicsDemo::RestartPvd()
{
    // 멈춘 디버거의 종료는 명시적인 복구 버튼에서만 수행하며, 저장소에 포함된 것으로
    // 확인한 실행 파일만 대상으로 한다. 임의의 PVD 프로세스는 종료하지 않는다.
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
        m_PvdMessage = u8"포함된 PVD를 다시 시작하지 못했습니다. 응답하지 않는 디버거를 닫고 다시 연결하세요.";
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
        m_PvdMessage = m_PvdUnresponsive ? u8"PVD가 응답하지 않습니다. 디버거를 다시 시작한 뒤 재시도하세요." : u8"PVD가 127.0.0.1:5425 연결을 받지 않았습니다. "
                                                                                                                u8"PVD 도구 모음 맨 오른쪽 버튼에서 외부 연결을 허용한 뒤 다시 시도하세요.";
        return;
    }
    // PVD가 OpenGL을 시작할 때 창 메시지 처리를 일시적으로 멈출 수 있다.
    // 시작 중에는 이를 멈춘 디버거로 판단하지 않고 계속 재시도한다.
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
    const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor, bool comparisonScene)
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

    // fetchResults() 이후 PhysX의 실제 위치와 회전을 모두 복사한다.
    std::vector<Vertex> vertices(36 * (m_MapBoxes.size() + 1));
    Vertex* output = vertices.data();
    if (comparisonScene)
    {
        AppendBox(output, PxTransform(PxVec3(0, -.5f, 0)), PxVec3(250, .5f, 250), PxVec3(.24f, .34f, .42f));
    }
    else
    {
        for (const MapBox& box : m_MapBoxes)
        {
            AppendBox(output, box.body->getGlobalPose(), box.halfExtents, box.color);
        }
    }
    const UINT count = static_cast<UINT>(output - vertices.data());
    if (count)
    {
        context.SetDynamicVB(0, count, sizeof(Vertex), vertices.data());
        context.Draw(count);
    }
}

void PhysicsDemo::RenderSpheres(GraphicsContext& context, const Math::Camera& camera,
    const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor,
    const std::vector<PxRigidDynamic*>& spheres, float radius)
{
    if (spheres.empty())
    {
        return;
    }
    constexpr int latitudeCount = 12, longitudeCount = 24;
    std::vector<Vertex> vertices;
    vertices.reserve(spheres.size() * latitudeCount * longitudeCount * 6);
    const std::function<physx::PxVec3(int, int)> normal = [=](int latitude, int longitude) -> physx::PxVec3
    {
        const float a = -PxHalfPi + PxPi * latitude / latitudeCount;
        const float b = PxTwoPi * longitude / longitudeCount;
        return PxVec3(std::cos(a) * std::cos(b), std::sin(a), std::cos(a) * std::sin(b));
    };
    for (const physx::PxRigidDynamic* sphere : spheres)
    {
        const physx::PxTransform pose = sphere->getGlobalPose();
        for (int y = 0; y < latitudeCount; ++y)
        {
            for (int x = 0; x < longitudeCount; ++x)
            {
                const PxVec3 corners[] = {normal(y, x), normal(y + 1, x), normal(y + 1, x + 1),
                    normal(y, x), normal(y + 1, x + 1), normal(y, x + 1)};
                for (const physx::PxVec3& n : corners)
                {
                    vertices.push_back({pose.transform(n * radius), pose.q.rotate(n), PxVec3(1, .65f, .08f)});
                }
            }
        }
    }
    context.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET);
    context.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, true);
    context.SetRenderTarget(g_SceneColorBuffer.GetRTV(), g_SceneDepthBuffer.GetDSV());
    context.SetViewportAndScissor(viewport, scissor);
    context.SetRootSignature(m_RootSignature);
    context.SetPipelineState(m_PSO);
    context.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    context.SetDynamicConstantBufferView(0, sizeof(Math::Matrix4), &camera.GetViewProjMatrix());
    context.SetDynamicVB(0, static_cast<UINT>(vertices.size()), sizeof(Vertex), vertices.data());
    context.Draw(static_cast<UINT>(vertices.size()));
}

void PhysicsDemo::RenderCapsules(GraphicsContext& context, const Math::Camera& camera,
    const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor)
{
    std::vector<PxDebugLine> lines;
    m_World.GetCapsuleDebugLines(lines);
    RenderDebugLines(context, camera, viewport, scissor, lines);
}

void PhysicsDemo::RenderDebugLines(GraphicsContext& context, const Math::Camera& camera,
    const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor,
    const std::vector<PxDebugLine>& lines, bool throughGeometry)
{
    if (lines.empty())
    {
        return;
    }
    std::vector<Vertex> vertices;
    vertices.reserve(lines.size() * 2);
    for (const physx::PxDebugLine& line : lines)
    {
        const PxVec3 color(float((line.color0 >> 16) & 255) / 255, float((line.color0 >> 8) & 255) / 255, float(line.color0 & 255) / 255);
        vertices.push_back({line.pos0, PxVec3(0, 1, 0), color});
        vertices.push_back({line.pos1, PxVec3(0, 1, 0), color});
    }
    context.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET, throughGeometry);
    if (throughGeometry)
    {
        context.SetRenderTarget(g_SceneColorBuffer.GetRTV());
    }
    else
    {
        context.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_READ, true);
        context.SetRenderTarget(g_SceneColorBuffer.GetRTV(), g_SceneDepthBuffer.GetDSV_DepthReadOnly());
    }
    context.SetViewportAndScissor(viewport, scissor);
    context.SetRootSignature(m_RootSignature);
    context.SetPipelineState(throughGeometry ? m_OverlayLinePSO : m_LinePSO);
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
    ImGui::SetNextWindowSize(ImVec2(410, 460), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(390, 360), ImVec2(FLT_MAX, FLT_MAX));
    if (ImGui::Begin(u8"PARS - 물리", &visible))
    {
        bool paused = m_GameWorld.IsPhysicsPaused();
        bool slowMotion = m_GameWorld.GetPhysicsTimeScale() < 1.0f;
        ImGui::TextUnformatted(paused ? u8"월드 일시정지" : u8"시뮬레이션 중");
        if (ImGui::Checkbox(u8"느리게 (0.1배, F)", &slowMotion))
        {
            m_GameWorld.SetPhysicsTimeScale(slowMotion ? 0.1f : 1.0f);
        }
        if (ImGui::Checkbox(u8"월드 일시정지", &paused))
        {
            m_GameWorld.SetPhysicsPaused(paused);
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!paused);
        if (ImGui::Button(u8"1/60초 진행"))
        {
            m_GameWorld.RequestPhysicsStep();
        }
        ImGui::EndDisabled();
        ImGui::Separator();
        ImGui::Text(u8"%.1f FPS | 물리: 60 Hz | 속도: %.2f배", ImGui::GetIO().Framerate,
            m_GameWorld.GetPhysicsTimeScale());
        ImGui::Text(u8"물리 스텝: %llu | 현재 프레임: %u", static_cast<unsigned long long>(m_World.GetStepCount()),
            m_World.GetLastSubsteps());
        ImGui::Text(u8"지연으로 제외된 시간: %.2f초", m_World.GetDroppedTime());
        ImGui::Separator();
        bool showCapsule = m_World.IsCapsuleDebugVisible();
        if (ImGui::Checkbox(u8"이동 캡슐 윤곽선", &showCapsule))
        {
            m_World.SetCapsuleDebugVisible(showCapsule);
        }
        if (!PhysicsWorld::SupportsPvd())
        {
            ImGui::TextWrapped(u8"PVD 기록은 Debug 또는 Profile 빌드에서 사용할 수 있습니다.");
        }
        else
        {
            const bool connected = m_World.IsPvdConnected();
            if (ImGui::Checkbox(u8"PVD에 장면 쿼리 표시", &m_PvdSceneQueries))
            {
                m_World.SetPvdSceneQueries(m_PvdSceneQueries);
            }
            ImGui::TextColored(connected ? ImVec4(0.4f, 0.95f, 0.65f, 1) : ImVec4(1, 0.75f, 0.3f, 1),
                "PVD: %s", connected ? u8"연결됨" : u8"연결 안 됨");
            ImGui::BeginDisabled(m_PvdConnecting);
            if (ImGui::Button(m_PvdConnecting ? u8"연결 중..." : connected ? u8"PVD 다시 연결"
                                                                           : u8"PVD 실행 / 연결"))
            {
                ConnectPvd();
            }
            ImGui::EndDisabled();
            if (m_PvdUnresponsive && ImGui::Button(u8"응답 없는 PVD 다시 시작"))
            {
                RestartPvd();
            }
            if (connected)
            {
                ImGui::TextWrapped(u8"월드를 갱신할 때 캐릭터와 리지드바디를 PVD에 기록합니다.");
            }
            else if (!m_PvdMessage.empty())
            {
                ImGui::TextWrapped("%s", m_PvdMessage.c_str());
            }
            else
            {
                ImGui::TextWrapped(u8"연결 버튼을 누르면 로컬 PVD 프로그램을 자동으로 실행합니다.");
            }
        }
        ImGui::Text(u8"PhysX 오류: %u개", m_World.GetErrorCount());
        ImGui::TextWrapped(u8"Tab: 캐릭터 / 자유 카메라 전환. F1: 패널 표시. 단위: m.");
    }
    ImGui::End();
}
