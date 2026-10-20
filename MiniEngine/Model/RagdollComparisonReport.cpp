#include "RagdollComparison.h"
#include <fstream>
#include <iomanip>
#include <locale>

namespace PARS
{
    namespace
    {
        std::string JsonString(const std::string& value)
        {
            std::string result = "\"";
            const char* hex = "0123456789abcdef";
            for (unsigned char c : value)
            {
                if (c == '"' || c == '\\')
                {
                    result += '\\';
                    result += c;
                }
                else if (c < 32 || c == '<' || c == '>' || c == '&')
                {
                    result += "\\u00";
                    result += hex[c >> 4];
                    result += hex[c & 15];
                }
                else
                {
                    result += c;
                }
            }
            return result + '"';
        }

        void Vector(std::ostream& out, const DirectX::XMFLOAT3& value)
        {
            out << '[' << value.x << ',' << value.y << ',' << value.z << ']';
        }

        void FrameTransform(std::ostream& out, const PhysicsJointFrame& frame)
        {
            out << "{\"p\":";
            Vector(out, frame.offset);
            out << ",\"q\":[" << frame.rotation.x << ',' << frame.rotation.y << ',' << frame.rotation.z << ',' << frame.rotation.w << "]}";
        }

        template<size_t N>
        void Array(std::ostream& out, const std::array<float, N>& values)
        {
            out << '[';
            for (size_t i = 0; i < N; ++i)
            {
                out << (i ? "," : "") << values[i];
            }
            out << ']';
        }

        void Rotation(std::ostream& out, const ComparisonJointRotation& value)
        {
            out << "{\"a\":";
            Array(out, value.angles);
            out << ",\"e\":";
            Array(out, value.excess);
            out << ",\"singular\":" << (value.singular ? "true" : "false") << '}';
        }
    }

    bool RagdollComparison::ExportHtml(const std::string& path, const std::string& description, std::string& error) const
    {
        if (m_Frames.empty())
        {
            error = "Record at least one physics step before exporting.";
            return false;
        }
        std::ofstream out(path, std::ios::binary);
        if (!out)
        {
            error = "Could not open the HTML report path.";
            return false;
        }
        out.imbue(std::locale::classic());
        out << std::setprecision(9);
        out << u8R"HTML(<!doctype html>
<html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PARS · 래그돌 비교 보고서</title>
<style>
:root{color-scheme:light;--ink:#15253b;--muted:#52647b;--blue:#1674c5;--orange:#bd590a}
*{box-sizing:border-box}body{margin:0;background:#eef2f6;color:var(--ink);font:14px/1.65 system-ui,"Malgun Gothic",sans-serif;word-break:keep-all}
main{max-width:1440px;margin:auto;padding:36px}h1{font-size:30px;margin:0 0 8px}h2{margin-top:0;font-size:21px}
section{background:white;border:1px solid #dbe3ec;border-radius:12px;padding:24px;margin:20px 0}
p{max-width:1100px}.muted{color:var(--muted)}.scroll{overflow:auto;max-height:460px}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;white-space:nowrap}
th,td{text-align:right;padding:9px 12px;border-bottom:1px solid #e2e8f0}th:first-child,td:first-child{text-align:left}
th{position:sticky;top:0;background:#e9eff6}tbody tr:nth-child(even){background:#f7f9fc}
button,select{font:inherit;padding:7px 12px;border:1px solid #bac8d8;border-radius:6px;background:white;color:var(--ink)}
button{cursor:pointer}input[type=range]{width:min(600px,70%)}canvas{width:100%;height:230px;background:#f8fafc;border-radius:8px}
.bad{color:#a74308;font-weight:650}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}.legend{font-size:13px}
code{font-size:12px}details{margin:12px 0}pre{overflow:auto;max-height:400px}
@media(max-width:800px){main{padding:16px}.grid{grid-template-columns:1fr}section{padding:16px}}
@media print{body{background:white}main{padding:0;max-width:none}section{border:0;padding:10px 0;break-inside:avoid}.scroll{max-height:none;overflow:visible}
table{font-size:9px;white-space:normal}td,th{padding:4px}button,input,select{display:none}canvas{height:180px}th{position:static}}
</style>
<main><h1>PARS / 래그돌 비교 보고서</h1><p id="description" class="muted"></p>
<p>플레이어와 분리된 두 더미가 같은 애니메이션과 지정된 이동 경로를 따릅니다. 키네마틱 더미의 원본 자세가 절대 기준입니다.
최초 배치 차이만 제거하며, 골반이 기준에서 벗어난 이동은 오차로 남깁니다. 물리 계산이 완료될 때마다 1/60초 간격으로 기록하고 시작 직후의 과도응답도 포함합니다.
이동량 오차는 <b>두 변위 벡터의 차이 크기</b>이며, 회전 오차는 두 쿼터니언 사이의 최단 회전각입니다.</p>
<button onclick="window.print()">인쇄 / PDF 저장</button> <button id="csv">전체 측정값 내려받기 (CSV)</button>
<section><h2>애니메이션 재생 구간</h2><p class="muted">전환 시간은 다음 동작의 측정 구간에 포함됩니다. 시간 그래프의 음영은 블렌딩 구간입니다. 아래 바디·관절 요약은 전체 기록 기준입니다.</p>
<div class="scroll"><table id="playback"></table></div></section>
<section><h2>바디별 이동 비교</h2><p class="muted">최대 위치 오차가 큰 순서입니다. 위치는 콜라이더 중심 기준이며, 품질을 하나의 합격·불합격 점수로 환산하지 않습니다. 본 이름은 에셋의 식별자를 그대로 표시합니다.</p>
<div class="scroll"><table id="bodies"></table></div></section>
<section><h2>관절 제한과 추종 오차</h2><p class="muted">원본 → 보정 목표: 관절 제한 때문에 목표 자세가 변경된 각도입니다.
보정 목표 → 실제: 제한 안의 목표와 물리 결과 사이의 추종 오차입니다. 제한값, 조인트 기준 좌표계, 원본 애니메이션과 제어 강도를 함께 확인하세요.
스윙 Y/Z는 오일러 각도가 아닌 PhysX D6의 quarter-angle 좌표로 계산합니다. 두 축을 함께 제한하는 스윙 원뿔도 검사합니다.</p>
<div class="scroll"><table id="joints"></table></div></section>
<section><h2>시간별 측정값</h2>
<p><label>바디 <select id="body"></select></label> <label>관절 <select id="joint"></select></label></p>
<div class="grid"><div><p>바디 오차: <span style="color:#1674c5">위치 (cm)</span> / <span style="color:#bd590a">이동량 (cm/스텝)</span></p><canvas id="bodyPlot"></canvas></div>
<div><p>관절 오차: <span style="color:#1674c5">원본 → 보정 목표</span> / <span style="color:#bd590a">보정 목표 → 실제</span> (°)</p><canvas id="jointPlot"></canvas></div></div>
<p><input id="frame" type="range" min="0" value="0"><b id="time"></b></p>
<p id="position"></p><div class="scroll"><table id="angles"></table></div>
<p id="range" class="muted"></p><p id="singular" class="bad"></p>
<div class="grid"><div><p>기준 / 실제 이동 경로의 측면도 (가로 Z / 세로 Y, m)</p><canvas id="trajectory"></canvas><span class="legend">파랑: 키네마틱 기준 · 주황: 액티브 래그돌 실제 결과</span></div>
<div><p>선택한 축의 각도 (°): <select id="axis"><option value="0">트위스트 X</option><option value="1">스윙 Y</option><option value="2">스윙 Z</option></select></p><canvas id="axisPlot"></canvas>
<span class="legend">파랑: 원본 · 초록: 보정 목표 · 주황: 실제 · 회색: 축별 허용 범위 (스윙 원뿔은 두 축을 함께 제한)</span></div></div>
</section>
<section><h2>관절 제한 위반 내역</h2><p class="muted">잠긴 축도 위반 여부를 검사하며, 자유 축에는 제한 초과가 없습니다.
원뿔 초과량은 스윙 각도 좌표에서 경계까지의 방사 방향 거리(°)이며, 최단 회전 보정각과는 다릅니다.
위반 판정 허용치는 0.0001 rad입니다. 지속 시간은 위반 표본 수 × 고정 시간 간격으로 계산하며, 최초·마지막 시각이 -1이면 위반이 없었다는 뜻입니다.</p>
<div class="scroll"><table id="violations"></table></div></section>
<section><h2>실험 시작 시 설정</h2><p class="muted">시작할 때 복사한 설정입니다. 이후 에디터에서 수정한 값은 이 실험에 영향을 주지 않습니다.
모든 원본 측정값이 이 HTML 파일에 포함되어 있습니다. 내부 저장 단위는 위치 m, 각도 rad입니다.</p>
<details><summary>바디 질량·형상 및 관절 좌표계·제한·드라이브 설정 보기</summary><pre id="settings"></pre></details></section>
</main><script>
"use strict";
const data = )HTML";
        out << "{\"description\":" << JsonString(description) << ",\"step\":" << m_Step << ",\"bodies\":[";
        for (size_t i = 0; i < m_Asset.bodies.size(); ++i)
        {
            const auto& body = m_Asset.bodies[i];
            out << (i ? "," : "") << "{\"name\":" << JsonString(body.bone) << ",\"massKg\":" << body.mass
                << ",\"shape\":" << static_cast<int>(body.shape) << ",\"radius\":" << body.radius << ",\"halfLength\":" << body.halfLength << ",\"halfExtents\":";
            Vector(out, body.halfExtents);
            out << ",\"frame\":";
            FrameTransform(out, PhysicsJointFrame{body.offset, body.rotation});
            out << '}';
        }
        out << "],\"joints\":[";
        for (size_t i = 0; i < m_Asset.constraints.size(); ++i)
        {
            const auto& joint = m_Asset.constraints[i];
            out << (i ? "," : "") << "{\"name\":" << JsonString(joint.childBone) << ",\"parent\":" << JsonString(joint.parentBone)
                << ",\"motion\":[" << static_cast<int>(joint.twist) << ',' << static_cast<int>(joint.swingY) << ',' << static_cast<int>(joint.swingZ)
                << "],\"min\":[" << joint.twistMin << ',' << -joint.swingYLimit << ',' << -joint.swingZLimit
                << "],\"max\":[" << joint.twistMax << ',' << joint.swingYLimit << ',' << joint.swingZLimit << "],\"slerp\":" << (joint.useSlerpDrive ? "true" : "false")
                << ",\"collision\":" << (joint.enableCollision ? "true" : "false") << ",\"parentFrame\":";
            FrameTransform(out, joint.parentFrame);
            out << ",\"childFrame\":";
            FrameTransform(out, joint.childFrame);
            out << ",\"drives\":[";
            for (int axis = 0; axis < 3; ++axis)
            {
                const auto& drive = joint.drives[axis];
                out << (axis ? "," : "") << "{\"enabled\":" << (drive.enabled ? "true" : "false")
                    << ",\"kp\":" << drive.stiffness << ",\"kd\":" << drive.damping << ",\"maxNm\":" << drive.maxTorque << '}';
            }
            out << "]}";
        }
        out << "],\"frames\":[";
        for (size_t i = 0; i < m_Frames.size(); ++i)
        {
            const auto& frame = m_Frames[i];
            out << (i ? "," : "") << "{\"t\":" << frame.time << ",\"clip\":" << frame.clipTime
                << ",\"animation\":" << JsonString(frame.clipName) << ",\"blend\":" << frame.blendAlpha << ",\"b\":[";
            for (size_t b = 0; b < frame.bodies.size(); ++b)
            {
                const auto& body = frame.bodies[b];
                out << (b ? "," : "") << "{\"r\":";
                Vector(out, body.reference);
                out << ",\"a\":";
                Vector(out, body.actual);
                out << ",\"d\":";
                Vector(out, body.difference);
                out << ",\"v\":";
                Vector(out, body.movementDifference);
                out << ",\"p\":" << body.positionError << ",\"m\":" << body.movementError << ",\"q\":" << body.rotationError << '}';
            }
            out << "],\"j\":[";
            for (size_t j = 0; j < frame.joints.size(); ++j)
            {
                const auto& joint = frame.joints[j];
                out << (j ? "," : "") << "{\"r\":";
                Rotation(out, joint.raw);
                out << ",\"b\":";
                Rotation(out, joint.bounded);
                out << ",\"a\":";
                Rotation(out, joint.actual);
                out << ",\"c\":" << joint.clampError << ",\"k\":" << joint.trackingError << ",\"e\":" << joint.totalError << '}';
            }
            out << "]}";
        }
        out << u8R"HTML(]};
const $=id=>document.getElementById(id), deg=180/Math.PI, fmt=v=>Number(v).toFixed(3), axes=["트위스트 X","스윙 Y","스윙 Z","스윙 원뿔"];
function table(id,headers,rows){const t=$(id);t.replaceChildren();const head=t.createTHead().insertRow();headers.forEach(s=>{const c=document.createElement("th");c.textContent=s;head.append(c)});
const body=t.createTBody();rows.forEach(row=>{const tr=body.insertRow();row.forEach(s=>{const c=tr.insertCell();c.textContent=typeof s==="number"?fmt(s):s})})}
function stats(values){let sum=0,sq=0,peak=-1,at=0;values.forEach((v,i)=>{sum+=v;sq+=v*v;if(v>peak){peak=v;at=data.frames[i].t}});
return {mean:sum/values.length,rms:Math.sqrt(sq/values.length),peak,at}}
function exposure(values){let count=0,longest=0,run=0,first=-1,last=-1;values.forEach((v,i)=>{if(v>1e-4){if(!count)first=data.frames[i].t;last=data.frames[i].t;count++;longest=Math.max(longest,++run)}else run=0});
return {...stats(values.map(v=>v*deg)),percent:100*count/values.length,duration:count*data.step,longest:longest*data.step,first,last}}
function download(name,text,type){const url=URL.createObjectURL(new Blob([text],{type})),a=document.createElement("a");a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)}
$("description").textContent=data.description;
const animationName=name=>({"Idle":"대기 (Idle)","Walking":"걷기 (Walking)","Slow Run":"달리기 (Slow Run)"}[name]||name||"이름 미기록");
const stages=[];
data.frames.forEach((f,i)=>{let stage=stages[stages.length-1];if(!stage||stage.name!==f.animation){
stage={name:f.animation,start:i?data.frames[i-1].t:0,end:f.t,count:0,blending:f.blend<1,blendEnd:null};stages.push(stage)}
stage.end=f.t;stage.count++;if(stage.blending&&stage.blendEnd===null&&f.blend>=1)stage.blendEnd=f.t});
table("playback",["동작","시작 (초)","종료 (초)","기록 시간 (초)","표본 수","블렌딩 완료 (초)"],
stages.map(s=>[animationName(s.name),s.start,s.end,s.count*data.step,String(s.count),s.blending?(s.blendEnd===null?"진행 중":s.blendEnd):"-"]));
const frameSettings=f=>({"위치 (m)":f.p,"회전 쿼터니언 (x, y, z, w)":f.q});
$("settings").textContent=JSON.stringify({
"물리 시간 간격 (초)":data.step,
"바디":data.bodies.map(b=>({"본":b.name,"질량 (kg)":b.massKg,"형상":["캡슐","상자","구"][b.shape],"반지름 (m)":b.radius,"원통 반길이 (m)":b.halfLength,"상자 반크기 (m)":b.halfExtents,"바디 기준 좌표계":frameSettings(b.frame)})),
"관절":data.joints.map(j=>({"부모 본":j.parent,"자식 본":j.name,"축별 운동":j.motion.map((m,a)=>`${axes[a]}: ${["잠금","제한","자유"][m]}`),"설정 최소각 (rad)":j.min,"설정 최대각 (rad)":j.max,"SLERP 드라이브":j.slerp?"사용":"사용 안 함","연결된 바디 간 충돌":j.collision?"사용":"사용 안 함","부모 기준 좌표계":frameSettings(j.parentFrame),"자식 기준 좌표계":frameSettings(j.childFrame),"드라이브":j.drives.map((d,a)=>({"축":axes[a],"설정상 활성":d.enabled?"켜짐":"꺼짐","강성 Kp":d.kp,"감쇠 Kd":d.kd,"최대 토크 (Nm)":d.maxNm}))}))
},null,2);
data.bodies.forEach((b,i)=>$("body").add(new Option(b.name,i)));
data.joints.forEach((j,i)=>$("joint").add(new Option(j.parent+" → "+j.name,i)));
$("frame").max=data.frames.length-1;
const br=data.bodies.map((b,i)=>{const p=stats(data.frames.map(f=>f.b[i].p*100)),m=stats(data.frames.map(f=>f.b[i].m*100)),r=stats(data.frames.map(f=>f.b[i].q*deg));
return [b.name,p.mean,p.rms,p.peak,p.at,m.mean,m.peak,r.mean,r.peak]});
table("bodies",["바디","위치 오차 평균 (cm)","위치 오차 RMS (cm)","위치 오차 최대 (cm)","최대 시각 (초)","이동량 오차 평균 (cm/스텝)","이동량 오차 최대 (cm/스텝)","회전 오차 평균 (°)","회전 오차 최대 (°)"],br.sort((a,b)=>b[3]-a[3]));
const jr=data.joints.map((j,i)=>{const c=stats(data.frames.map(f=>f.j[i].c*deg)),k=stats(data.frames.map(f=>f.j[i].k*deg)),t=stats(data.frames.map(f=>f.j[i].e*deg));
return [j.name,c.mean,c.peak,c.at,k.mean,k.peak,t.peak,...axes.map((_,a)=>exposure(data.frames.map(f=>f.j[i].r.e[a])).percent)]});
table("joints",["관절","평균 목표 보정 (°)","최대 목표 보정 (°)","최대 보정 시각 (초)","평균 추종 오차 (°)","최대 추종 오차 (°)","최대 전체 오차 (°)","트위스트 위반 (%)","스윙 Y 위반 (%)","스윙 Z 위반 (%)","원뿔 위반 (%)"],jr.sort((a,b)=>b[2]-a[2]));
function chart(id,series,selected,xlabel="시간 (초)",bounds=[]){const c=$(id),ctx=c.getContext("2d"),w=c.width=1100,h=c.height=250;
const all=series.flatMap(s=>s.points),xs=all.map(p=>p[0]),ys=all.map(p=>p[1]).concat(bounds);
let xmin=Math.min(...xs),xmax=Math.max(...xs),ymin=Math.min(0,...ys),ymax=Math.max(...ys);if(xmax-xmin<1e-6)xmax=xmin+1;if(ymax-ymin<1e-6)ymax=ymin+1;
const x=v=>60+(v-xmin)/(xmax-xmin)*(w-90),y=v=>h-35-(v-ymin)/(ymax-ymin)*(h-65);
ctx.clearRect(0,0,w,h);ctx.font="14px system-ui";ctx.fillStyle="#52647b";ctx.fillText(fmt(ymax),3,23);ctx.fillText(fmt(ymin),3,h-32);ctx.fillText(fmt(xmin),60,h-9);ctx.fillText(fmt(xmax),w-80,h-9);ctx.fillText(xlabel,w/2,h-9);
ctx.strokeStyle="#c4ceda";ctx.beginPath();ctx.moveTo(60,20);ctx.lineTo(60,h-35);ctx.lineTo(w-30,h-35);ctx.stroke();
bounds.forEach(v=>{ctx.setLineDash([6,5]);ctx.beginPath();ctx.moveTo(60,y(v));ctx.lineTo(w-30,y(v));ctx.stroke();ctx.setLineDash([])});
if(xlabel==="시간 (초)")stages.filter(s=>s.blending).forEach(s=>{ctx.fillStyle="rgba(115,134,157,.15)";ctx.fillRect(x(s.start),20,x(s.blendEnd===null?s.end:s.blendEnd)-x(s.start),h-55);
ctx.fillStyle="#52647b";ctx.fillText(animationName(s.name),x(s.start)+4,35)});
series.forEach(s=>{ctx.strokeStyle=s.color;ctx.lineWidth=2;ctx.beginPath();s.points.forEach((p,i)=>{if(i)ctx.lineTo(x(p[0]),y(p[1]));else ctx.moveTo(x(p[0]),y(p[1]))});ctx.stroke()});
if(selected!==null){ctx.strokeStyle="#263b52";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x(selected),20);ctx.lineTo(x(selected),h-35);ctx.stroke()}}
function refresh(){const bi=+$("body").value,ji=+$("joint").value,fi=+$("frame").value,f=data.frames[fi],b=f.b[bi],a=+$("axis").value;
$("time").textContent=`표본 ${fi+1} | 실험 시간 ${fmt(f.t)}초 | ${animationName(f.animation)} | 클립 시간 ${fmt(f.clip)}초 | ${f.blend<1?"전환 블렌딩":"목표 동작"} ${(f.blend*100).toFixed(1)}%`;
$("position").textContent=`${data.bodies[bi].name} | 기준 위치 XYZ (m): ${b.r.map(fmt).join(", ")} | 실제 위치 XYZ (m): ${b.a.map(fmt).join(", ")} | 위치 차이 (cm): ${b.d.map(v=>fmt(v*100)).join(", ")} | 이동량 차이 (cm/스텝): ${b.v.map(v=>fmt(v*100)).join(", ")}`;
chart("bodyPlot",[{color:"#1674c5",points:data.frames.map(f=>[f.t,f.b[bi].p*100])},{color:"#bd590a",points:data.frames.map(f=>[f.t,f.b[bi].m*100])}],f.t);
chart("trajectory",[{color:"#1674c5",points:data.frames.map(f=>[f.b[bi].r[2],f.b[bi].r[1]])},{color:"#bd590a",points:data.frames.map(f=>[f.b[bi].a[2],f.b[bi].a[1]])}],null,"Z (m)");
if(!data.joints.length)return;const j=f.j[ji],cfg=data.joints[ji];
chart("jointPlot",[{color:"#1674c5",points:data.frames.map(f=>[f.t,f.j[ji].c*deg])},{color:"#bd590a",points:data.frames.map(f=>[f.t,f.j[ji].k*deg])}],f.t);
const bounds=cfg.motion[a]===2?[]:cfg.motion[a]===0?[0]:[cfg.min[a]*deg,cfg.max[a]*deg];
chart("axisPlot",[{color:"#1674c5",points:data.frames.map(f=>[f.t,f.j[ji].r.a[a]*deg])},{color:"#27815d",points:data.frames.map(f=>[f.t,f.j[ji].b.a[a]*deg])},{color:"#bd590a",points:data.frames.map(f=>[f.t,f.j[ji].a.a[a]*deg])}],f.t,"시간 (초)",bounds);
const rows=axes.map((name,a)=>[name,a===3?(cfg.motion[1]===1&&cfg.motion[2]===1?"두 축의 타원 제한":"적용 안 됨"):cfg.motion[a]===2?"자유":cfg.motion[a]===0?"잠금 (0°)":`${fmt(cfg.min[a]*deg)} ~ ${fmt(cfg.max[a]*deg)}`,
...(a<3?[j.r.a[a]*deg,j.b.a[a]*deg,j.a.a[a]*deg]:["-","-","-"]),j.r.e[a]*deg,j.a.e[a]*deg]);
table("angles",["축","허용 범위 (°)","원본 (°)","보정 목표 (°)","실제 (°)","원본 초과량 (°)","실제 초과량 (°)"],rows);
table("violations",["축","위반 비율 (%)","총 지속 (초)","최장 연속 (초)","최초 (초)","마지막 (초)","최대 초과 (°)","최대 시각 (초)","실제 최대 초과 (°)"],axes.map((name,a)=>{
const e=exposure(data.frames.map(f=>f.j[ji].r.e[a])),actual=exposure(data.frames.map(f=>f.j[ji].a.e[a]));return [name,e.percent,e.duration,e.longest,e.first,e.last,e.peak,e.at,actual.peak]}));
$("range").textContent=axes.slice(0,3).map((name,a)=>{const v=data.frames.map(f=>f.j[ji].r.a[a]*deg);return `${name} 원본 범위: ${fmt(Math.min(...v))} ~ ${fmt(Math.max(...v))}°`}).join(" | ");
const singular=data.frames.filter(f=>f.j[ji].r.singular).length;$("singular").textContent=singular?`주의: ${singular}개 표본은 스윙이 180°에 가까워 트위스트 분해가 불명확합니다. 조인트 기준 좌표계를 확인하세요.`:"";}
["body","joint","frame","axis"].forEach(id=>$(id).addEventListener("input",refresh));
$("csv").onclick=()=>{const rows=[["종류","본 이름","실험 시간 (초)","클립 시간 (초)","애니메이션","블렌딩 비율","기준 X (m)","기준 Y (m)","기준 Z (m)","실제 X (m)","실제 Y (m)","실제 Z (m)","위치 차이 X (m)","위치 차이 Y (m)","위치 차이 Z (m)","이동량 차이 X (m/스텝)","이동량 차이 Y (m/스텝)","이동량 차이 Z (m/스텝)","위치 오차 (m)","이동량 오차 (m/스텝)","회전 오차 (rad)","목표 보정각 (rad)","추종 오차 (rad)","전체 오차 (rad)","원본 트위스트 (rad)","원본 스윙 Y (rad)","원본 스윙 Z (rad)","보정 트위스트 (rad)","보정 스윙 Y (rad)","보정 스윙 Z (rad)","실제 트위스트 (rad)","실제 스윙 Y (rad)","실제 스윙 Z (rad)","원본 트위스트 초과 (rad)","원본 스윙 Y 초과 (rad)","원본 스윙 Z 초과 (rad)","원본 원뿔 초과 (rad)"]];
data.frames.forEach(f=>{f.b.forEach((b,i)=>rows.push(["바디",data.bodies[i].name,f.t,f.clip,animationName(f.animation),f.blend,...b.r,...b.a,...b.d,...b.v,b.p,b.m,b.q,...Array(16).fill("")]));
f.j.forEach((j,i)=>rows.push(["관절",data.joints[i].name,f.t,f.clip,animationName(f.animation),f.blend,...Array(15).fill(""),j.c,j.k,j.e,...j.r.a,...j.b.a,...j.a.a,...j.r.e]))});
const quote=v=>'"'+String(v).replace(/"/g,'""')+'"';download("래그돌-비교.csv","\ufeff"+rows.map(r=>r.map(quote).join(",")).join("\r\n"),"text/csv;charset=utf-8")};
refresh();
</script></html>)HTML";
        out.flush();
        if (!out)
        {
            error = "Could not finish writing the HTML report.";
            return false;
        }
        error.clear();
        return true;
    }
}
