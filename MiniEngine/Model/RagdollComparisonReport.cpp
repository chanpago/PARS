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
        out << R"HTML(<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PARS &middot; Ragdoll comparison</title>
<style>
:root{color-scheme:light;--ink:#15253b;--muted:#52647b;--blue:#1674c5;--orange:#bd590a}
*{box-sizing:border-box}body{margin:0;background:#eef2f6;color:var(--ink);font:14px/1.55 system-ui,sans-serif}
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
<main><h1>PARS / Ragdoll comparison</h1><p id="description" class="muted"></p>
<p>Two non-player dummies share the same clip clock and prescribed path. The kinematic pose is the unmodified reference.
Initial placement is removed once; pelvis drift remains. Every sample follows a complete 1/60 s physics step; startup is included.
Movement error is the norm of the <b>difference between displacement vectors</b>. Rotations use quaternion shortest-angle distance.</p>
<button onclick="window.print()">Print / save PDF</button> <button id="csv">Download all samples (CSV)</button>
<section><h2>Body trajectories</h2><p class="muted">Sorted by peak position error. Positions are collider centres. No universal pass/fail score is assigned.</p>
<div class="scroll"><table id="bodies"></table></div></section>
<section><h2>Joint limits and tracking</h2><p class="muted">Raw &rarr; bounded: target change caused by limit clamping.
Bounded &rarr; actual: remaining tracking error. Inspect limits, joint frames, animation and controller strength separately.
Swing Y/Z are PhysX D6 quarter-angle coordinates, not Euler angles. The coupled cone must also be checked.</p>
<div class="scroll"><table id="joints"></table></div></section>
<section><h2>Recorded timeline</h2>
<p><label>Body <select id="body"></select></label> <label>Joint <select id="joint"></select></label></p>
<div class="grid"><div><p>Body error: <span style="color:#1674c5">position cm</span> / <span style="color:#bd590a">movement cm per step</span></p><canvas id="bodyPlot"></canvas></div>
<div><p>Joint error: <span style="color:#1674c5">raw &rarr; bounded</span> / <span style="color:#bd590a">bounded &rarr; actual</span> (deg)</p><canvas id="jointPlot"></canvas></div></div>
<p><input id="frame" type="range" min="0" value="0"><b id="time"></b></p>
<p id="position"></p><div class="scroll"><table id="angles"></table></div>
<p id="range" class="muted"></p><p id="singular" class="bad"></p>
<div class="grid"><div><p>Reference / actual trajectory, side view (Z / Y metres)</p><canvas id="trajectory"></canvas></div>
<div><p>Selected axis angle (deg): <select id="axis"><option value="0">Twist X</option><option value="1">Swing Y</option><option value="2">Swing Z</option></select></p><canvas id="axisPlot"></canvas>
<span class="legend">Blue: raw &middot; green: bounded &middot; orange: actual &middot; grey: configured axis bounds (cone is coupled).</span></div></div>
</section>
<section><h2>Limit exposure</h2><p class="muted">Per-axis rows include Locked axes. Free axes have no excess.
Cone excess is radial distance in swing-angle coordinates, in degrees; it is not a shortest rotation.
Violation threshold: 0.0001 rad. Durations are sampled-frame counts &times; fixed dt. Negative first/last times mean no violation.</p>
<div class="scroll"><table id="violations"></table></div></section>
<section><h2>Captured settings</h2><p class="muted">The snapshot used at start; later editor changes do not affect this run.
Raw samples remain embedded in this standalone file. All reported positions use metres internally, angles use radians.</p>
<details><summary>Body masses/shapes and joint frames/limits/drives</summary><pre id="settings"></pre></details></section>
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
            out << (i ? "," : "") << "{\"t\":" << frame.time << ",\"clip\":" << frame.clipTime << ",\"b\":[";
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
        out << R"HTML(]};
const $=id=>document.getElementById(id), deg=180/Math.PI, fmt=v=>Number(v).toFixed(3), axes=["Twist X","Swing Y","Swing Z","Swing cone"];
function table(id,headers,rows){const t=$(id);t.replaceChildren();const head=t.createTHead().insertRow();headers.forEach(s=>{const c=document.createElement("th");c.textContent=s;head.append(c)});
const body=t.createTBody();rows.forEach(row=>{const tr=body.insertRow();row.forEach(s=>{const c=tr.insertCell();c.textContent=typeof s==="number"?fmt(s):s})})}
function stats(values){let sum=0,sq=0,peak=-1,at=0;values.forEach((v,i)=>{sum+=v;sq+=v*v;if(v>peak){peak=v;at=data.frames[i].t}});
return {mean:sum/values.length,rms:Math.sqrt(sq/values.length),peak,at}}
function exposure(values){let count=0,longest=0,run=0,first=-1,last=-1;values.forEach((v,i)=>{if(v>1e-4){if(!count)first=data.frames[i].t;last=data.frames[i].t;count++;longest=Math.max(longest,++run)}else run=0});
return {...stats(values.map(v=>v*deg)),percent:100*count/values.length,duration:count*data.step,longest:longest*data.step,first,last}}
function download(name,text,type){const url=URL.createObjectURL(new Blob([text],{type})),a=document.createElement("a");a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)}
$("description").textContent=data.description;
$("settings").textContent=JSON.stringify({step:data.step,bodies:data.bodies,joints:data.joints},null,2);
data.bodies.forEach((b,i)=>$("body").add(new Option(b.name,i)));
data.joints.forEach((j,i)=>$("joint").add(new Option(j.parent+" -> "+j.name,i)));
$("frame").max=data.frames.length-1;
const br=data.bodies.map((b,i)=>{const p=stats(data.frames.map(f=>f.b[i].p*100)),m=stats(data.frames.map(f=>f.b[i].m*100)),r=stats(data.frames.map(f=>f.b[i].q*deg));
return [b.name,p.mean,p.rms,p.peak,p.at,m.mean,m.peak,r.mean,r.peak]});
table("bodies",["Body","Mean cm","RMS cm","Peak cm","Peak at s","Move mean cm/step","Move peak cm/step","Rot mean deg","Rot peak deg"],br.sort((a,b)=>b[3]-a[3]));
const jr=data.joints.map((j,i)=>{const c=stats(data.frames.map(f=>f.j[i].c*deg)),k=stats(data.frames.map(f=>f.j[i].k*deg)),t=stats(data.frames.map(f=>f.j[i].e*deg));
return [j.name,c.mean,c.peak,c.at,k.mean,k.peak,t.peak,...axes.map((_,a)=>exposure(data.frames.map(f=>f.j[i].r.e[a])).percent)]});
table("joints",["Joint","Clamp mean deg","Clamp peak deg","Clamp peak s","Track mean deg","Track peak deg","Total peak deg","Twist bad %","Y bad %","Z bad %","Cone bad %"],jr.sort((a,b)=>b[2]-a[2]));
function chart(id,series,selected,xlabel="Time (s)",bounds=[]){const c=$(id),ctx=c.getContext("2d"),w=c.width=1100,h=c.height=250;
const all=series.flatMap(s=>s.points),xs=all.map(p=>p[0]),ys=all.map(p=>p[1]).concat(bounds);
let xmin=Math.min(...xs),xmax=Math.max(...xs),ymin=Math.min(0,...ys),ymax=Math.max(...ys);if(xmax-xmin<1e-6)xmax=xmin+1;if(ymax-ymin<1e-6)ymax=ymin+1;
const x=v=>60+(v-xmin)/(xmax-xmin)*(w-90),y=v=>h-35-(v-ymin)/(ymax-ymin)*(h-65);
ctx.clearRect(0,0,w,h);ctx.font="14px system-ui";ctx.fillStyle="#52647b";ctx.fillText(fmt(ymax),3,23);ctx.fillText(fmt(ymin),3,h-32);ctx.fillText(fmt(xmin),60,h-9);ctx.fillText(fmt(xmax),w-80,h-9);ctx.fillText(xlabel,w/2,h-9);
ctx.strokeStyle="#c4ceda";ctx.beginPath();ctx.moveTo(60,20);ctx.lineTo(60,h-35);ctx.lineTo(w-30,h-35);ctx.stroke();
bounds.forEach(v=>{ctx.setLineDash([6,5]);ctx.beginPath();ctx.moveTo(60,y(v));ctx.lineTo(w-30,y(v));ctx.stroke();ctx.setLineDash([])});
series.forEach(s=>{ctx.strokeStyle=s.color;ctx.lineWidth=2;ctx.beginPath();s.points.forEach((p,i)=>{if(i)ctx.lineTo(x(p[0]),y(p[1]));else ctx.moveTo(x(p[0]),y(p[1]))});ctx.stroke()});
if(selected!==null){ctx.strokeStyle="#263b52";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x(selected),20);ctx.lineTo(x(selected),h-35);ctx.stroke()}}
function refresh(){const bi=+$("body").value,ji=+$("joint").value,fi=+$("frame").value,f=data.frames[fi],b=f.b[bi],a=+$("axis").value;
$("time").textContent=`Sample ${fi+1} | t ${fmt(f.t)} s | clip ${fmt(f.clip)} s`;
$("position").textContent=`${data.bodies[bi].name} | reference xyz m: ${b.r.map(fmt).join(", ")} | actual xyz m: ${b.a.map(fmt).join(", ")} | diff cm: ${b.d.map(v=>fmt(v*100)).join(", ")} | movement diff cm/step: ${b.v.map(v=>fmt(v*100)).join(", ")}`;
chart("bodyPlot",[{color:"#1674c5",points:data.frames.map(f=>[f.t,f.b[bi].p*100])},{color:"#bd590a",points:data.frames.map(f=>[f.t,f.b[bi].m*100])}],f.t);
chart("trajectory",[{color:"#1674c5",points:data.frames.map(f=>[f.b[bi].r[2],f.b[bi].r[1]])},{color:"#bd590a",points:data.frames.map(f=>[f.b[bi].a[2],f.b[bi].a[1]])}],null,"Z (m)");
if(!data.joints.length)return;const j=f.j[ji],cfg=data.joints[ji];
chart("jointPlot",[{color:"#1674c5",points:data.frames.map(f=>[f.t,f.j[ji].c*deg])},{color:"#bd590a",points:data.frames.map(f=>[f.t,f.j[ji].k*deg])}],f.t);
const bounds=cfg.motion[a]===2?[]:cfg.motion[a]===0?[0]:[cfg.min[a]*deg,cfg.max[a]*deg];
chart("axisPlot",[{color:"#1674c5",points:data.frames.map(f=>[f.t,f.j[ji].r.a[a]*deg])},{color:"#27815d",points:data.frames.map(f=>[f.t,f.j[ji].b.a[a]*deg])},{color:"#bd590a",points:data.frames.map(f=>[f.t,f.j[ji].a.a[a]*deg])}],f.t,"Time (s)",bounds);
const rows=axes.map((name,a)=>[name,a===3?(cfg.motion[1]===1&&cfg.motion[2]===1?"Coupled ellipse":"Not applied"):cfg.motion[a]===2?"Free":cfg.motion[a]===0?"Locked":`${fmt(cfg.min[a]*deg)} .. ${fmt(cfg.max[a]*deg)}`,
...(a<3?[j.r.a[a]*deg,j.b.a[a]*deg,j.a.a[a]*deg]:["-","-","-"]),j.r.e[a]*deg,j.a.e[a]*deg]);
table("angles",["Axis","Allowed deg","Raw deg","Bounded deg","Actual deg","Raw excess deg","Actual excess deg"],rows);
table("violations",["Axis","Bad %","Total s","Longest s","First s","Last s","Peak excess deg","Peak at s","Actual peak excess deg"],axes.map((name,a)=>{
const e=exposure(data.frames.map(f=>f.j[ji].r.e[a])),actual=exposure(data.frames.map(f=>f.j[ji].a.e[a]));return [name,e.percent,e.duration,e.longest,e.first,e.last,e.peak,e.at,actual.peak]}));
$("range").textContent=axes.slice(0,3).map((name,a)=>{const v=data.frames.map(f=>f.j[ji].r.a[a]*deg);return `${name} raw range: ${fmt(Math.min(...v))} .. ${fmt(Math.max(...v))} deg`}).join(" | ");
const singular=data.frames.filter(f=>f.j[ji].r.singular).length;$("singular").textContent=singular?`Warning: ${singular} samples have ambiguous twist near 180-degree swing. Inspect joint frames.`:"";}
["body","joint","frame","axis"].forEach(id=>$(id).addEventListener("input",refresh));
$("csv").onclick=()=>{const rows=[["type","name","time_s","clip_time_s","ref_x_m","ref_y_m","ref_z_m","actual_x_m","actual_y_m","actual_z_m","diff_x_m","diff_y_m","diff_z_m","move_diff_x_m","move_diff_y_m","move_diff_z_m","position_m","movement_m_step","rotation_rad","clamp_rad","tracking_rad","total_rad","raw_twist_rad","raw_y_rad","raw_z_rad","bounded_twist_rad","bounded_y_rad","bounded_z_rad","actual_twist_rad","actual_y_rad","actual_z_rad","twist_excess_rad","y_excess_rad","z_excess_rad","cone_excess_rad"]];
data.frames.forEach(f=>{f.b.forEach((b,i)=>rows.push(["body",data.bodies[i].name,f.t,f.clip,...b.r,...b.a,...b.d,...b.v,b.p,b.m,b.q,...Array(16).fill("")]));
f.j.forEach((j,i)=>rows.push(["joint",data.joints[i].name,f.t,f.clip,...Array(15).fill(""),j.c,j.k,j.e,...j.r.a,...j.b.a,...j.a.a,...j.r.e]))});
const quote=v=>'"'+String(v).replace(/"/g,'""')+'"';download("ragdoll-comparison.csv","\ufeff"+rows.map(r=>r.map(quote).join(",")).join("\r\n"),"text/csv;charset=utf-8")};
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
