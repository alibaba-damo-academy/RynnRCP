#!/usr/bin/env python3
"""Browser configuration and calibration wizard for LeKiwi."""

from __future__ import annotations

import argparse
import base64
import glob
import json
import logging
import os
import re
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import parse_qs, urlparse

import yaml

from rynnrcp.utils.device_identity import machine_mac_suffix, with_machine_suffix
from rynnrcp.utils.web_urls import browser_urls, primary_browser_url


LOGGER = logging.getLogger("rynnrcp.lekiwi.configure_web")
PACKAGE_DIR = Path(__file__).resolve().parent
CONFIG_DIR = PACKAGE_DIR / "config"
CONFIG_FILES = {
    "follower_server": CONFIG_DIR / "lekiwi_server.yaml",
    "leader_server": CONFIG_DIR / "lekiwi_leader_server.yaml",
    "rynnbot_app": CONFIG_DIR / "lekiwi_rynnbot_app.yaml",
}
HARDWARE_LOCK = threading.Lock()
CALIBRATION_JOBS: dict[str, "CalibrationJob"] = {}


HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LeKiwi 配置向导 - RynnRCP</title><style>
*{box-sizing:border-box}body{margin:0;min-height:100vh;color:#e8e8e8;background:linear-gradient(135deg,#1a1a2e,#16213e);font:14px/1.55 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
.container{max-width:1200px;margin:auto;padding:20px}header{position:relative;text-align:center;padding:20px 0;border-bottom:1px solid #333;margin-bottom:20px}h1{margin:0;font-size:28px;background:linear-gradient(90deg,#00d4ff,#00ff88);-webkit-background-clip:text;-webkit-text-fill-color:transparent}header p{color:#9aa4b2}
.tabs{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:20px}.tab{padding:12px 18px;background:#2a2a3e;border:1px solid transparent;border-radius:8px;color:#9aa4b2;cursor:pointer;font-weight:650}.tab.active{background:linear-gradient(135deg,#00d4ff,#0099cc);color:white}.tab.done{background:#1e3a2e;border-color:#00ff88;color:#00ff88}
.section{display:none;background:#1e1e2f;border:1px solid #303044;border-radius:12px;padding:25px;box-shadow:0 4px 20px #0005}.section.active{display:block}.title{font-size:20px;color:#00d4ff;margin:0 0 18px;padding-bottom:10px;border-bottom:1px solid #333}.tip{background:#24364d;border-left:4px solid #00d4ff;padding:14px 15px;border-radius:0 8px 8px 0;margin-bottom:20px;color:#b8c0cc}.tip strong{display:block;color:#00d4ff;margin-bottom:5px}
.grid{display:grid;grid-template-columns:repeat(2,minmax(260px,1fr));gap:16px}.group{margin-bottom:15px}label{display:block;color:#a6afbd;font-size:13px;margin-bottom:7px;font-weight:650}input,select{width:100%;padding:12px 14px;border-radius:8px;border:1px solid #44485a;background:#2a2a3e;color:#e8e8e8;font:inherit}.buttons{display:flex;gap:10px;flex-wrap:wrap;margin-top:20px}button{padding:11px 18px;border:0;border-radius:8px;font:inherit;font-weight:700;cursor:pointer}button:disabled{opacity:.45;cursor:not-allowed}.primary{background:linear-gradient(135deg,#00d4ff,#0099cc);color:white}.success{background:linear-gradient(135deg,#00ff88,#00cc6a);color:#101828}.warning{background:linear-gradient(135deg,#ffa500,#cc8400);color:#101828}.danger{background:linear-gradient(135deg,#ff4757,#cc3a47);color:white}.plain{background:#555;color:white}
.mapping,.status{background:#2a2a3e;border:1px solid #38384d;border-radius:8px;padding:15px;margin-top:15px}.row{display:flex;justify-content:space-between;gap:14px;padding:9px 0;border-bottom:1px solid #3a3a4e}.row:last-child{border:0}.key{color:#00d4ff;font-weight:700}.value{color:#00ff88;font-weight:700;overflow-wrap:anywhere}.unset{color:#ff4757}
.camera-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:15px;margin-top:20px}.camera{background:#2a2a3e;border-radius:8px;overflow:hidden;border:2px solid transparent}.camera.selected{border-color:#00ff88}.camera img{width:100%;aspect-ratio:16/9;object-fit:cover;background:#101828;display:block}.camera-info{padding:12px}.camera-actions{display:flex;gap:8px;margin-top:10px}.camera-actions button{padding:8px 11px;font-size:12px}
.serial-list{margin-top:15px}.serial{display:flex;justify-content:space-between;align-items:center;gap:14px;padding:12px 15px;background:#2a2a3e;border-radius:8px;margin-bottom:10px;border-left:4px solid transparent}.serial.new{border-left-color:#00ff88}.device{font-weight:750;overflow-wrap:anywhere}.desc{font-size:12px;color:#9aa4b2;overflow-wrap:anywhere}
.steps{margin-top:18px}.step{display:flex;gap:14px;padding:13px;background:#2a2a3e;border-radius:8px;margin-bottom:9px}.num{width:32px;height:32px;border-radius:50%;background:#3a3a4e;display:grid;place-items:center;font-weight:800;flex:none}.step.active .num{background:#00d4ff;color:#101828}.step.done .num{background:#00ff88;color:#101828}.step-title{font-weight:750}.step-desc{color:#9aa4b2;font-size:13px}.hint{margin-top:10px;padding:10px 12px;border:1px solid #3a331e;border-radius:8px;background:#241f13;color:#ffd166;font-weight:700}.hint.ready{border-color:#1e3a2e;background:#10251d;color:#00ff88}
table{width:100%;border-collapse:collapse;margin-top:15px}th,td{border-bottom:1px solid #3a3a4e;padding:8px;text-align:right;font-family:SFMono-Regular,Consolas,monospace}th:first-child,td:first-child{text-align:left}th{color:#00d4ff}.log{background:#0d0d1a;border-radius:8px;padding:14px;margin-top:15px;max-height:260px;overflow:auto;font-family:SFMono-Regular,Consolas,monospace}.log div{white-space:pre-wrap;color:#00d4ff}.log .error{color:#ff4757}.log .warning{color:#ffa500}
.toast{position:fixed;right:20px;bottom:20px;max-width:calc(100vw - 40px);padding:14px 20px;border-radius:8px;color:white;font-weight:700;transform:translateX(150%);transition:.25s;z-index:10}.toast.show{transform:none}.toast.success{background:#00a86b}.toast.error{background:#ff4757}.toast.info{background:#0099cc}.exit{position:absolute;right:0;top:20px}
@media(max-width:760px){.container{padding:14px}header{text-align:left;padding-right:90px}.grid{grid-template-columns:1fr}.tab{flex:1 1 150px}.serial,.row{align-items:flex-start;flex-direction:column}}
</style></head><body><div class="container"><header><h1>LeKiwi 配置向导</h1><p>按步骤完成设备配置、连接检查和标定</p><button class="danger exit" onclick="shutdown()">退出</button></header>
<nav class="tabs">
<button class="tab active" id="tab-device" onclick="show('device')">1. 设备设置</button><button class="tab" id="tab-camera" onclick="show('camera')">2. 相机配置</button><button class="tab" id="tab-follower-serial" onclick="show('follower-serial')">3. 从臂串口</button><button class="tab" id="tab-follower-calib" onclick="show('follower-calib')">4. 从臂标定</button><button class="tab" id="tab-leader-serial" onclick="show('leader-serial')">5. 主臂串口</button><button class="tab" id="tab-leader-calib" onclick="show('leader-calib')">6. 主臂标定</button></nav>

<section class="section active" id="section-device"><h2 class="title">机器人身份与云端连接</h2><div class="tip"><strong>先按使用方式判断</strong><b>本地 Teleop、MCP、Protocol Debug：</b>保留两个 Robot ID，然后继续相机配置。<br><b>让 LeKiwi 从臂接入 RynnBot：</b>填写下面一套从臂执行端凭据。主臂由本地 Teleop App 连接，页面仅提供从臂云端凭据。</div><div class="tip"><strong>Robot ID 与 App ID 的区别</strong>从臂和主臂 Robot ID 是两个本地 RCP Server 的身份，请为二者设置不同值。App ID 是从臂 RynnBot App 的标识，由配置工具自动生成。</div><div class="grid">
<div class="group"><label>从臂 Robot ID（必填，默认值可直接使用）</label><input id="follower-id"></div><div class="group"><label>主臂 Robot ID（必填，默认值可直接使用）</label><input id="leader-id"></div><div class="group"><label>从臂 RynnBot App ID（自动生成）</label><input id="app-id" readonly></div><div class="group"><label>从臂 Product Key</label><input id="product-key" placeholder="从臂接入 RynnBot 时填写"></div><div class="group"><label>从臂 Device Name</label><input id="device-name" placeholder="从臂接入 RynnBot 时填写"></div><div class="group"><label>从臂 Device Secret</label><input id="device-secret" placeholder="从臂接入 RynnBot 时填写"></div><div class="group"><label>从臂 HTTP URL（通常保持默认）</label><input id="http-url"></div><div class="group"><label>云端图片上传编码</label><select id="image-codec"><option>jpeg</option><option>npy_gzip</option></select></div></div><div class="buttons"><button class="success" onclick="saveDevice()">保存设置</button><button class="primary" onclick="show('camera')">下一步</button></div></section>

<section class="section" id="section-camera"><h2 class="title">相机配置</h2><div class="tip"><strong>绑定两路画面</strong>扫描后根据预览画面绑定前置和腕部相机。RCP 输出固定为 640 × 360 JPEG、30 FPS。</div><div class="buttons"><button class="primary" onclick="scanCameras()">扫描相机</button><button class="warning" onclick="clearCameras()">清除绑定</button></div><div class="mapping"><div class="row"><span class="key">observation.front.image</span><span class="value" id="map-front"></span></div><div class="row"><span class="key">observation.wrist.image</span><span class="value" id="map-wrist"></span></div></div><div class="camera-grid" id="camera-grid"><div class="desc">点击“扫描相机”查看画面。</div></div><div class="buttons"><button class="success" id="save-camera" onclick="saveCameras()">保存配置</button><button class="primary" onclick="show('follower-serial')">下一步</button></div></section>

<section class="section" id="section-follower-serial"><h2 class="title">从臂串口配置</h2><div class="tip"><strong>一条总线，九个电机</strong>从臂机械臂 ID 1–6 与底盘轮 ID 7–9 使用同一个串口。先记录拔出设备时的基线，再插入设备并检测新增串口。</div><div class="buttons"><button class="warning" onclick="baseline('follower')">记录基线</button><button class="primary" onclick="scanSerial('follower',true)">检测新增</button><button class="plain" onclick="scanSerial('follower',false)">刷新列表</button></div><div class="status" id="follower-serial-status">等待扫描。</div><div class="serial-list" id="follower-serial-list"></div><div class="mapping"><div class="row"><span class="key">从臂串口</span><span class="value" id="follower-selected"></span></div></div><div class="buttons"><button class="success" onclick="savePort('follower')">保存并检查 1–9 号电机</button><button class="primary" onclick="show('follower-calib')">下一步</button></div></section>

<section class="section" id="section-follower-calib"><h2 class="title">从臂标定</h2><div class="tip"><strong>标定六轴机械臂</strong>启动后将机械臂放在各关节中间位置；记录中位后，缓慢推动肩、肘和夹爪走完整范围。底盘轮保持架空和静止。</div><div id="follower-steps" class="steps"></div><div class="buttons"><button class="primary" onclick="calibStart('follower')">启动标定</button><button class="success" id="follower-enter" onclick="calibAdvance('follower')" disabled>记录中间位置</button><button class="danger" onclick="calibStop('follower')">停止</button><button class="primary" onclick="show('leader-serial')">下一步</button></div><div class="hint" id="follower-hint">点击“启动标定”。</div><div id="follower-ranges"></div><div class="log" id="follower-log"></div></section>

<section class="section" id="section-leader-serial"><h2 class="title">主臂串口配置</h2><div class="tip"><strong>主臂六个电机</strong>主臂使用独立串口，读取 ID 1–6。可使用拔插识别或直接选择列表中的串口。</div><div class="buttons"><button class="warning" onclick="baseline('leader')">记录基线</button><button class="primary" onclick="scanSerial('leader',true)">检测新增</button><button class="plain" onclick="scanSerial('leader',false)">刷新列表</button></div><div class="status" id="leader-serial-status">等待扫描。</div><div class="serial-list" id="leader-serial-list"></div><div class="mapping"><div class="row"><span class="key">主臂串口</span><span class="value" id="leader-selected"></span></div></div><div class="buttons"><button class="success" onclick="savePort('leader')">保存并检查 1–6 号电机</button><button class="primary" onclick="show('leader-calib')">下一步</button></div></section>

<section class="section" id="section-leader-calib"><h2 class="title">主臂标定</h2><div class="tip"><strong>标定六轴主臂</strong>流程与从臂相同。标定期间保持主臂 Server 和 Teleop App 停止。</div><div id="leader-steps" class="steps"></div><div class="buttons"><button class="primary" onclick="calibStart('leader')">启动标定</button><button class="success" id="leader-enter" onclick="calibAdvance('leader')" disabled>记录中间位置</button><button class="danger" onclick="calibStop('leader')">停止</button></div><div class="hint" id="leader-hint">点击“启动标定”。</div><div id="leader-ranges"></div><div class="log" id="leader-log"></div></section>
</div><div class="toast" id="toast"></div><script>
const $=id=>document.getElementById(id), arms=['follower','leader'];let config={},camera={front:null,wrist:null},foundCameras=[],selected={follower:'',leader:''},bases={follower:[],leader:[]},pollers={};
async function api(url,opt){const r=await fetch(url,opt),x=await r.json();if(!r.ok)throw Error(x.error||r.statusText);return x}function post(url,value={}){return api(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(value)})}function toast(text,type='success'){const e=$('toast');e.textContent=text;e.className='toast '+type+' show';setTimeout(()=>e.classList.remove('show'),3500)}function esc(s){const d=document.createElement('div');d.textContent=s??'';return d.innerHTML}
function show(name){document.querySelectorAll('.section').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));$('section-'+name).classList.add('active');$('tab-'+name).classList.add('active')}function done(name){$('tab-'+name).classList.add('done')}
async function load(){config=await api('/api/config');const s=config.server,h=config.hardware,r=config.rynnbot;$('follower-id').value=s.follower_id;$('leader-id').value=s.leader_id;$('app-id').value=r.app_id;$('product-key').value=r.product_key;$('device-name').value=r.device_name;$('device-secret').value=r.device_secret;$('http-url').value=r.http_url;$('image-codec').value=r.image_upload_codec;selected.follower=h.follower_port;selected.leader=h.leader_port;camera.front=h.front_camera;camera.wrist=h.wrist_camera;updateMappings();renderSteps('follower','idle');renderSteps('leader','idle')}
async function update(patch){config=await post('/api/config',patch);return config}async function saveDevice(){try{await update({server:{follower_id:$('follower-id').value,leader_id:$('leader-id').value},rynnbot:{app_id:$('app-id').value,product_key:$('product-key').value,device_name:$('device-name').value,device_secret:$('device-secret').value,http_url:$('http-url').value,image_upload_codec:$('image-codec').value}});done('device');toast('设置已保存；本地模式可直接使用 Robot ID')}catch(e){toast(e.message,'error')}}
function updateMappings(){$('map-front').textContent=camera.front===null?'(未绑定)':'Camera '+camera.front;$('map-wrist').textContent=camera.wrist===null?'(未绑定)':'Camera '+camera.wrist;arms.forEach(a=>$(a+'-selected').textContent=selected[a]||'(未选择)')}
async function scanCameras(){const grid=$('camera-grid');grid.innerHTML='<div class="desc">正在扫描相机…</div>';try{foundCameras=(await api('/api/cameras?max_index=10')).cameras;renderCameras()}catch(e){grid.textContent=e.message;toast(e.message,'error')}}function renderCameras(){const grid=$('camera-grid');grid.innerHTML='';if(!foundCameras.length)grid.innerHTML='<div class="desc">未检测到可读取画面的相机。</div>';foundCameras.forEach(c=>{const card=document.createElement('div');card.className='camera'+([camera.front,camera.wrist].includes(c.index)?' selected':'');card.innerHTML=`<img src="${c.image}" alt="Camera ${c.index}"><div class="camera-info"><strong>Camera ${c.index} (${c.width} × ${c.height})</strong><div class="camera-actions"><button class="primary" onclick="bindCamera(${c.index},'front')">绑定前置</button><button class="warning" onclick="bindCamera(${c.index},'wrist')">绑定腕部</button></div></div>`;grid.appendChild(card)})}function bindCamera(i,k){camera[k]=i;updateMappings();renderCameras()}function clearCameras(){camera={front:null,wrist:null};updateMappings();renderCameras()}async function saveCameras(){if(camera.front===null||camera.wrist===null)return toast('请先绑定两路相机','error');try{await update({hardware:{front_camera:camera.front,wrist_camera:camera.wrist}});done('camera');toast('相机配置已保存')}catch(e){toast(e.message,'error')}}
async function serials(){return (await api('/api/serial')).ports}async function baseline(a){bases[a]=(await serials()).map(p=>p.device);$(a+'-serial-status').textContent='已记录 '+bases[a].length+' 个串口；现在插入设备后点击“检测新增”。'}async function scanSerial(a,newOnly){try{const ports=await serials(),fresh=new Set(newOnly?ports.map(p=>p.device).filter(x=>!bases[a].includes(x)):[]);renderSerial(a,ports,fresh);$(a+'-serial-status').textContent=fresh.size?'检测到新增串口：'+[...fresh].join(', '):'已列出 '+ports.length+' 个串口。'}catch(e){toast(e.message,'error')}}function renderSerial(a,ports,fresh){const box=$(a+'-serial-list');box.innerHTML='';ports.forEach(p=>{const e=document.createElement('div');e.className='serial'+(fresh.has(p.device)?' new':'');e.innerHTML=`<div><div class="device">${esc(p.device)}</div><div class="desc">${esc(p.description||p.hwid)}</div></div><button class="primary">选择</button>`;e.querySelector('button').onclick=()=>{selected[a]=p.device;updateMappings()};box.appendChild(e)});if(!ports.length)box.innerHTML='<div class="desc">未检测到串口。</div>'}async function savePort(a){if(!selected[a])return toast('请选择串口','error');try{await update({hardware:{[a+'_port']:selected[a]}});$(a+'-serial-status').textContent='正在检查电机，请保持设备静止…';const x=await post('/api/check-bus',{arm:a,port:selected[a]});$(a+'-serial-status').textContent=x.message;done(a+'-serial');toast(x.message)}catch(e){$(a+'-serial-status').textContent=e.message;toast(e.message,'error')}}
function renderSteps(a,phase){const labels=[['连接设备','打开串口并读取电机'],['记录中位','将各关节放在中间位置'],['采集范围','推动关节走完整范围'],['保存完成','写入电机和标定文件']],states={starting:0,middle_position:1,recording_range:2,saving:3,saved:4,finished:4,failed:-1,stopped:-1,idle:-1};const n=states[phase]??-1;$(a+'-steps').innerHTML=labels.map((x,i)=>`<div class="step ${i<n?'done':i===n?'active':''}"><div class="num">${i+1}</div><div><div class="step-title">${x[0]}</div><div class="step-desc">${x[1]}</div></div></div>`).join('')}
async function calibStart(a){try{await post('/api/calibration/start',{arm:a});startPoll(a);toast((a==='follower'?'从臂':'主臂')+'标定已启动')}catch(e){toast(e.message,'error')}}async function calibAdvance(a){try{await post('/api/calibration/advance',{arm:a})}catch(e){toast(e.message,'error')}}async function calibStop(a){try{await post('/api/calibration/stop',{arm:a});await poll(a)}catch(e){toast(e.message,'error')}}function startPoll(a){clearInterval(pollers[a]);pollers[a]=setInterval(()=>poll(a),250);poll(a)}
async function poll(a){try{const x=await api('/api/calibration/status?arm='+a),p=x.phase;renderSteps(a,p);const enter=$(a+'-enter'),hint=$(a+'-hint');enter.disabled=!['middle_position','recording_range'].includes(p);enter.textContent=p==='recording_range'?'结束并保存':'记录中间位置';const hints={starting:'正在连接并读取电机…',middle_position:'把机械臂放到各关节中间位置，然后点击“记录中间位置”。',recording_range:'缓慢推动肩、肘和夹爪走完整范围，然后点击“结束并保存”。',saving:'正在写入标定，请等待。',saved:'标定文件已保存：'+(x.path||''),failed:'标定失败，请查看日志。',stopped:'标定已停止。',idle:'点击“启动标定”。'};hint.textContent=hints[p]||p;hint.className='hint '+(['middle_position','recording_range','saved'].includes(p)?'ready':'');renderRanges(a,x.ranges||{});$(a+'-log').innerHTML=(x.logs||[]).map(l=>`<div class="${esc(l.type)}">[${esc(l.time)}] ${esc(l.text)}</div>`).join('');if(!x.running&&['saved','failed','stopped'].includes(p)){clearInterval(pollers[a]);if(p==='saved'){done(a+'-calib');toast((a==='follower'?'从臂':'主臂')+'标定完成')}}}catch(e){clearInterval(pollers[a]);toast(e.message,'error')}}function renderRanges(a,r){const names=Object.keys(r);$(a+'-ranges').innerHTML=names.length?`<table><thead><tr><th>关节</th><th>Min</th><th>当前</th><th>Max</th></tr></thead><tbody>${names.map(n=>`<tr><td>${esc(n)}</td><td>${r[n].min}</td><td>${r[n].position}</td><td>${r[n].max}</td></tr>`).join('')}</tbody></table>`:''}
async function shutdown(){try{await post('/api/shutdown');document.body.innerHTML='<div class="container"><h1>配置工具已退出</h1><p>可以关闭此页面。</p></div>'}catch(e){toast(e.message,'error')}}load().catch(e=>toast(e.message,'error'));
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    server_version = "LeKiwiConfigure/2.0"

    def do_GET(self) -> None:
        try:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)
            if parsed.path == "/":
                self._send(200, HTML, "text/html; charset=utf-8")
            elif parsed.path == "/api/config":
                self._json(200, build_snapshot(load_all_configs()))
            elif parsed.path == "/api/serial":
                self._json(200, {"ok": True, "ports": scan_serial_ports()})
            elif parsed.path == "/api/cameras":
                self._json(200, scan_cameras(int(query.get("max_index", ["10"])[0])))
            elif parsed.path == "/api/calibration/status":
                self._json(200, calibration_job(query.get("arm", ["follower"])[0]).status())
            else:
                self._json(404, {"error": "not found"})
        except Exception as exc:
            LOGGER.exception("GET %s failed", self.path)
            self._json(500, {"error": str(exc)})

    def do_POST(self) -> None:
        try:
            data = self._body()
            if self.path == "/api/config":
                configs = load_all_configs()
                snapshot = normalize_snapshot(data, build_snapshot(configs))
                apply_snapshot(configs, snapshot)
                validate_configs(configs)
                save_all_configs(configs)
                self._json(200, snapshot)
            elif self.path == "/api/check-bus":
                self._json(200, {"message": check_bus(str(data.get("arm")), str(data.get("port") or ""))})
            elif self.path.startswith("/api/calibration/"):
                job = calibration_job(str(data.get("arm") or "follower"))
                operation = self.path.rsplit("/", 1)[-1]
                result = {"start": job.start, "advance": job.advance, "stop": job.stop}[operation]()
                self._json(200, result)
            elif self.path == "/api/shutdown":
                for job in CALIBRATION_JOBS.values():
                    job.stop()
                self._json(200, {"ok": True})
                threading.Thread(target=self.server.shutdown, daemon=True).start()
            else:
                self._json(404, {"error": "not found"})
        except Exception as exc:
            LOGGER.exception("POST %s failed", self.path)
            self._json(400, {"error": str(exc)})

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _body(self) -> dict[str, Any]:
        value = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
        if not isinstance(value, dict):
            raise ValueError("请求内容必须是 JSON 对象")
        return value

    def _json(self, status: int, value: Any) -> None:
        self._send(status, json.dumps(value, ensure_ascii=False), "application/json; charset=utf-8")

    def _send(self, status: int, value: str, content_type: str) -> None:
        body = value.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class CalibrationJob:
    """One browser-controlled LeKiwi calibration session."""

    def __init__(self, arm: str) -> None:
        self.arm = normalize_arm(arm)
        self._phase = "idle"
        self._ranges: dict[str, dict[str, int]] = {}
        self._logs: list[dict[str, str]] = []
        self._path = ""
        self._thread: threading.Thread | None = None
        self._advance = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.RLock()

    def start(self) -> dict[str, Any]:
        with self._lock:
            if self.running:
                raise RuntimeError(f"{self.arm} 标定正在运行")
            self._phase, self._ranges, self._logs, self._path = "starting", {}, [], ""
            self._advance.clear()
            self._stop.clear()
            self._append("info", "开始标定")
            self._thread = threading.Thread(target=self._run, daemon=True, name=f"lekiwi-calibrate-{self.arm}")
            self._thread.start()
            return self.status()

    def advance(self) -> dict[str, Any]:
        with self._lock:
            if not self.running or self._phase not in {"middle_position", "recording_range"}:
                raise RuntimeError("当前标定阶段不能继续")
            self._advance.set()
            return self.status()

    def stop(self) -> dict[str, Any]:
        self._stop.set()
        self._advance.set()
        return self.status()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {"arm": self.arm, "running": self.running, "phase": self._phase, "ranges": dict(self._ranges), "logs": list(self._logs), "path": self._path}

    def _run(self) -> None:
        device = None
        try:
            from lerobot_lekiwi import ARM_MOTORS, LeKiwi, LeKiwiLeader
            from lerobot_lekiwi.driver import MOTORS, LEADER_MOTORS
            from lerobot_lekiwi.feetech_bus import MotorCalibration, RESOLUTION

            config = build_snapshot(load_all_configs())
            hardware, server = config["hardware"], config["server"]
            if self.arm == "follower":
                device = LeKiwi(port=hardware["follower_port"], robot_id=server["follower_id"])
                motor_ids, all_names = MOTORS, list(MOTORS)
            else:
                device = LeKiwiLeader(port=hardware["leader_port"], robot_id=server["leader_id"])
                motor_ids, all_names = LEADER_MOTORS, list(LEADER_MOTORS)

            with HARDWARE_LOCK:
                if self._stop.is_set():
                    return
                bus = device.bus
                bus.connect()
                bus.disable_torque(retries=5)
                for name in ARM_MOTORS:
                    bus.write("Operating_Mode", name, 0)
                if self.arm == "follower":
                    from lerobot_lekiwi.kinematics import WHEEL_NAMES
                    for name in WHEEL_NAMES:
                        bus.write("Operating_Mode", name, 1)

                self._set_phase("middle_position", "串口已连接；请确认机械臂中间位置")
                while not self._stop.is_set() and not self._advance.wait(0.1):
                    self._sample(bus, list(ARM_MOTORS))
                self._advance.clear()
                if self._stop.is_set():
                    return
                homings = bus.set_half_turn_homings(ARM_MOTORS)
                if self.arm == "follower":
                    from lerobot_lekiwi.kinematics import WHEEL_NAMES
                    homings.update(dict.fromkeys(WHEEL_NAMES, 0))

                full_turn = {"arm_wrist_flex", "arm_wrist_roll"}
                measured = [name for name in ARM_MOTORS if name not in full_turn]
                positions = bus.sync_read("Present_Position", list(ARM_MOTORS))
                positions = {name: positions[name] for name in measured}
                minimums, maximums = dict(positions), dict(positions)
                self._set_phase("recording_range", "开始记录关节运动范围")
                while not self._stop.is_set() and not self._advance.wait(0.05):
                    current = bus.sync_read("Present_Position", list(ARM_MOTORS))
                    minimums = {name: min(minimums[name], current[name]) for name in measured}
                    maximums = {name: max(maximums[name], current[name]) for name in measured}
                    self._update_ranges(current, minimums, maximums, full_turn, RESOLUTION)
                self._advance.clear()
                if self._stop.is_set():
                    return
                unmoved = [name for name in measured if minimums[name] == maximums[name]]
                if unmoved:
                    raise ValueError("请重新推动以下关节走完整运动范围：" + ", ".join(unmoved))

                range_mins = {**minimums, **dict.fromkeys(full_turn, 0)}
                range_maxes = {**maximums, **dict.fromkeys(full_turn, RESOLUTION - 1)}
                if self.arm == "follower":
                    from lerobot_lekiwi.kinematics import WHEEL_NAMES
                    range_mins.update(dict.fromkeys(WHEEL_NAMES, 0))
                    range_maxes.update(dict.fromkeys(WHEEL_NAMES, RESOLUTION - 1))
                self._set_phase("saving", "正在写入标定")
                calibration = {
                    name: MotorCalibration(id=motor_ids[name], drive_mode=0, homing_offset=homings[name], range_min=range_mins[name], range_max=range_maxes[name])
                    for name in all_names
                }
                bus.write_calibration(calibration)
                device.calibration = calibration
                device._save_calibration()
                with self._lock:
                    self._path = str(device.calibration_path)
                self._set_phase("saved", f"标定文件已保存：{device.calibration_path}")
        except Exception as exc:
            LOGGER.exception("%s calibration failed", self.arm)
            self._set_phase("failed", f"{type(exc).__name__}: {exc}", "error")
        finally:
            if device is not None:
                try:
                    device.disconnect()
                except Exception as exc:
                    self._append("warning", f"关闭串口时忽略错误：{exc}")
            if self._stop.is_set() and self._phase not in {"saved", "failed"}:
                self._set_phase("stopped", "标定已停止", "warning")

    def _sample(self, bus: Any, names: list[str]) -> None:
        positions = bus.sync_read("Present_Position", names)
        with self._lock:
            self._ranges = {name: {"min": value, "position": value, "max": value} for name, value in positions.items()}

    def _update_ranges(self, current: Mapping[str, int], minimums: Mapping[str, int], maximums: Mapping[str, int], full_turn: set[str], resolution: int) -> None:
        with self._lock:
            self._ranges = {
                name: {"min": 0 if name in full_turn else int(minimums[name]), "position": int(value), "max": resolution - 1 if name in full_turn else int(maximums[name])}
                for name, value in current.items()
            }

    def _set_phase(self, phase: str, message: str, level: str = "info") -> None:
        with self._lock:
            self._phase = phase
        self._append(level, message)

    def _append(self, level: str, text: str) -> None:
        with self._lock:
            self._logs.append({"time": time.strftime("%H:%M:%S"), "type": level, "text": text})
            self._logs = self._logs[-100:]


def calibration_job(arm: str) -> CalibrationJob:
    normalized = normalize_arm(arm)
    if normalized not in CALIBRATION_JOBS:
        CALIBRATION_JOBS[normalized] = CalibrationJob(normalized)
    return CALIBRATION_JOBS[normalized]


def normalize_arm(arm: str) -> str:
    value = str(arm).strip().lower()
    if value not in {"follower", "leader"}:
        raise ValueError("arm 必须是 follower 或 leader")
    return value


def check_bus(arm: str, port: str) -> str:
    from lerobot_lekiwi.driver import LEADER_MOTORS, MOTORS
    from lerobot_lekiwi.feetech_bus import FeetechBus

    normalized = normalize_arm(arm)
    if not port.strip():
        raise ValueError("请选择串口")
    motors = MOTORS if normalized == "follower" else LEADER_MOTORS
    with HARDWARE_LOCK:
        bus = FeetechBus(port, motors)
        try:
            bus.connect()
        finally:
            bus.disconnect(disable_torque=False)
    label = "从臂" if normalized == "follower" else "主臂"
    return f"{label}串口检查通过：{port}，检测到 {len(motors)} 个电机"


def load_all_configs() -> dict[str, dict[str, Any]]:
    return {name: load_yaml(path) for name, path in CONFIG_FILES.items()}


def load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise ValueError(f"配置必须是 YAML 对象：{path}")
    return value


def build_snapshot(configs: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    follower, leader, rynnbot = configs["follower_server"], configs["leader_server"], configs["rynnbot_app"]
    fc, lc, app = follower["components"], leader["components"], rynnbot["app"]
    suffix = machine_mac_suffix()
    return {
        "server": {
            "follower_id": with_machine_suffix(follower["manifest"]["robot_id"], "lekiwi_follower", suffix),
            "leader_id": with_machine_suffix(leader["manifest"]["robot_id"], "lekiwi_leader", suffix),
        },
        "hardware": {"follower_port": fc["robot"]["port"], "leader_port": lc["robot"]["port"], "front_camera": fc["front_camera"]["device_id"], "wrist_camera": fc["wrist_camera"]["device_id"], "front_rotate": fc["front_camera"]["rotate"], "wrist_rotate": fc["wrist_camera"]["rotate"]},
        "rynnbot": {
            "app_id": with_machine_suffix(app.get("app_id"), "lekiwi_rynnbot_app", suffix),
            **{key: app.get(key, "") for key in ("product_key", "device_name", "device_secret", "http_url", "image_upload_codec")},
        },
    }


def normalize_snapshot(patch: Mapping[str, Any], current: Mapping[str, Any]) -> dict[str, Any]:
    merged = deep_merge(current, patch)
    server, hardware, rynnbot = merged["server"], merged["hardware"], merged["rynnbot"]
    suffix = machine_mac_suffix()
    result = {
        "server": {
            "follower_id": with_machine_suffix(server.get("follower_id"), "lekiwi_follower", suffix),
            "leader_id": with_machine_suffix(server.get("leader_id"), "lekiwi_leader", suffix),
        },
        "hardware": {"follower_port": required(hardware.get("follower_port"), "从臂串口"), "leader_port": required(hardware.get("leader_port"), "主臂串口"), "front_camera": nonnegative(hardware.get("front_camera"), "前置相机"), "wrist_camera": nonnegative(hardware.get("wrist_camera"), "腕部相机"), "front_rotate": int(hardware.get("front_rotate", 180)), "wrist_rotate": int(hardware.get("wrist_rotate", 180))},
        "rynnbot": {key: str(rynnbot.get(key, "")).strip() for key in ("app_id", "product_key", "device_name", "device_secret", "http_url", "image_upload_codec")},
    }
    result["rynnbot"]["app_id"] = with_machine_suffix(
        result["rynnbot"]["app_id"], "lekiwi_rynnbot_app", suffix
    )
    if result["server"]["follower_id"] == result["server"]["leader_id"]:
        raise ValueError("从臂和主臂 Robot ID 必须不同")
    if result["hardware"]["follower_port"] == result["hardware"]["leader_port"]:
        raise ValueError("从臂和主臂必须选择不同串口")
    if result["hardware"]["front_camera"] == result["hardware"]["wrist_camera"]:
        raise ValueError("前置和腕部必须选择不同相机")
    if result["rynnbot"]["image_upload_codec"] not in {"jpeg", "npy_gzip"}:
        raise ValueError("图片上传编码必须是 jpeg 或 npy_gzip")
    return result


def apply_snapshot(configs: Mapping[str, dict[str, Any]], snapshot: Mapping[str, Any]) -> None:
    follower, leader, app = configs["follower_server"], configs["leader_server"], configs["rynnbot_app"]["app"]
    server, hardware, rynnbot = snapshot["server"], snapshot["hardware"], snapshot["rynnbot"]
    suffix = machine_mac_suffix()
    follower["manifest"]["robot_id"] = with_machine_suffix(server["follower_id"], "lekiwi_follower", suffix)
    leader["manifest"]["robot_id"] = with_machine_suffix(server["leader_id"], "lekiwi_leader", suffix)
    follower["components"]["robot"]["port"], leader["components"]["robot"]["port"] = hardware["follower_port"], hardware["leader_port"]
    for name in ("front", "wrist"):
        follower["components"][f"{name}_camera"]["device_id"] = hardware[f"{name}_camera"]
        follower["components"][f"{name}_camera"]["rotate"] = hardware[f"{name}_rotate"]
    app.update(rynnbot)
    _ensure_machine_app_id(configs["rynnbot_app"], "lekiwi_rynnbot_app", suffix)


def _ensure_machine_app_id(config: dict[str, Any], base_app_id: str, suffix: str) -> str:
    app = config.setdefault("app", {})
    app_id = with_machine_suffix(app.get("app_id"), base_app_id, suffix)
    app["app_id"] = app_id
    return app_id


def validate_configs(configs: Mapping[str, dict[str, Any]]) -> None:
    from rynnrcp.config.runtime_config import RuntimeConfig
    from rynnrcp.config.validator import validate_source

    for name in ("follower_server", "leader_server"):
        validate_source(configs[name])
    # Validate expanded configs before any file is replaced.
    temporary = []
    try:
        for name in ("follower_server", "leader_server"):
            path = CONFIG_FILES[name].with_suffix(".validate.yaml")
            path.write_text(yaml.safe_dump(configs[name], sort_keys=False, allow_unicode=True), encoding="utf-8")
            temporary.append(path)
            RuntimeConfig.load(str(path))
    finally:
        for path in temporary:
            path.unlink(missing_ok=True)


def save_all_configs(configs: Mapping[str, Mapping[str, Any]]) -> None:
    for name, path in CONFIG_FILES.items():
        path.write_text(yaml.safe_dump(dict(configs[name]), sort_keys=False, allow_unicode=True), encoding="utf-8")


def deep_merge(base: Mapping[str, Any], patch: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in patch.items():
        result[key] = deep_merge(result[key], value) if isinstance(value, Mapping) and isinstance(result.get(key), Mapping) else value
    return result


def required(value: Any, label: str) -> str:
    text = str(value or "").strip()
    if not text:
        raise ValueError(f"{label}不能为空")
    return text


def nonnegative(value: Any, label: str) -> int:
    number = int(value)
    if number < 0:
        raise ValueError(f"{label}索引必须大于等于 0")
    return number


def scan_serial_ports() -> list[dict[str, str]]:
    ports: list[dict[str, str]] = []
    try:
        from serial.tools import list_ports

        ports.extend({"device": str(p.device), "description": str(p.description or ""), "hwid": str(p.hwid or "")} for p in list_ports.comports())
    except Exception as exc:
        LOGGER.info("pyserial scan unavailable: %s", exc)
    known = {p["device"] for p in ports}
    for device in _fallback_serial_devices():
        if device not in known:
            ports.append({"device": device, "description": "serial device", "hwid": ""})
            known.add(device)
    return _dedupe_serial_ports(ports)


def _fallback_serial_devices() -> list[str]:
    devices: list[str] = []
    for pattern in (
        "/dev/tty.usbmodem*",
        "/dev/cu.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/cu.usbserial*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "/dev/ttyCH343USB*",
        "/dev/ttyCH34x*",
        "/dev/ttyWCHUSB*",
    ):
        devices.extend(sorted(glob.glob(pattern)))
    if os.name == "nt":
        devices.extend(_windows_serial_devices())
    return devices


def _windows_serial_devices() -> list[str]:
    """Return COM ports currently mapped by Windows."""
    try:
        import winreg
    except ImportError:
        return []

    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
    except OSError:
        return []

    devices: list[str] = []
    try:
        index = 0
        while True:
            try:
                _name, value, _value_type = winreg.EnumValue(key, index)
            except OSError:
                break
            device = str(value).strip()
            if re.fullmatch(r"COM\d+", device, flags=re.IGNORECASE):
                devices.append(device.upper())
            index += 1
    finally:
        winreg.CloseKey(key)
    return sorted(set(devices), key=_serial_device_sort_key)


def _dedupe_serial_ports(ports: list[dict[str, str]]) -> list[dict[str, str]]:
    selected: dict[str, dict[str, str]] = {}
    for port in ports:
        device = str(port.get("device") or "")
        if not device:
            continue
        if re.fullmatch(r"COM\d+", device, flags=re.IGNORECASE):
            device = device.upper()
        selected.setdefault(_serial_device_key(device), {**port, "device": device})
    return sorted(
        selected.values(),
        key=lambda item: _serial_device_sort_key(str(item.get("device") or "")),
    )


def _serial_device_key(device: str) -> str:
    name = device.rsplit("/", 1)[-1]
    if re.fullmatch(r"COM\d+", name, flags=re.IGNORECASE):
        return name.upper()
    for prefix in ("cu.", "tty."):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _serial_device_sort_key(device: str) -> tuple[int, int | str]:
    name = device.rsplit("/", 1)[-1]
    match = re.fullmatch(r"COM(\d+)", name, flags=re.IGNORECASE)
    if match:
        return (0, int(match.group(1)))
    return (1, device.casefold())


def scan_cameras(max_index: int) -> dict[str, Any]:
    import cv2
    cameras = []
    with HARDWARE_LOCK:
        for index in range(max(0, min(int(max_index), 20)) + 1):
            capture = cv2.VideoCapture(index)
            try:
                ok, frame = capture.read() if capture.isOpened() else (False, None)
            finally:
                capture.release()
            if not ok or frame is None:
                continue
            height, width = frame.shape[:2]
            scale = min(1.0, 360.0 / max(width, height))
            if scale < 1:
                frame = cv2.resize(frame, (int(width * scale), int(height * scale)))
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ok:
                cameras.append({"index": index, "width": int(width), "height": int(height), "image": "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")})
    return {"ok": True, "cameras": cameras}


def available_port(host: str, preferred: int) -> int:
    for port in range(preferred, preferred + 20):
        with socket.socket() as sock:
            try:
                sock.bind((host, port))
                return port
            except OSError:
                continue
    raise RuntimeError("找不到可用的 Web 配置端口")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Open the LeKiwi browser configuration and calibration wizard")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=28405)
    parser.add_argument("--no-open", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    port = available_port(args.host, args.port)
    server = ThreadingHTTPServer((args.host, port), Handler)
    urls = browser_urls(args.host, port)
    LOGGER.info("LeKiwi configuration UI Local: %s", urls[0])
    for url in urls[1:]:
        LOGGER.info("LeKiwi configuration UI LAN:   %s", url)
    if not args.no_open:
        threading.Timer(0.5, lambda: webbrowser.open(primary_browser_url(args.host, port))).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for job in CALIBRATION_JOBS.values():
            job.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
