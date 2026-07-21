"""Demand-driven HTTP visualization for RynnRCP state and actions.

The server intentionally has no sampling thread. Runtime data is read only
when a browser requests a snapshot, so an idle visualization endpoint costs
only an HTTP listener thread.
"""

from __future__ import annotations

import base64
import json
import logging
import threading
import time
from collections.abc import Mapping
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

from rynnrcp.utils.web_urls import browser_urls

logger = logging.getLogger(__name__)

DEFAULT_VISUALIZATION_HOST = "127.0.0.1"
DEFAULT_VISUALIZATION_PORT = 8092
PORT_SEARCH_LIMIT = 100
ACTION_CAPTURE_IDLE_SECONDS = 3.0


class VisualizationServer:
    """Serve a read-only browser dashboard backed by a local Runtime."""

    def __init__(
        self,
        runtime: Any,
        *,
        host: str = DEFAULT_VISUALIZATION_HOST,
        port: int = DEFAULT_VISUALIZATION_PORT,
    ):
        self._runtime = runtime
        self._host = str(host or DEFAULT_VISUALIZATION_HOST)
        self._preferred_port = int(port)
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._provider = RuntimeSnapshotProvider(runtime)

    @property
    def bound_port(self) -> int:
        return int(self._httpd.server_port) if self._httpd is not None else 0

    @property
    def url(self) -> str:
        return self.urls[0]

    @property
    def urls(self) -> list[str]:
        return browser_urls(self._host, self.bound_port)

    def start(self) -> None:
        if self._httpd is not None:
            raise RuntimeError("VisualizationServer is already started")
        handler = _handler_for(self._provider)
        self._httpd = _bind_http_server(self._host, self._preferred_port, handler)
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="rynnrcp-visualization",
            daemon=True,
        )
        try:
            self._thread.start()
        except Exception:
            self._httpd.server_close()
            self._httpd = None
            self._thread = None
            raise

    def stop(self) -> None:
        httpd = self._httpd
        thread = self._thread
        self._httpd = None
        self._thread = None
        if httpd is None:
            return
        httpd.shutdown()
        self._provider.stop()
        httpd.server_close()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)


class RuntimeSnapshotProvider:
    """Read protocol-shaped snapshots directly from a Runtime ToolBus."""

    def __init__(self, runtime: Any):
        self._runtime = runtime
        self._manifest: dict[str, Any] | None = None
        self._lock = threading.RLock()
        self._last_snapshot_access = 0.0
        self._capture_monitor: threading.Thread | None = None
        self._capture_stop = threading.Event()

    def schema(self) -> dict[str, Any]:
        with self._lock:
            if self._manifest is None:
                self._manifest = _as_dict(_unwrap(self._call("get_manifest")))
            return self._manifest

    def snapshot(self, names: list[str]) -> dict[str, Any]:
        self._touch_action_capture()
        manifest = self.schema()
        available = {
            str(item.get("name"))
            for item in manifest.get("observations", [])
            if isinstance(item, Mapping) and item.get("name")
        }
        requested = [name for name in names if name in available]
        observations: list[Any] = []
        error = ""
        if requested:
            try:
                payload = _unwrap(self._call("get_observations", names=requested))
                if isinstance(payload, Mapping):
                    observations = list(payload.get("observations") or [])
            except Exception as exc:
                error = str(exc)
        return {
            "observations": observations,
            "actions": self._latest_actions(),
            "error": error,
        }

    def stop(self) -> None:
        self._capture_stop.set()
        monitor = self._capture_monitor
        self._set_action_capture(False)
        if monitor is not None and monitor is not threading.current_thread():
            monitor.join(timeout=1.5)

    def _call(self, name: str, **kwargs: Any) -> Any:
        bus = getattr(self._runtime, "bus", None)
        if bus is None:
            raise RuntimeError("runtime ToolBus is unavailable")
        return bus.call_tool(name, **kwargs)

    def _latest_actions(self) -> dict[str, Any]:
        service = self._action_service()
        latest = getattr(service, "latest_actions", {}) if service is not None else {}
        return dict(latest) if isinstance(latest, Mapping) else {}

    def _action_service(self) -> Any | None:
        services = getattr(self._runtime, "services", {})
        if not isinstance(services, Mapping):
            return None
        return services.get("action_service")

    def _touch_action_capture(self) -> None:
        service = self._action_service()
        if service is None or not callable(getattr(service, "set_latest_action_capture", None)):
            return
        with self._lock:
            self._last_snapshot_access = time.monotonic()
            if self._capture_monitor is None or not self._capture_monitor.is_alive():
                service.set_latest_action_capture(True)
                self._capture_stop.clear()
                self._capture_monitor = threading.Thread(
                    target=self._monitor_action_capture,
                    name="rynnrcp-visualization-idle",
                    daemon=True,
                )
                self._capture_monitor.start()

    def _monitor_action_capture(self) -> None:
        while not self._capture_stop.wait(1.0):
            with self._lock:
                if time.monotonic() - self._last_snapshot_access < ACTION_CAPTURE_IDLE_SECONDS:
                    continue
                self._set_action_capture(False)
                self._capture_monitor = None
                return

    def _set_action_capture(self, enabled: bool) -> None:
        service = self._action_service()
        setter = getattr(service, "set_latest_action_capture", None) if service is not None else None
        if callable(setter):
            setter(enabled)


def _bind_http_server(host: str, preferred_port: int, handler: type[BaseHTTPRequestHandler]) -> ThreadingHTTPServer:
    if preferred_port < 0 or preferred_port > 65535:
        raise ValueError("visualization port must be between 0 and 65535")
    first_error: OSError | None = None
    ports = (
        [preferred_port]
        if preferred_port == 0
        else range(preferred_port, min(65535, preferred_port + PORT_SEARCH_LIMIT) + 1)
    )
    for port in ports:
        try:
            httpd = ThreadingHTTPServer((host, int(port)), handler)
            if int(port) != preferred_port:
                logger.warning(
                    "Visualization port %d is unavailable; using %d instead",
                    preferred_port,
                    httpd.server_port,
                )
            return httpd
        except OSError as exc:
            if first_error is None:
                first_error = exc
    if preferred_port != 0:
        try:
            httpd = ThreadingHTTPServer((host, 0), handler)
            logger.warning(
                "Visualization ports %d-%d are unavailable; using %d instead",
                preferred_port,
                min(65535, preferred_port + PORT_SEARCH_LIMIT),
                httpd.server_port,
            )
            return httpd
        except OSError:
            pass
    assert first_error is not None
    raise first_error


def _handler_for(provider: RuntimeSnapshotProvider) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self._send(200, _HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if parsed.path == "/api/schema":
                self._json(200, {"success": True, "result": provider.schema()})
                return
            if parsed.path == "/api/snapshot":
                names = [value for value in parse_qs(parsed.query).get("name", []) if value]
                self._json(200, {"success": True, "result": provider.snapshot(names)})
                return
            self._json(404, {"success": False, "error": "not found"})

        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _json(self, status: int, value: Any) -> None:
            data = json.dumps(_jsonable(value), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            self._send(status, data, "application/json; charset=utf-8")

        def _send(self, status: int, data: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("content-type", content_type)
            self.send_header("content-length", str(len(data)))
            self.send_header("cache-control", "no-store")
            self.end_headers()
            self.wfile.write(data)

    return Handler


def _unwrap(value: Any) -> Any:
    if not isinstance(value, Mapping) or "success" not in value or "result" not in value:
        return value
    if bool(value.get("success")):
        return value.get("result")
    raise RuntimeError(str(value.get("message") or "runtime request failed"))


def _as_dict(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("manifest must be an object")
    return dict(value)


def _jsonable(value: Any) -> Any:
    if isinstance(value, bytes):
        return {"__bytes_base64": base64.b64encode(value).decode("ascii")}
    if isinstance(value, bytearray):
        return {"__bytes_base64": base64.b64encode(bytes(value)).decode("ascii")}
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    try:
        import numpy as np

        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
    except Exception:
        pass
    return value


_HTML = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RynnRCP Visualizer</title><style>
:root{color-scheme:dark;--bg:#0b1020;--panel:#141b2d;--line:#26314a;--text:#e6edf7;--muted:#8895aa;--cyan:#36d7ff;--green:#52e3a4;--orange:#ffba69}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 15% 0,#16274b 0,transparent 34%),var(--bg);color:var(--text);font:14px -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
header{position:sticky;top:0;z-index:3;display:flex;justify-content:space-between;align-items:center;padding:18px 24px;background:#0b1020e8;border-bottom:1px solid var(--line);backdrop-filter:blur(10px)}
h1{font-size:20px;margin:0}.brand{color:var(--cyan)}.meta,.empty{color:var(--muted)}.live{display:inline-flex;gap:7px;align-items:center}.dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 12px var(--green)}
main{padding:20px;max-width:1600px;margin:auto}.section{margin-bottom:24px}.section-head{display:flex;align-items:end;justify-content:space-between;margin:0 2px 10px}.section h2{font-size:14px;text-transform:uppercase;letter-spacing:.12em;color:#a9b6ca;margin:0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:14px}.card{background:linear-gradient(145deg,#161f34,#111827);border:1px solid var(--line);border-radius:12px;overflow:hidden;box-shadow:0 10px 28px #0004}.card-head{display:flex;justify-content:space-between;gap:12px;padding:12px 14px;border-bottom:1px solid var(--line)}.name{font-weight:650;overflow:hidden;text-overflow:ellipsis}.type{font:11px ui-monospace,SFMono-Regular,monospace;color:var(--cyan)}
.body{padding:12px 14px}.image{display:block;width:100%;min-height:180px;max-height:420px;object-fit:contain;background:#070b14}.value-view{display:grid;gap:10px;max-height:260px;overflow:auto;padding-right:2px}.value-object,.value-list{display:grid;gap:9px}.value-field,.value-item{display:grid;gap:6px}.field-head{display:flex;align-items:center;justify-content:space-between;gap:8px}.field-name{font:600 11px/1.3 ui-monospace,SFMono-Regular,monospace;color:#aebbd0;overflow:hidden;text-overflow:ellipsis}.count{flex:none;padding:2px 6px;border:1px solid #2b3a57;border-radius:999px;color:var(--muted);font:10px ui-monospace,SFMono-Regular,monospace}.vector{display:grid;grid-template-columns:repeat(auto-fill,minmax(72px,1fr));gap:6px}.vector-cell{display:flex;align-items:baseline;justify-content:space-between;gap:7px;min-width:0;padding:7px 8px;background:#0b1222;border:1px solid #24314a;border-radius:7px}.vector-index{color:#66758d;font:9px ui-monospace,SFMono-Regular,monospace}.vector-value,.scalar{font:600 12px ui-monospace,SFMono-Regular,monospace;font-variant-numeric:tabular-nums;color:#dce8f8}.scalar{display:inline-block;width:max-content;max-width:100%;padding:6px 8px;background:#0b1222;border:1px solid #24314a;border-radius:7px;overflow:hidden;text-overflow:ellipsis}.scalar.number{color:#75ddff}.scalar.string{color:#b9e68b}.scalar.null{color:var(--muted)}.nested{padding-left:9px;border-left:2px solid #24314a}.age{margin-top:10px;color:var(--muted);font-size:11px}.chart{width:100%;height:100px;display:block;margin-bottom:10px;border-radius:7px;background:#0a1020}.action .type{color:var(--orange)}.action .vector-value,.action .scalar.number{color:#ffc174}
@media(max-width:600px){header{padding:14px}main{padding:12px}.grid{grid-template-columns:1fr}}
</style></head><body><header><div><h1><span class="brand">RynnRCP</span> Visualizer</h1><div id="robot" class="meta">正在连接…</div></div><div class="live"><span class="dot"></span><span id="status">LIVE</span></div></header>
<main><section class="section"><div class="section-head"><h2>Observations / State</h2><span id="updated" class="meta"></span></div><div id="observations" class="grid"></div></section><section class="section"><div class="section-head"><h2>Actions</h2><span class="meta">只读 · 最近下发值</span></div><div id="actions" class="grid"></div></section></main>
<script>
const $=id=>document.getElementById(id), histories=new Map();let schema={}, timer=null;
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function bytesUrl(value,encoding){const b=value?.__bytes_base64;if(!b)return '';const e=String(encoding||'jpeg').toLowerCase();return `data:${e==='png'?'image/png':'image/jpeg'};base64,${b}`}
function card(item,kind){const id=btoa(unescape(encodeURIComponent(item.name))).replace(/=/g,'');return `<article class="card ${kind}"><div class="card-head"><span class="name" title="${esc(item.name)}">${esc(item.name)}</span><span class="type">${esc(item.type||kind)}</span></div><div class="body" id="b-${id}"><div class="empty">等待数据…</div></div></article>`}
function setup(){const obs=schema.observations||[], acts=schema.actions||[];$('observations').innerHTML=obs.length?obs.map(x=>card(x,'observation')).join(''):'<div class="empty">没有配置 Observation</div>';$('actions').innerHTML=acts.length?acts.map(x=>card(x,'action')).join(''):'<div class="empty">没有配置 Action</div>'}
function body(name){const id=btoa(unescape(encodeURIComponent(name))).replace(/=/g,'');return $('b-'+id)}
function numbers(v){if(Array.isArray(v)&&v.every(Number.isFinite))return v;if(v&&typeof v==='object'){for(const x of Object.values(v)){if(Array.isArray(x)&&x.every(Number.isFinite))return x}}return null}
function fixed(value){const n=Number(value);if(!Number.isFinite(n))return String(value);const rounded=Object.is(n,-0)?0:n;return rounded.toFixed(3)}
function valueHtml(value,depth=0){
  if(typeof value==='number')return `<span class="scalar number">${fixed(value)}</span>`;
  if(typeof value==='string')return `<span class="scalar string" title="${esc(value)}">${esc(value)}</span>`;
  if(typeof value==='boolean')return `<span class="scalar">${value?'true':'false'}</span>`;
  if(value===null||value===undefined)return '<span class="scalar null">null</span>';
  if(Array.isArray(value)){
    if(value.every(Number.isFinite))return `<div class="vector">${value.map((n,i)=>`<div class="vector-cell"><span class="vector-index">${String(i).padStart(2,'0')}</span><span class="vector-value">${fixed(n)}</span></div>`).join('')}</div>`;
    return `<div class="value-list${depth?' nested':''}">${value.map((item,i)=>`<div class="value-item"><div class="field-head"><span class="field-name">[${i}]</span></div>${valueHtml(item,depth+1)}</div>`).join('')}</div>`;
  }
  if(typeof value==='object'){
    const entries=Object.entries(value);
    if(!entries.length)return '<span class="scalar null">empty</span>';
    return `<div class="value-object${depth?' nested':''}">${entries.map(([key,item])=>`<div class="value-field"><div class="field-head"><span class="field-name" title="${esc(key)}">${esc(key)}</span>${Array.isArray(item)?`<span class="count">${item.length} values</span>`:''}</div>${valueHtml(item,depth+1)}</div>`).join('')}</div>`;
  }
  return `<span class="scalar">${esc(String(value))}</span>`;
}
function draw(canvas,series){const d=devicePixelRatio||1,w=canvas.clientWidth*d,h=canvas.clientHeight*d;canvas.width=w;canvas.height=h;const c=canvas.getContext('2d');c.clearRect(0,0,w,h);if(series.length<2)return;const flat=series.flat(),lo=Math.min(...flat),hi=Math.max(...flat),span=hi-lo||1,cols=['#36d7ff','#52e3a4','#ffba69','#c48cff','#ff7185','#7ca7ff'];for(let j=0;j<Math.min(series[0].length,12);j++){c.beginPath();c.strokeStyle=cols[j%cols.length];c.lineWidth=1.5*d;series.forEach((row,i)=>{const x=i/(series.length-1)*w,y=h-(row[j]-lo)/span*(h-8*d)-4*d;i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()}}
function renderObs(o){const el=body(o.name);if(!el)return;const v=o.value||{};if(v.image){el.innerHTML=`<img class="image" alt="${esc(o.name)}"><div class="age"></div>`;el.querySelector('img').src=bytesUrl(v.image,v.encoding);el.querySelector('.age').textContent=`${v.width||'?'} × ${v.height||'?'} · ${new Date((o.timestamp||0)*1000).toLocaleTimeString()}`;return}const nums=numbers(v);let chart='';if(nums){const h=histories.get(o.name)||[];h.push(nums);if(h.length>300)h.shift();histories.set(o.name,h);chart='<canvas class="chart"></canvas>'}el.innerHTML=chart+`<div class="value-view">${valueHtml(v)}</div><div class="age">timestamp ${fixed(o.timestamp||0)}</div>`;if(nums)draw(el.querySelector('canvas'),histories.get(o.name))}
function renderActions(values){for(const a of schema.actions||[]){const el=body(a.name);if(!el)continue;const v=values[a.name];if(!v){el.innerHTML='<div class="empty">尚未捕获 Action</div>';continue}const nums=numbers(v.value);let chart='';if(nums){const key='action:'+a.name,h=histories.get(key)||[];h.push(nums);if(h.length>300)h.shift();histories.set(key,h);chart='<canvas class="chart"></canvas>'}el.innerHTML=chart+`<div class="value-view">${valueHtml(v.value)}</div><div class="age">${fixed(v.frame_rate||0)} Hz · ${new Date((v.timestamp||0)*1000).toLocaleTimeString()}</div>`;if(nums)draw(el.querySelector('canvas'),histories.get('action:'+a.name))}}
async function tick(){if(document.hidden)return;const q=(schema.observations||[]).map(x=>'name='+encodeURIComponent(x.name)).join('&');try{const r=await fetch('/api/snapshot?'+q,{cache:'no-store'}),j=await r.json();if(!j.success)throw Error(j.error||'snapshot failed');for(const o of j.result.observations||[])renderObs(o);renderActions(j.result.actions||{});$('updated').textContent='更新 '+new Date().toLocaleTimeString();$('status').textContent=j.result.error?'PARTIAL':'LIVE';document.querySelector('.dot').style.background=j.result.error?'#ffba69':'#52e3a4'}catch(e){$('status').textContent='DISCONNECTED';document.querySelector('.dot').style.background='#ff7185'}}
async function start(){const r=await fetch('/api/schema'),j=await r.json();schema=j.result||{};$('robot').textContent=`${schema.robot_name||schema.robot_id||'Robot'} · ${schema.robot_id||''}`;setup();await tick();timer=setInterval(tick,100)}
document.addEventListener('visibilitychange',()=>{if(!document.hidden)tick()});start().catch(e=>{$('status').textContent='ERROR';$('robot').textContent=e.message});
</script></body></html>'''
