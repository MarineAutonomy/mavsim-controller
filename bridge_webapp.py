#!/usr/bin/env python3
"""
mavsim Bridge Web Control Panel
===============================

A lightweight browser UI for the mavsim ROS2 bridge. Unlike the legacy
controller web app, there is NO user-code upload here: the bridge is fixed and
exposes the simulation as local ROS2 topics. This panel just lets you enter a
controller code or token, toggle options (sensors, observe-others), start/stop
the bridge, watch logs, and preview the camera.

It is mounted into the container and launched by `start.sh --mode web`. It
spawns run_controller.py (which auto-discovers the mounted bridge_controller as
/app/user_code/my_controller.py) and passes bridge options via environment
variables.
"""

import argparse
import collections
import io
import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Optional

from flask import Flask, Response, jsonify, request, send_file, stream_with_context

TOKEN_DIR = Path("/tmp/mavsim_tokens")
BAG_DIR = Path("/tmp/mavsim_bags")
FRAME_DIR = Path("/tmp/mavsim_camera_frames")
MAX_LOG_LINES = 2000
CAMERA_STALE_SECONDS = 5

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("bridge_webapp")

app = Flask(__name__)


class BridgeState:
    """Tracks the running bridge subprocess and its log output."""

    def __init__(self):
        self.process: Optional[subprocess.Popen] = None
        self.running = False
        self.config: dict = {}
        self.log_lines: collections.deque = collections.deque(maxlen=MAX_LOG_LINES)
        self.log_version = 0
        self._reader_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def start(self, config: dict) -> bool:
        with self._lock:
            if self.running:
                return False

            cmd = self._build_command(config)
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"

            # Bridge options are consumed by bridge_controller.py via env vars.
            if config.get("observe_others"):
                env["MAVSIM_OBSERVE_OTHERS"] = "1"
            else:
                env.pop("MAVSIM_OBSERVE_OTHERS", None)
            env["MAVSIM_CMD_TIMEOUT"] = str(config.get("cmd_timeout", 1.0))
            domain_id = config.get("ros_domain_id")
            if domain_id not in (None, ""):
                env["ROS_DOMAIN_ID"] = str(domain_id)

            logger.info("Starting bridge: %s", " ".join(cmd))
            self.log_lines.clear()
            self.log_version = 0
            self.config = config

            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                cwd="/app",
                bufsize=1,
                universal_newlines=True,
                start_new_session=True,
            )
            self.running = True
            self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
            self._reader_thread.start()
            return True

    def stop(self) -> bool:
        with self._lock:
            if not self.running or self.process is None:
                return False
            logger.info("Stopping bridge (pid=%s)", self.process.pid)
            # Signal/kill the whole process group (start_new_session=True at
            # launch), not just this PID - run_controller.py's SIGTERM
            # handler runs BaseController.close(), which itself needs up to
            # ~7s to cleanly tear down its own rosbridge/observer/visualizer
            # subprocess groups, leaving little slack in our 8s wait below.
            # If close() doesn't finish in time, a plain kill() here would
            # only SIGKILL run_controller.py itself, leaving whatever it
            # hadn't gotten to yet as orphans one layer further out.
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError) as e:
                logger.debug("Error signaling bridge process group: %s", e)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError) as e:
                    logger.debug("Error killing bridge process group: %s", e)
                self.process.wait()
            self.running = False
            self._cleanup_frames()
            return True

    @staticmethod
    def _build_command(config: dict) -> list:
        rate = config.get("rate", 10.0)
        token_path = config.get("token_path", "")

        cmd = [sys.executable, "-u", "run_controller.py"]
        if token_path:
            cmd += ["--token", token_path]
        else:
            cmd += [
                "--code",
                config.get("code", ""),
                "--backend-url",
                config.get("backend_url", "http://localhost:5000"),
            ]
            vessel_name = config.get("vessel_name", "")
            if vessel_name:
                cmd += ["--vessel-name", vessel_name]

        cmd += ["--rate", str(rate)]
        # Sensors are always enabled (plans/plan_headless_observer.md) - no more
        # opt-in --enable-sensors flag. The headless observer needs a real,
        # reachable frontend URL to trigger camera/lidar streaming.
        cmd += ["--frontend-url", config.get("frontend_url", "http://localhost:5173")]
        return cmd

    def _read_output(self):
        try:
            for line in self.process.stdout:
                ts = datetime.utcnow().strftime("%H:%M:%S")
                self.log_lines.append(f"[{ts}] {line.rstrip()}")
                self.log_version += 1
        except Exception:
            pass
        finally:
            with self._lock:
                self.running = False
            exit_code = self.process.returncode if self.process else -1
            ts = datetime.utcnow().strftime("%H:%M:%S")
            self.log_lines.append(f"[{ts}] --- Bridge exited (code {exit_code}) ---")
            self.log_version += 1

    @staticmethod
    def _cleanup_frames():
        if FRAME_DIR.exists():
            for item in FRAME_DIR.iterdir():
                try:
                    item.unlink()
                except OSError:
                    pass


state = BridgeState()


@app.route("/")
def index():
    return Response(_FRONTEND_HTML, content_type="text/html")


@app.route("/api/token", methods=["POST"])
def upload_token():
    if state.running:
        return jsonify({"error": "Cannot load token while bridge is running"}), 409

    if request.content_type and "multipart" in request.content_type:
        f = request.files.get("token")
        if not f:
            return jsonify({"error": "No token file provided"}), 400
        try:
            token_data = json.loads(f.read().decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            return jsonify({"error": f"Invalid JSON in token file: {e}"}), 400
    else:
        token_data = request.get_json(silent=True)
        if not token_data:
            return jsonify({"error": "No valid JSON token provided"}), 400

    missing = [k for k in ("session_id", "controller_code") if k not in token_data]
    if missing:
        return jsonify({"error": f"Token missing required fields: {missing}"}), 400

    TOKEN_DIR.mkdir(parents=True, exist_ok=True)
    token_path = TOKEN_DIR / "controller_token.json"
    token_path.write_text(json.dumps(token_data, indent=2))

    vessels = token_data.get("vessels", [])
    return jsonify({
        "status": "ok",
        "session_id": token_data["session_id"],
        "controller_code": token_data["controller_code"],
        "vessels": vessels,
        "vessel_count": len(vessels),
        "token_path": str(token_path),
    }), 200


@app.route("/api/token", methods=["GET"])
def get_token_info():
    token_path = TOKEN_DIR / "controller_token.json"
    if not token_path.exists():
        return jsonify({"loaded": False}), 200
    try:
        token_data = json.loads(token_path.read_text())
        vessels = token_data.get("vessels", [])
        return jsonify({
            "loaded": True,
            "session_id": token_data.get("session_id"),
            "controller_code": token_data.get("controller_code"),
            "vessels": vessels,
            "vessel_count": len(vessels),
        }), 200
    except (json.JSONDecodeError, OSError):
        return jsonify({"loaded": False}), 200


@app.route("/api/token", methods=["DELETE"])
def clear_token():
    if state.running:
        return jsonify({"error": "Cannot clear token while bridge is running"}), 409
    token_path = TOKEN_DIR / "controller_token.json"
    if token_path.exists():
        token_path.unlink()
    return jsonify({"status": "cleared"}), 200


@app.route("/api/start", methods=["POST"])
def start_bridge():
    data = request.get_json(silent=True) or {}
    use_token = bool(data.get("use_token", False))
    token_path = TOKEN_DIR / "controller_token.json"

    if use_token:
        if not token_path.exists():
            return jsonify({"error": "No token loaded. Load a token first."}), 400
    else:
        code = (data.get("code") or "").strip()
        if not code:
            return jsonify({"error": "Controller code is required"}), 400

    config = {
        "rate": float(data.get("rate", 10.0)),
        "observe_others": bool(data.get("observe_others", False)),
        "cmd_timeout": float(data.get("cmd_timeout", 1.0)),
        "ros_domain_id": data.get("ros_domain_id", ""),
        "frontend_url": data.get("frontend_url") or "http://localhost:5173",
    }
    if use_token:
        config["token_path"] = str(token_path)
    else:
        config["code"] = (data.get("code") or "").strip()
        config["backend_url"] = data.get("backend_url", "http://localhost:5000")
        config["vessel_name"] = data.get("vessel_name", "")

    if state.start(config):
        return jsonify({"status": "started", "config": config}), 200
    return jsonify({"error": "Bridge is already running"}), 409


@app.route("/api/stop", methods=["POST"])
def stop_bridge():
    if state.stop():
        return jsonify({"status": "stopped"}), 200
    return jsonify({"error": "Bridge is not running"}), 409


@app.route("/api/status", methods=["GET"])
def get_status():
    return jsonify({
        "running": state.running,
        "config": state.config if state.running else {},
        "pid": state.process.pid if state.process and state.running else None,
    }), 200


@app.route("/api/logs")
def stream_logs():
    def generate():
        last_version = 0
        sent_count = 0
        while True:
            current_version = state.log_version
            if current_version > last_version:
                lines = list(state.log_lines)
                for line in lines[sent_count:]:
                    yield f"data: {line}\n\n"
                sent_count = len(lines)
                last_version = current_version
            if not state.running and last_version == state.log_version:
                lines = list(state.log_lines)
                for line in lines[sent_count:]:
                    yield f"data: {line}\n\n"
                yield "event: done\ndata: finished\n\n"
                break
            time.sleep(0.3)

    return Response(
        stream_with_context(generate()),
        content_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@app.route("/api/recordings", methods=["GET"])
def list_recordings():
    recordings = []
    if BAG_DIR.exists():
        for item in sorted(BAG_DIR.iterdir(), reverse=True):
            if item.is_dir():
                total_size = sum(f.stat().st_size for f in item.iterdir() if f.is_file())
                recordings.append({
                    "name": item.name,
                    "size": total_size,
                    "modified": datetime.fromtimestamp(item.stat().st_mtime).isoformat() + "Z",
                })
            elif item.suffix == ".mcap":
                recordings.append({
                    "name": item.name,
                    "size": item.stat().st_size,
                    "modified": datetime.fromtimestamp(item.stat().st_mtime).isoformat() + "Z",
                })
    return jsonify({"recordings": recordings}), 200


@app.route("/api/recordings/<name>/download", methods=["GET"])
def download_recording(name: str):
    target = BAG_DIR / name
    if not target.exists():
        return jsonify({"error": "Recording not found"}), 404
    if target.is_file():
        return send_file(str(target), as_attachment=True)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in target.rglob("*"):
            if f.is_file():
                zf.write(str(f), f.relative_to(target))
    buf.seek(0)
    return send_file(buf, mimetype="application/zip", as_attachment=True, download_name=f"{name}.zip")


@app.route("/api/camera/list")
def camera_list():
    cameras = []
    now = time.time()
    if FRAME_DIR.exists():
        for f in sorted(FRAME_DIR.glob("*.jpg")):
            parts = f.stem.split("_")
            if len(parts) != 2:
                continue
            try:
                vid, cid = int(parts[0]), int(parts[1])
                if now - f.stat().st_mtime > CAMERA_STALE_SECONDS:
                    continue
                cameras.append({"vessel_id": vid, "camera_id": cid, "size": f.stat().st_size})
            except (ValueError, OSError):
                pass
    return jsonify({"cameras": cameras}), 200


@app.route("/api/camera/frame")
def camera_frame():
    vid = request.args.get("v", "0")
    cid = request.args.get("c", "0")
    frame_path = FRAME_DIR / f"{vid}_{cid}.jpg"
    if frame_path.exists():
        try:
            mtime_ns = frame_path.stat().st_mtime_ns
            return Response(
                frame_path.read_bytes(),
                mimetype="image/jpeg",
                headers={
                    "Cache-Control": "no-store",
                    # Sub-second frame identity so the client can count only
                    # genuinely NEW frames (true FPS), not every poll.
                    "X-Frame-Mtime": str(mtime_ns),
                    "Access-Control-Expose-Headers": "X-Frame-Mtime",
                },
            )
        except OSError:
            return Response(b"", status=503)
    return Response(b"", status=204)


_FRONTEND_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>mavsim Bridge</title>
<style>
  :root {
    --bg:#0f1117; --surface:#1a1d27; --surface2:#23273a; --border:#2d3348;
    --text:#e1e4ed; --text2:#8b91a8; --accent:#4f8ff7; --accent-hover:#3a7ae0;
    --danger:#e5484d; --danger-hover:#cd2b31; --success:#30a46c; --warning:#f5a623;
    --radius:8px; --font:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
    --mono:'SF Mono','Fira Code','Consolas',monospace;
  }
  *{margin:0;padding:0;box-sizing:border-box;}
  body{font-family:var(--font);background:var(--bg);color:var(--text);min-height:100vh;}
  header{background:var(--surface);border-bottom:1px solid var(--border);padding:16px 24px;
    display:flex;align-items:center;justify-content:space-between;}
  header h1{font-size:1.2rem;font-weight:600;}
  header .sub{font-size:.78rem;color:var(--text2);margin-top:2px;}
  .status-badge{padding:4px 12px;border-radius:12px;font-size:.8rem;font-weight:500;}
  .status-badge.running{background:rgba(48,164,108,.15);color:var(--success);}
  .status-badge.stopped{background:rgba(139,145,168,.15);color:var(--text2);}
  main{max-width:1200px;margin:0 auto;padding:24px;display:grid;grid-template-columns:380px 1fr;gap:24px;}
  @media (max-width:900px){main{grid-template-columns:1fr;}}
  .panel{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:20px;}
  .panel h2{font-size:.95rem;font-weight:600;margin-bottom:16px;color:var(--text2);
    text-transform:uppercase;letter-spacing:.5px;}
  label{display:block;font-size:.85rem;color:var(--text2);margin-bottom:4px;margin-top:12px;}
  label:first-of-type{margin-top:0;}
  input[type="text"],input[type="number"]{width:100%;padding:8px 12px;border:1px solid var(--border);
    border-radius:var(--radius);background:var(--bg);color:var(--text);font-size:.9rem;outline:none;}
  input:focus{border-color:var(--accent);}
  .toggle-row{display:flex;align-items:center;justify-content:space-between;margin-top:14px;}
  .toggle-row .lbl{font-size:.88rem;}
  .toggle-row .hint{font-size:.72rem;color:var(--text2);}
  .toggle{position:relative;width:44px;height:24px;flex-shrink:0;}
  .toggle input{opacity:0;width:0;height:0;}
  .toggle .slider{position:absolute;inset:0;background:var(--border);border-radius:12px;cursor:pointer;transition:background .2s;}
  .toggle .slider::after{content:'';position:absolute;width:18px;height:18px;left:3px;top:3px;
    background:var(--text);border-radius:50%;transition:transform .2s;}
  .toggle input:checked + .slider{background:var(--accent);}
  .toggle input:checked + .slider::after{transform:translateX(20px);}
  .btn-row{display:flex;gap:8px;margin-top:20px;}
  button{padding:9px 20px;border:none;border-radius:var(--radius);font-size:.9rem;font-weight:500;cursor:pointer;}
  button:disabled{opacity:.4;cursor:not-allowed;}
  .btn-primary{background:var(--accent);color:#fff;flex:1;}
  .btn-primary:hover:not(:disabled){background:var(--accent-hover);}
  .btn-danger{background:var(--danger);color:#fff;}
  .btn-danger:hover:not(:disabled){background:var(--danger-hover);}
  .btn-secondary{background:var(--surface2);color:var(--text);border:1px solid var(--border);}
  .mode-tabs{display:flex;margin-bottom:16px;border:1px solid var(--border);border-radius:var(--radius);overflow:hidden;}
  .mode-tab{flex:1;padding:8px 12px;text-align:center;font-size:.85rem;font-weight:500;cursor:pointer;
    background:var(--bg);color:var(--text2);border:none;}
  .mode-tab.active{background:var(--accent);color:#fff;}
  .token-section textarea{width:100%;min-height:110px;padding:10px 12px;border:1px solid var(--border);
    border-radius:var(--radius);background:var(--bg);color:var(--text);font-family:var(--mono);font-size:.78rem;resize:vertical;outline:none;}
  .token-info{margin-top:12px;padding:12px;background:var(--bg);border:1px solid var(--border);border-radius:var(--radius);}
  .token-info .row{display:flex;justify-content:space-between;padding:4px 0;font-size:.82rem;}
  .token-info .k{color:var(--text2);} .token-info .v{font-family:var(--mono);}
  .right-col{display:flex;flex-direction:column;gap:24px;}
  .empty-state{color:var(--text2);font-size:.85rem;text-align:center;padding:20px;}
  #logViewer{background:var(--bg);border:1px solid var(--border);border-radius:var(--radius);padding:12px;
    font-family:var(--mono);font-size:.78rem;line-height:1.6;overflow-y:auto;min-height:300px;max-height:60vh;
    color:var(--text2);white-space:pre-wrap;word-break:break-all;}
  #logViewer .log-error{color:var(--danger);} #logViewer .log-warn{color:var(--warning);} #logViewer .log-info{color:var(--success);}
  .rec-item{display:flex;align-items:center;justify-content:space-between;padding:10px 12px;background:var(--bg);
    border-radius:var(--radius);margin-top:6px;}
  .rec-item .n{font-family:var(--mono);font-size:.82rem;} .rec-item .m{font-size:.75rem;color:var(--text2);}
  .cam-toolbar{display:flex;align-items:center;gap:10px;margin-bottom:12px;flex-wrap:wrap;}
  .cam-toolbar select{padding:6px 10px;border:1px solid var(--border);border-radius:var(--radius);
    background:var(--bg);color:var(--text);font-size:.85rem;outline:none;}
  .cam-stat{font-size:.78rem;color:var(--text2);}
  .cam-view{background:var(--bg);border:1px solid var(--border);border-radius:var(--radius);overflow:hidden;text-align:center;min-height:80px;}
  .cam-view img{max-width:100%;max-height:45vh;display:block;margin:0 auto;}
</style>
</head>
<body>
<header>
  <div><h1>mavsim Bridge</h1><div class="sub">ROS2 bridge control panel</div></div>
  <div style="display:flex;align-items:center;gap:12px">
    <a id="visualizerLink" href="#" target="_blank" class="btn-secondary" style="display:none;text-decoration:none;padding:7px 14px;border-radius:6px;font-size:.85rem">Open ROS2 Visualizer &#8599;</a>
    <span id="statusBadge" class="status-badge stopped">Stopped</span>
  </div>
</header>
<main>
  <div class="left-col">
    <div class="panel">
      <h2>Connection</h2>
      <div class="mode-tabs">
        <button class="mode-tab active" id="modeCodeTab" onclick="setMode('code')">Code</button>
        <button class="mode-tab" id="modeTokenTab" onclick="setMode('token')">Token</button>
      </div>
      <div id="codeModeFields">
        <label for="code">Controller Code *</label>
        <input type="text" id="code" placeholder="e.g. ABC123">
        <label for="backendUrl">Backend URL</label>
        <input type="text" id="backendUrl" placeholder="http://localhost:5000">
        <label for="vesselName">Vessel Name (optional)</label>
        <input type="text" id="vesselName" placeholder="auto-assigned if empty">
      </div>
      <div id="tokenModeFields" class="token-section" style="display:none">
        <label>Paste token JSON or upload file</label>
        <textarea id="tokenText" placeholder='{"session_id":"...","controller_code":"...","vessels":["matsya_02"]}'></textarea>
        <input type="file" id="tokenFileInput" accept=".json" style="display:none" onchange="handleTokenFileSelect(event)">
        <div class="btn-row" style="margin-top:8px">
          <button class="btn-secondary" id="loadTokenBtn" onclick="loadTokenFromText()" style="flex:1">Load Token</button>
          <button class="btn-secondary" onclick="document.getElementById('tokenFileInput').click()" style="width:auto">File</button>
          <button class="btn-secondary" id="clearTokenBtn" onclick="clearToken()" style="width:auto">Clear</button>
        </div>
        <div id="tokenInfo" style="display:none"></div>
      </div>
    </div>

    <div class="panel">
      <h2>Bridge Options</h2>
      <label for="rate">Forward Rate (Hz)</label>
      <input type="number" id="rate" value="10" min="1" max="200" step="1">
      <label for="cmdTimeout">Command Timeout (s)</label>
      <input type="number" id="cmdTimeout" value="1.0" min="0" step="0.1">
      <label for="rosDomainId">ROS_DOMAIN_ID</label>
      <input type="number" id="rosDomainId" placeholder="container default" min="0" max="232">
      <label for="frontendUrl">Frontend URL (for camera/lidar streaming)</label>
      <input type="text" id="frontendUrl" placeholder="http://localhost:5173">
      <div class="toggle-row">
        <span class="lbl">Observe Others<br><span class="hint">read-only odometry + actuator state for non-owned vessels</span></span>
        <label class="toggle"><input type="checkbox" id="observeOthers"><span class="slider"></span></label>
      </div>
      <div class="btn-row">
        <button class="btn-primary" id="startBtn" onclick="startBridge()">Start</button>
        <button class="btn-danger" id="stopBtn" onclick="stopBridge()" disabled>Stop</button>
      </div>
    </div>
  </div>

  <div class="right-col">
    <div class="panel" id="cameraPanel" style="display:none">
      <h2>Camera Feed</h2>
      <div class="cam-toolbar">
        <select id="cameraSelect" onchange="selectCamera()"></select>
        <span class="cam-stat" id="camFps">-- FPS</span>
        <span class="cam-stat" id="camSize"></span>
      </div>
      <div class="cam-view">
        <img id="camFeed" alt="" style="display:none" />
        <div id="camPlaceholder" class="empty-state">Waiting for camera frames&hellip;</div>
      </div>
    </div>
    <div class="panel" style="flex:1">
      <h2>Logs</h2>
      <div id="logViewer"><span class="empty-state">Logs appear here when the bridge starts.</span></div>
    </div>
    <div class="panel">
      <h2>Recordings</h2>
      <div id="recList"><span class="empty-state">No recordings yet.</span></div>
      <div style="margin-top:12px;text-align:right">
        <button class="btn-secondary" onclick="refreshRecordings()">Refresh</button>
      </div>
    </div>
  </div>
</main>
<script>
const $ = (s) => document.querySelector(s);
let eventSource = null, currentMode = 'code', tokenLoaded = false;

function setMode(mode){
  currentMode = mode;
  $('#modeCodeTab').classList.toggle('active', mode==='code');
  $('#modeTokenTab').classList.toggle('active', mode==='token');
  $('#codeModeFields').style.display = mode==='code' ? '' : 'none';
  $('#tokenModeFields').style.display = mode==='token' ? '' : 'none';
}

async function loadTokenFromText(){
  const text = $('#tokenText').value.trim();
  if(!text){alert('Paste a token JSON first');return;}
  try{JSON.parse(text);}catch{alert('Invalid JSON');return;}
  const res = await fetch('/api/token',{method:'POST',headers:{'Content-Type':'application/json'},body:text});
  const data = await res.json();
  if(!res.ok){alert(data.error||'Failed to load token');return;}
  showTokenInfo(data);
}
function handleTokenFileSelect(e){
  const file = e.target.files[0]; if(!file) return;
  const reader = new FileReader();
  reader.onload = async (ev)=>{
    $('#tokenText').value = ev.target.result;
    await loadTokenFromText();
  };
  reader.readAsText(file); e.target.value='';
}
function showTokenInfo(data){
  tokenLoaded = true;
  const info = $('#tokenInfo'); info.style.display='';
  let vlist='';
  if(data.vessels && data.vessels.length){
    vlist = data.vessels.map(v=>{
      const n = typeof v==='string'?v:(v.ros_name||v.name||'vessel');
      return '<div class="row"><span class="k">vessel</span><span class="v">'+esc(n)+'</span></div>';
    }).join('');
  }
  info.innerHTML = '<div class="token-info">'
    +'<div class="row"><span class="k">Session</span><span class="v">'+esc(data.session_id||'')+'</span></div>'
    +'<div class="row"><span class="k">Code</span><span class="v">'+esc(data.controller_code||'')+'</span></div>'
    +'<div class="row"><span class="k">Vessels</span><span class="v">'+(data.vessel_count||0)+'</span></div>'
    +vlist+'</div>';
}
async function clearToken(){
  await fetch('/api/token',{method:'DELETE'});
  tokenLoaded=false; $('#tokenInfo').style.display='none'; $('#tokenInfo').innerHTML=''; $('#tokenText').value='';
}
async function refreshTokenInfo(){
  try{const r=await fetch('/api/token');const d=await r.json();if(d.loaded)showTokenInfo(d);}catch{}
}

async function startBridge(){
  const body = {
    rate: parseFloat($('#rate').value)||10,
    cmd_timeout: parseFloat($('#cmdTimeout').value)||0,
    observe_others: $('#observeOthers').checked,
    ros_domain_id: $('#rosDomainId').value.trim(),
    frontend_url: $('#frontendUrl').value.trim()||'http://localhost:5173',
  };
  if(currentMode==='token'){
    if(!tokenLoaded){alert('Load a token first');return;}
    body.use_token = true;
  }else{
    const code = $('#code').value.trim();
    if(!code){alert('Controller code is required');return;}
    body.code = code;
    body.backend_url = $('#backendUrl').value.trim()||'http://localhost:5000';
    body.vessel_name = $('#vesselName').value.trim();
  }
  const res = await fetch('/api/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const data = await res.json();
  if(!res.ok){alert(data.error||'Failed to start');return;}
  setRunning(true); startLogStream(); startCameraPolling();
}
async function stopBridge(){
  await fetch('/api/stop',{method:'POST'});
  setRunning(false); stopLogStream(); stopCameraPolling();
  setTimeout(refreshRecordings,1000);
}
function setRunning(running){
  $('#startBtn').disabled = running; $('#stopBtn').disabled = !running;
  const b=$('#statusBadge'); b.textContent = running?'Running':'Stopped';
  b.className='status-badge '+(running?'running':'stopped');
  const vLink=$('#visualizerLink');
  vLink.style.display = running ? 'inline-block' : 'none';
  if(running) vLink.href = 'http://'+location.hostname+':8899/';
  ['code','backendUrl','vesselName','rate','cmdTimeout','rosDomainId','frontendUrl','observeOthers',
   'modeCodeTab','modeTokenTab','tokenText','loadTokenBtn','clearTokenBtn'].forEach(id=>{
    const el=$('#'+id); if(el) el.disabled=running;
  });
}
function startLogStream(){
  stopLogStream(); const v=$('#logViewer'); v.innerHTML='';
  eventSource = new EventSource('/api/logs');
  eventSource.onmessage=(e)=>{
    const l=document.createElement('div'); l.textContent=e.data;
    if(/error/i.test(e.data))l.className='log-error';
    else if(/warn/i.test(e.data))l.className='log-warn';
    else if(/info/i.test(e.data))l.className='log-info';
    v.appendChild(l); v.scrollTop=v.scrollHeight;
  };
  eventSource.addEventListener('done',()=>{stopLogStream();stopCameraPolling();setRunning(false);setTimeout(refreshRecordings,1000);});
  eventSource.onerror=()=>{stopLogStream();pollStatus();};
}
function stopLogStream(){if(eventSource){eventSource.close();eventSource=null;}}
async function pollStatus(){
  try{const r=await fetch('/api/status');const d=await r.json();setRunning(d.running);
    if(d.running&&!eventSource)startLogStream();
    if(d.running&&!camPollTimer)startCameraPolling();}catch{}
}
async function refreshRecordings(){
  try{const r=await fetch('/api/recordings');const d=await r.json();const c=$('#recList');
    if(!d.recordings.length){c.innerHTML='<span class="empty-state">No recordings yet.</span>';return;}
    c.innerHTML=d.recordings.map(x=>'<div class="rec-item"><div><div class="n">'+esc(x.name)+'</div>'
      +'<div class="m">'+fmtSize(x.size)+' &middot; '+fmtDate(x.modified)+'</div></div>'
      +'<button class="btn-secondary" onclick="dl(\\''+esc(x.name)+'\\')">Download</button></div>').join('');
  }catch{}
}
function dl(n){window.open('/api/recordings/'+encodeURIComponent(n)+'/download','_blank');}
function esc(s){return (s+'').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;').replace(/'/g,'&#39;');}
function fmtSize(b){if(b<1024)return b+' B';if(b<1048576)return (b/1024).toFixed(1)+' KB';return (b/1048576).toFixed(1)+' MB';}
function fmtDate(iso){try{return new Date(iso).toLocaleString();}catch{return iso;}}

let camPollTimer=null,camListTimer=null,camFpsCount=0,camFpsTick=performance.now(),camSelected=null,camPrevBlob=null,camLastFrameId=null;
async function refreshCameraList(){
  try{const r=await fetch('/api/camera/list');const d=await r.json();const sel=$('#cameraSelect');const p=$('#cameraPanel');
    if(!d.cameras.length){p.style.display='none';return;}
    p.style.display='';
    const firstLoad = sel.options.length===0;
    const prev=sel.value;
    sel.innerHTML='<option value="">None (off)</option>'
      +d.cameras.map(c=>'<option value="'+c.vessel_id+'_'+c.camera_id+'">Vessel '+c.vessel_id+' \\u2013 Camera '+c.camera_id+'</option>').join('');
    if(!firstLoad&&[...sel.options].some(o=>o.value===prev))sel.value=prev;
    else sel.value=d.cameras[0].vessel_id+'_'+d.cameras[0].camera_id;
    selectCamera();
  }catch{}
}
function selectCamera(){
  const v=$('#cameraSelect').value;
  camLastFrameId=null;
  if(!v){
    camSelected=null;
    $('#camFeed').style.display='none';
    const ph=$('#camPlaceholder'); ph.style.display=''; ph.textContent='Camera feed off (None selected)';
    $('#camFps').textContent='-- FPS'; $('#camSize').textContent='';
    return;
  }
  const [a,b]=v.split('_'); camSelected={v:a,c:b};
  $('#camPlaceholder').textContent='Waiting for camera frames…';
}
async function pollCameraFrame(){
  if(!camSelected)return;
  try{const r=await fetch('/api/camera/frame?v='+camSelected.v+'&c='+camSelected.c,{cache:'no-store'});
    if(r.ok&&r.status===200){
      const fid=r.headers.get('X-Frame-Mtime');
      // Only treat as a real frame if it changed since last time -> true FPS.
      if(fid===null||fid!==camLastFrameId){
        const blob=await r.blob();
        if(blob.size>0){
          camLastFrameId=fid;
          const url=URL.createObjectURL(blob);const img=$('#camFeed');img.src=url;img.style.display='';
          $('#camPlaceholder').style.display='none';if(camPrevBlob)URL.revokeObjectURL(camPrevBlob);camPrevBlob=url;
          camFpsCount++;$('#camSize').textContent=fmtSize(blob.size);
        }
      }
    }
  }catch{}
  const now=performance.now();
  if(now-camFpsTick>=1000){$('#camFps').textContent=camFpsCount+' FPS';camFpsCount=0;camFpsTick=now;}
}
function startCameraPolling(){stopCameraPolling();refreshCameraList();camListTimer=setInterval(refreshCameraList,3000);camPollTimer=setInterval(pollCameraFrame,80);}
function stopCameraPolling(){if(camPollTimer){clearInterval(camPollTimer);camPollTimer=null;}if(camListTimer){clearInterval(camListTimer);camListTimer=null;}$('#cameraPanel').style.display='none';camSelected=null;}

pollStatus(); refreshRecordings(); refreshTokenInfo();
setInterval(refreshRecordings,30000);
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="mavsim Bridge Web Control Panel")
    parser.add_argument("--port", type=int, default=8888, help="Port to listen on")
    parser.add_argument("--backend-url", default="http://localhost:5000",
                        help="Default backend URL shown in the UI")
    parser.add_argument("--frontend-url", default="http://localhost:5173",
                        help="Default frontend URL shown in the UI - where the headless sensor "
                             "observer navigates to (plans/plan_headless_observer.md)")
    args = parser.parse_args()

    global _FRONTEND_HTML
    _FRONTEND_HTML = _FRONTEND_HTML.replace(
        'placeholder="http://localhost:5000"',
        f'placeholder="http://localhost:5000" value="{args.backend_url}"',
        1,
    )
    _FRONTEND_HTML = _FRONTEND_HTML.replace(
        'placeholder="http://localhost:5173"',
        f'placeholder="http://localhost:5173" value="{args.frontend_url}"',
        1,
    )

    BAG_DIR.mkdir(parents=True, exist_ok=True)
    TOKEN_DIR.mkdir(parents=True, exist_ok=True)

    logger.info(
        "Starting bridge web panel on port %d (backend %s, frontend %s)",
        args.port, args.backend_url, args.frontend_url,
    )

    def handle_signal(sig, frame):
        logger.info("Shutting down...")
        state.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    app.run(host="0.0.0.0", port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
