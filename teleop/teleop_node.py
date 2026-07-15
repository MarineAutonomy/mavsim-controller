#!/usr/bin/env python3
"""
mavsim Keyboard Teleop Node
===========================

A real rclpy node (not just a browser talking to rosbridge) that publishes
interfaces/Actuator commands on /<vessel>/actuator_cmd for every vessel the
bridge owns, driven by keypresses from a browser page it serves itself.
Launched as a subprocess of base_controller.py (see _launch_teleop()), the
same way rosbridge/the visualizer/the sensor observer already are - see
plans/plan_teleop.md.

Three concerns, one process:
  - A fixed-rate command loop (own thread) ramps each vessel's normalized
    6DOF command (surge/sway/heave/roll/pitch/yaw, each in [-1,1]) toward
    whatever its currently-held keys imply, or back toward 0 if released/
    disconnected (dead-man safety), runs it through that vessel's
    ThrustAllocator (allocation.py), and publishes the result. It publishes
    continuously, even all-zero - bridge_controller.py's own
    MAVSIM_CMD_TIMEOUT (default 1s) is a second, independent dead-man
    backstop if this whole process dies; this loop's own ramp is the smooth
    front line, not a replacement for it.
  - A WebSocket server (its own asyncio loop/thread) receives raw key
    up/down events from the browser and pushes ramped-command telemetry
    back for the page's live DOF meters.
  - A small Flask app serves the page itself and a read-only /api/config
    (which vessels are owned, which DOFs each can actually reach).

Vessel geometry (thrusters/control_surfaces) is read once at startup from
the JSON file base_controller.py writes per session (see
_write_teleop_config()) - not fetched over HTTP, since that data is already
in-process on the BaseController side.
"""

import argparse
import asyncio
import json
import logging
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np
from flask import Flask, Response, jsonify

from allocation import DOF_ORDER, ThrustAllocator

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("teleop_node")

DEFAULT_CONFIG_FILE = "/tmp/mavsim_bridge_state/teleop_config.json"
CONFIG_WAIT_TIMEOUT = 5.0  # seconds; base_controller.py writes this file
                            # slightly after launching this process

# Browser KeyboardEvent.code -> (DOF index into DOF_ORDER, sign). event.code
# (not event.key) is used so the mapping is layout-independent (e.g. AZERTY
# keyboards don't break WASD).
KEY_MAP = {
    "ArrowUp": (0, 1.0), "ArrowDown": (0, -1.0),     # surge
    "ArrowRight": (1, 1.0), "ArrowLeft": (1, -1.0),  # sway
    "KeyW": (2, 1.0), "KeyS": (2, -1.0),             # heave
    "KeyD": (5, 1.0), "KeyA": (5, -1.0),             # yaw
    "KeyQ": (3, 1.0), "KeyE": (3, -1.0),             # roll
    "KeyR": (4, 1.0), "KeyF": (4, -1.0),             # pitch
}

DOF_LABELS = ("Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw")


def _import_actuator():
    """Lazy import guard, same pattern as bridge_controller.py's
    _import_actuator() - tolerates environments where the built `interfaces`
    package isn't sourced (e.g. running this file for local dev outside the
    bridge container)."""
    try:
        from interfaces.msg import Actuator
        return Actuator
    except ImportError:
        return None


def _init_rclpy_node():
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        return rclpy.create_node("mavsim_teleop")
    except Exception as exc:  # pragma: no cover - best effort
        logger.warning("rclpy not available (%s); commands will be computed but not published", exc)
        return None


class VesselTeleop:
    """Ramp state + allocator + ROS2 publisher for a single owned vessel."""

    def __init__(self, name: str, thrusters: list, control_surfaces: list, node, actuator_cls):
        self.name = name
        self.allocator = ThrustAllocator(thrusters, control_surfaces)
        self.pub = None
        if node is not None and actuator_cls is not None:
            self.pub = node.create_publisher(actuator_cls, f"/{name}/actuator_cmd", 10)
        self.cmd = np.zeros(6)
        self._held_keys = set()
        self._lock = threading.Lock()

    def set_key(self, code: str, down: bool):
        with self._lock:
            if down:
                self._held_keys.add(code)
            else:
                self._held_keys.discard(code)

    def clear_keys(self):
        with self._lock:
            self._held_keys.clear()

    def _target(self) -> np.ndarray:
        target = np.zeros(6)
        with self._lock:
            keys = list(self._held_keys)
        for code in keys:
            mapping = KEY_MAP.get(code)
            if mapping:
                idx, sign = mapping
                target[idx] += sign
        return np.clip(target, -1.0, 1.0)

    def step(self, dt: float, ramp_seconds: float) -> np.ndarray:
        """Ramp self.cmd toward the current key-implied target by at most
        one ramp-step this tick, and return the new command."""
        target = self._target()
        max_step = dt / max(ramp_seconds, 1e-6)
        change = np.clip(target - self.cmd, -max_step, max_step)
        self.cmd = np.clip(self.cmd + change, -1.0, 1.0)
        return self.cmd

    def publish(self, actuator_cls):
        if self.pub is None or actuator_cls is None:
            return
        values = self.allocator.solve(self.cmd)
        if not values:
            return
        names = list(values.keys())
        msg = actuator_cls()
        msg.actuator_names = names
        msg.actuator_values = [float(values[n]) for n in names]
        msg.covariance = [0.0] * len(names)
        self.pub.publish(msg)


def _load_vessel_configs(path: str, timeout: float) -> dict:
    deadline = time.monotonic() + timeout
    config_path = Path(path)
    while time.monotonic() < deadline:
        try:
            return json.loads(config_path.read_text())
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            time.sleep(0.2)
    logger.warning(
        "Teleop config file %s not found/readable after %.1fs; starting with no vessels",
        path, timeout,
    )
    return {}


class TeleopServer:
    def __init__(self, config_file: str, rate: float, ramp_seconds: float):
        self.rate = rate
        self.ramp_seconds = ramp_seconds
        self.node = _init_rclpy_node()
        self.actuator_cls = _import_actuator()
        if self.actuator_cls is None:
            logger.error(
                "interfaces/Actuator is not available in this image; teleop "
                "will compute commands but cannot publish them."
            )

        vessel_configs = _load_vessel_configs(config_file, CONFIG_WAIT_TIMEOUT)
        self.vessels: Dict[str, VesselTeleop] = {
            name: VesselTeleop(
                name,
                cfg.get("thrusters", []),
                cfg.get("control_surfaces", []),
                self.node,
                self.actuator_cls,
            )
            for name, cfg in vessel_configs.items()
        }
        if self.vessels:
            logger.info("Teleop ready for vessels: %s", ", ".join(self.vessels))
        else:
            logger.warning("Teleop started with no owned vessels")

        # vessel_name -> the single websocket currently allowed to drive it.
        self.sessions: Dict[str, object] = {}
        self._stop = False

    # ------------------------------------------------------------------
    # Fixed-rate command loop
    # ------------------------------------------------------------------

    def run_command_loop(self):
        period = 1.0 / self.rate
        while not self._stop:
            start = time.monotonic()
            for vessel in self.vessels.values():
                vessel.step(period, self.ramp_seconds)
                vessel.publish(self.actuator_cls)
            elapsed = time.monotonic() - start
            time.sleep(max(0.0, period - elapsed))

    # ------------------------------------------------------------------
    # WebSocket server (key intake + telemetry pushback)
    # ------------------------------------------------------------------

    async def _telemetry_sender(self, websocket, current: dict):
        import websockets
        try:
            while True:
                await asyncio.sleep(1.0 / self.rate)
                vname = current.get("vessel")
                vessel = self.vessels.get(vname) if vname else None
                if vessel is None:
                    continue
                await websocket.send(json.dumps({
                    "type": "state",
                    "vessel": vname,
                    "cmd": vessel.cmd.tolist(),
                    "dof_available": vessel.allocator.dof_available.tolist(),
                }))
        except (websockets.exceptions.ConnectionClosed, asyncio.CancelledError):
            pass

    async def _ws_handler(self, websocket):
        touched = set()
        current = {"vessel": None}
        sender_task = asyncio.ensure_future(self._telemetry_sender(websocket, current))
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if msg.get("type") != "key":
                    continue
                vname = msg.get("vessel")
                code = msg.get("code")
                state = msg.get("state")
                vessel = self.vessels.get(vname)
                if vessel is None or not code:
                    continue

                touched.add(vname)
                current["vessel"] = vname

                old_socket = self.sessions.get(vname)
                if old_socket is not None and old_socket is not websocket:
                    try:
                        await old_socket.close()
                    except Exception:
                        pass
                self.sessions[vname] = websocket

                vessel.set_key(code, state == "down")
        finally:
            sender_task.cancel()
            for vname in touched:
                if self.sessions.get(vname) is websocket:
                    del self.sessions[vname]
                vessel = self.vessels.get(vname)
                if vessel is not None:
                    vessel.clear_keys()

    def run_ws_server(self, ws_port: int):
        import websockets

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def _serve():
            async with websockets.serve(self._ws_handler, "0.0.0.0", ws_port):
                await asyncio.Future()  # run forever

        loop.run_until_complete(_serve())

    # ------------------------------------------------------------------
    # Flask app (page + config)
    # ------------------------------------------------------------------

    def build_flask_app(self, ws_port: int) -> Flask:
        app = Flask(__name__)

        @app.route("/")
        def index():
            return Response(_PAGE_HTML.replace("__WS_PORT__", str(ws_port)), content_type="text/html")

        @app.route("/api/config")
        def api_config():
            return jsonify({
                "vessels": {
                    name: {"dof_available": v.allocator.dof_available.tolist()}
                    for name, v in self.vessels.items()
                },
                "dof_labels": list(DOF_LABELS),
            }), 200

        return app


_PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>mavsim Teleop</title>
<style>
  :root {
    --bg:#0f1117; --surface:#1a1d27; --surface2:#23273a; --border:#2d3348;
    --text:#e1e4ed; --text2:#8b91a8; --accent:#4f8ff7;
    --danger:#e5484d; --success:#30a46c; --warning:#f5a623;
    --radius:8px; --font:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
    --mono:'SF Mono','Fira Code','Consolas',monospace;
  }
  *{margin:0;padding:0;box-sizing:border-box;}
  body{font-family:var(--font);background:var(--bg);color:var(--text);min-height:100vh;}
  header{background:var(--surface);border-bottom:1px solid var(--border);padding:14px 24px;
    display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:10px;}
  header h1{font-size:1.15rem;font-weight:600;}
  .status-badge{padding:4px 12px;border-radius:12px;font-size:.8rem;font-weight:500;}
  .status-badge.connected{background:rgba(48,164,108,.15);color:var(--success);}
  .status-badge.disconnected{background:rgba(229,72,77,.15);color:var(--danger);}
  select{padding:6px 10px;border:1px solid var(--border);border-radius:var(--radius);
    background:var(--bg);color:var(--text);font-size:.85rem;outline:none;}
  label.inline{font-size:.8rem;color:var(--text2);display:flex;align-items:center;gap:6px;}
  main{max-width:900px;margin:0 auto;padding:24px;display:flex;flex-direction:column;gap:20px;}
  .panel{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:20px;}
  .panel h2{font-size:.85rem;font-weight:600;color:var(--text2);text-transform:uppercase;
    letter-spacing:.5px;margin-bottom:14px;}
  .meter-row{display:flex;align-items:center;gap:12px;margin-bottom:10px;}
  .meter-row .lbl{width:60px;font-size:.82rem;color:var(--text2);}
  .meter-track{flex:1;height:16px;background:var(--bg);border:1px solid var(--border);
    border-radius:8px;position:relative;overflow:hidden;}
  .meter-track.unavailable{opacity:.3;}
  .meter-fill{position:absolute;top:0;bottom:0;left:50%;background:var(--accent);}
  .meter-mid{position:absolute;top:0;bottom:0;left:50%;width:1px;background:var(--border);}
  .meter-val{width:52px;text-align:right;font-family:var(--mono);font-size:.78rem;color:var(--text2);}
  .keys{font-size:.8rem;color:var(--text2);line-height:1.9;}
  .keys kbd{background:var(--surface2);border:1px solid var(--border);border-radius:4px;
    padding:1px 7px;font-family:var(--mono);color:var(--text);}
  .empty-state{color:var(--text2);font-size:.85rem;text-align:center;padding:20px;}
</style>
</head>
<body>
<header>
  <div><h1>mavsim Teleop</h1></div>
  <div style="display:flex;align-items:center;gap:14px;">
    <label class="inline" id="vesselRow" style="display:none">Vessel <select id="vesselSelect" onchange="onVesselChange()"></select></label>
    <span id="wsStatus" class="status-badge disconnected">connecting&hellip;</span>
  </div>
</header>
<main>
  <div class="panel">
    <h2>Key layout</h2>
    <div class="keys">
      <div><kbd>&uarr;</kbd> <kbd>&darr;</kbd> surge &middot; <kbd>&larr;</kbd> <kbd>&rarr;</kbd> sway</div>
      <div><kbd>W</kbd> <kbd>S</kbd> heave &middot; <kbd>A</kbd> <kbd>D</kbd> yaw</div>
      <div><kbd>Q</kbd> <kbd>E</kbd> roll &middot; <kbd>R</kbd> <kbd>F</kbd> pitch</div>
    </div>
  </div>
  <div class="panel">
    <h2>Commanded DOF (ramped, -1..1)</h2>
    <div id="meters"><div class="empty-state">Waiting for vessel config&hellip;</div></div>
  </div>
</main>
<script>
const $ = (s) => document.querySelector(s);
const WS_PORT = __WS_PORT__;
const DOF_LABELS = ['Surge','Sway','Heave','Roll','Pitch','Yaw'];
const KEY_CODES = ['ArrowUp','ArrowDown','ArrowLeft','ArrowRight',
                   'KeyW','KeyS','KeyA','KeyD','KeyQ','KeyE','KeyR','KeyF'];

let ws = null;
let currentVessel = null;
let vesselInfo = {};
const heldKeys = new Set();

function buildMeters(dofAvailable) {
  const wrap = $('#meters');
  wrap.innerHTML = '';
  for (let i = 0; i < 6; i++) {
    const row = document.createElement('div');
    row.className = 'meter-row';
    const avail = dofAvailable ? !!dofAvailable[i] : true;
    row.innerHTML = '<div class="lbl">' + DOF_LABELS[i] + '</div>'
      + '<div class="meter-track' + (avail ? '' : ' unavailable') + '" id="track-' + i + '">'
      + '<div class="meter-mid"></div><div class="meter-fill" id="fill-' + i + '"></div></div>'
      + '<div class="meter-val" id="val-' + i + '">0.00</div>';
    wrap.appendChild(row);
  }
}

function updateMeters(cmd) {
  for (let i = 0; i < 6; i++) {
    const v = Math.max(-1, Math.min(1, cmd[i] || 0));
    const fill = $('#fill-' + i);
    if (!fill) continue;
    const pct = Math.abs(v) * 50;
    fill.style.width = pct + '%';
    fill.style.left = v >= 0 ? '50%' : (50 - pct) + '%';
    $('#val-' + i).textContent = v.toFixed(2);
  }
}

async function loadConfig() {
  const res = await fetch('/api/config');
  const data = await res.json();
  vesselInfo = data.vessels || {};
  const names = Object.keys(vesselInfo);
  const sel = $('#vesselSelect');
  sel.innerHTML = names.map((n) => '<option value="' + n + '">' + n + '</option>').join('');
  $('#vesselRow').style.display = names.length > 1 ? '' : 'none';
  currentVessel = names[0] || null;
  buildMeters(currentVessel ? vesselInfo[currentVessel].dof_available : null);
}

function onVesselChange() {
  currentVessel = $('#vesselSelect').value;
  buildMeters(vesselInfo[currentVessel] ? vesselInfo[currentVessel].dof_available : null);
}

function sendKey(code, state) {
  if (!ws || ws.readyState !== WebSocket.OPEN || !currentVessel) return;
  ws.send(JSON.stringify({ type: 'key', vessel: currentVessel, code, state }));
}

document.addEventListener('keydown', (e) => {
  if (!KEY_CODES.includes(e.code)) return;
  e.preventDefault();
  if (heldKeys.has(e.code)) return; // ignore browser key-repeat
  heldKeys.add(e.code);
  sendKey(e.code, 'down');
});
document.addEventListener('keyup', (e) => {
  if (!KEY_CODES.includes(e.code)) return;
  e.preventDefault();
  heldKeys.delete(e.code);
  sendKey(e.code, 'up');
});
window.addEventListener('blur', () => {
  // Losing window focus is the browser-side analog of a dead-man release -
  // stop implying any key is still held so a background tab can't keep
  // commanding actuators.
  for (const code of Array.from(heldKeys)) sendKey(code, 'up');
  heldKeys.clear();
});

function connect() {
  ws = new WebSocket('ws://' + location.hostname + ':' + WS_PORT);
  ws.onopen = () => { $('#wsStatus').textContent = 'connected'; $('#wsStatus').className = 'status-badge connected'; };
  ws.onclose = () => { $('#wsStatus').textContent = 'reconnecting…'; $('#wsStatus').className = 'status-badge disconnected'; setTimeout(connect, 2000); };
  ws.onerror = () => { try { ws.close(); } catch (e) {} };
  ws.onmessage = (ev) => {
    let msg; try { msg = JSON.parse(ev.data); } catch (e) { return; }
    if (msg.type === 'state' && msg.vessel === currentVessel) updateMeters(msg.cmd || []);
  };
}

loadConfig().then(connect);
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="mavsim Keyboard Teleop Node")
    parser.add_argument("--http-port", type=int, default=8900, help="Teleop page port (default: 8900)")
    parser.add_argument("--ws-port", type=int, default=8901, help="Teleop key/telemetry WebSocket port (default: 8901)")
    parser.add_argument("--config-file", default=DEFAULT_CONFIG_FILE,
                         help=f"Path to per-vessel thruster/control-surface config JSON (default: {DEFAULT_CONFIG_FILE})")
    parser.add_argument("--rate", type=float, default=20.0, help="Command loop / telemetry rate in Hz (default: 20.0)")
    parser.add_argument("--ramp-seconds", type=float, default=0.5,
                         help="Seconds to ramp a DOF from 0 to +-1 while a key is held (default: 0.5)")
    args = parser.parse_args()

    server = TeleopServer(args.config_file, rate=args.rate, ramp_seconds=args.ramp_seconds)

    threading.Thread(target=server.run_command_loop, daemon=True).start()
    threading.Thread(target=server.run_ws_server, args=(args.ws_port,), daemon=True).start()

    app = server.build_flask_app(args.ws_port)
    logger.info("Teleop page on port %d, WS on port %d", args.http_port, args.ws_port)
    app.run(host="0.0.0.0", port=args.http_port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
