#!/usr/bin/env python3
"""
mavsim Local ROS2 Topic Visualizer
===================================

A standalone, read-only Flask app that serves a browser page for inspecting
the *local* ROS2 topics republished inside this container (vessel state time
histories, camera feed, lidar point cloud, and a camera+lidar overlay). It has
no start/stop controls of its own - it's launched as a subprocess of
BaseController (examples/base_controller.py, `_launch_visualizer_server`)
alongside the rosbridge websocket (`_launch_rosbridge`), in every bridge mode
(CLI/web/token), so it's available even when the user has no rviz2/X11 - only
a browser.

The page talks directly to rosbridge over its own WebSocket connection (a
hand-rolled minimal client - see ROSBRIDGE_CLIENT_JS below - not roslibjs) for
live topic data, and to this server only for two things: the static page
itself and a one-time snapshot of sensor extrinsics/intrinsics (mounting
location/orientation, camera fov/resolution) needed for the overlay's pinhole
projection math. That snapshot is written by BaseController right after
handshake (`_fetch_and_cache_sensor_config`) to STATE_DIR, following the same
shared-/tmp-directory pattern already used for camera preview frames
(CAMERA_FRAME_DIR in bridge_controller.py / bridge_webapp.py).
"""

import argparse
import json
import logging
from pathlib import Path

from flask import Flask, Response, jsonify, send_from_directory

STATE_DIR = Path("/tmp/mavsim_bridge_state")
SENSOR_CONFIG_FILE = STATE_DIR / "sensor_config.json"
STATIC_DIR = Path("/app/static")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("visualizer_server")

# static_folder=None disables Flask's built-in /static route, which is derived
# from this file's own location and would otherwise shadow the static_files()
# view below. That only works while the script happens to live next to the
# assets: run the same file from anywhere else and every /static request 404s
# even though STATIC_DIR is an absolute path.
app = Flask(__name__, static_folder=None)

_rosbridge_port = 9090


@app.route("/")
def index():
    return Response(
        _PAGE_HTML.replace("__ROSBRIDGE_PORT__", str(_rosbridge_port)),
        content_type="text/html",
    )


@app.route("/static/<path:filename>")
def static_files(filename):
    return send_from_directory(str(STATIC_DIR), filename)


@app.route("/api/sensor_config")
def sensor_config():
    if not SENSOR_CONFIG_FILE.exists():
        return jsonify({}), 200
    try:
        return jsonify(json.loads(SENSOR_CONFIG_FILE.read_text())), 200
    except (json.JSONDecodeError, OSError) as e:
        logger.warning("Failed to read sensor config snapshot: %s", e)
        return jsonify({}), 200


_PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>mavsim ROS2 Visualizer</title>
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
  .tabs{display:flex;gap:4px;padding:12px 24px 0;border-bottom:1px solid var(--border);background:var(--surface);}
  .tab{padding:10px 18px;font-size:.88rem;font-weight:500;cursor:pointer;color:var(--text2);
    border-bottom:2px solid transparent;}
  .tab.active{color:var(--accent);border-bottom-color:var(--accent);}
  main{max-width:1400px;margin:0 auto;padding:24px;}
  .view{display:none;}
  .view.active{display:block;}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:18px;}
  .panel{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:16px;}
  .panel h3{font-size:.85rem;font-weight:600;color:var(--text2);text-transform:uppercase;
    letter-spacing:.5px;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center;}
  .legend{font-size:.72rem;color:var(--text2);display:flex;gap:10px;flex-wrap:wrap;margin-top:6px;}
  .legend span{display:inline-flex;align-items:center;gap:4px;}
  .legend .sw{width:10px;height:10px;border-radius:2px;display:inline-block;}
  .toolbar{display:flex;align-items:center;gap:12px;margin-bottom:12px;flex-wrap:wrap;}
  .empty-state{color:var(--text2);font-size:.85rem;text-align:center;padding:30px;}
  .cam-view{background:var(--bg);border:1px solid var(--border);border-radius:var(--radius);
    overflow:hidden;text-align:center;position:relative;min-height:120px;}
  .cam-view img{max-width:100%;display:block;margin:0 auto;}
  .cam-view canvas.overlay{position:absolute;top:0;left:0;pointer-events:none;}
  #pcContainer{width:100%;height:520px;background:var(--bg);border:1px solid var(--border);
    border-radius:var(--radius);position:relative;overflow:hidden;}
  #pcContainer .hint{position:absolute;bottom:8px;left:10px;font-size:.72rem;color:var(--text2);
    background:rgba(15,17,23,.7);padding:4px 8px;border-radius:4px;}
  label.inline{font-size:.8rem;color:var(--text2);display:flex;align-items:center;gap:6px;}
  .readout{font-family:var(--mono);font-size:.82rem;line-height:1.7;}

  /* ---- Topic Inspector ---- */
  .insp-layout{display:grid;grid-template-columns:300px 1fr;gap:18px;align-items:start;}
  @media (max-width:1000px){.insp-layout{grid-template-columns:1fr;}}
  .insp-meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-bottom:14px;}
  .insp-meta .cell{background:var(--bg);border:1px solid var(--border);border-radius:var(--radius);padding:8px 12px;}
  .insp-meta .k{font-size:.68rem;color:var(--text2);text-transform:uppercase;letter-spacing:.5px;}
  .insp-meta .v{font-family:var(--mono);font-size:.85rem;margin-top:3px;word-break:break-all;}
  .insp-meta .v.big{font-size:1.05rem;color:var(--accent);}
  .field-list{max-height:420px;overflow-y:auto;border:1px solid var(--border);border-radius:var(--radius);
    background:var(--bg);padding:6px;}
  .field-list .fl-row{display:flex;align-items:center;gap:7px;padding:3px 5px;border-radius:4px;font-size:.78rem;
    font-family:var(--mono);cursor:pointer;}
  .field-list .fl-row:hover{background:var(--surface2);}
  .field-list .fl-row input{accent-color:var(--accent);cursor:pointer;flex-shrink:0;margin:0;}
  .field-list .fl-name{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;}
  .field-list .fl-type{color:var(--text2);font-size:.68rem;flex-shrink:0;}
  .field-list .fl-derived{color:var(--warning);}
  .field-list .fl-plottable .fl-name{color:var(--text);}
  .field-list .fl-row.nonplot .fl-name{color:var(--text2);}
  .fl-group{font-size:.68rem;color:var(--text2);text-transform:uppercase;letter-spacing:.5px;
    padding:8px 5px 3px;border-top:1px solid var(--border);margin-top:5px;}
  .fl-group:first-child{border-top:none;margin-top:0;}
  .mini-btn{padding:4px 9px;font-size:.72rem;background:var(--surface2);color:var(--text);
    border:1px solid var(--border);border-radius:5px;cursor:pointer;}
  .mini-btn:hover{background:var(--border);}
  .mini-btn.on{background:var(--accent);border-color:var(--accent);color:#fff;}
  #rawTerm{background:#0a0c12;border:1px solid var(--border);border-radius:var(--radius);padding:10px 12px;
    font-family:var(--mono);font-size:.75rem;line-height:1.55;height:300px;overflow-y:auto;
    white-space:pre-wrap;word-break:break-word;color:var(--text2);user-select:text;}
  #rawTerm .rl{border-bottom:1px solid rgba(45,51,72,.5);padding:3px 0;}
  #rawTerm .rl:last-child{border-bottom:none;}
  #rawTerm .rt{color:var(--accent);}
  #rawTerm .rk{color:var(--text2);}
  #rawTerm .rv{color:var(--success);}
  #rawTerm .rs{color:var(--warning);}
  canvas.plot{width:100%;height:300px;display:block;background:var(--bg);border:1px solid var(--border);border-radius:var(--radius);}
  .num-in{width:64px;padding:4px 7px;border:1px solid var(--border);border-radius:5px;
    background:var(--bg);color:var(--text);font-size:.8rem;outline:none;font-family:var(--mono);}
  .insp-hint{font-size:.72rem;color:var(--text2);margin-top:8px;}

  /* ---- Time Histories: one block of 4 odometry plots per vessel ---- */
  .vessel-block{margin-bottom:26px;}
  .vessel-block > h2{font-size:.95rem;font-weight:600;margin-bottom:12px;display:flex;
    align-items:center;gap:10px;}
  .vessel-block > h2 .vname{font-family:var(--mono);color:var(--accent);}
  .vessel-block > h2 .vrate{font-size:.72rem;color:var(--text2);font-weight:400;}
  /* Fixed 2x2 so each vessel reads as one block of four related plots;
     auto-fit would reflow to 3+1 on a wide screen and break that pairing. */
  .odo-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px;}
  @media (max-width:900px){.odo-grid{grid-template-columns:1fr;}}
  canvas.odo{width:100%;height:210px;display:block;background:var(--bg);
    border:1px solid var(--border);border-radius:var(--radius);}
  /* The panel h3 is uppercased and letter-spaced, which mangles unit
     symbols ("°" and "m/s"); opt this span out of both. */
  .odo-unit{font-weight:400;color:var(--text2);text-transform:none;
    letter-spacing:0;font-family:var(--mono);font-size:.78rem;margin-left:6px;}
</style>
</head>
<body>
<header>
  <div><h1>mavsim ROS2 Visualizer</h1></div>
  <div style="display:flex;align-items:center;gap:14px;">
    <label class="inline">Vessel <select id="vesselSelect" onchange="onVesselChange()"></select></label>
    <span id="rosStatus" class="status-badge disconnected">rosbridge: connecting&hellip;</span>
  </div>
</header>
<div class="tabs">
  <div class="tab active" data-view="history" onclick="setView('history')">Time Histories</div>
  <div class="tab" data-view="inspector" onclick="setView('inspector')">Topic Inspector</div>
  <div class="tab" data-view="camera" onclick="setView('camera')">Camera</div>
  <div class="tab" data-view="pointcloud" onclick="setView('pointcloud')">Point Cloud</div>
  <div class="tab" data-view="overlay" onclick="setView('overlay')">Overlay</div>
</div>
<main>
  <div class="view active" id="view-history">
    <div class="toolbar">
      <span class="cam-stat">Odometry for all accessible vessels &middot; frames NED / BODY</span>
      <label class="inline" style="margin-left:auto;">Window
        <input type="number" class="num-in" id="histWindow" value="20" min="1" max="600" step="1"
               onchange="onHistWindowChange()"> s
      </label>
      <button class="mini-btn" id="histPauseBtn" onclick="toggleHistPause()">Pause</button>
    </div>
    <div id="historyGrid"><div class="empty-state">Waiting for odometry&hellip;</div></div>
  </div>
  <div class="view" id="view-inspector">
    <div class="toolbar">
      <label class="inline" style="flex:1;min-width:280px;">Topic
        <select id="inspTopicSelect" onchange="onInspectorTopicChange()" style="flex:1;min-width:240px;"></select>
      </label>
      <label class="inline"><input type="checkbox" id="inspAllVessels" onchange="populateInspectorTopics()"> Show internal topics</label>
      <button class="mini-btn" onclick="refreshInspectorTopics()">Refresh</button>
    </div>
    <div class="insp-meta">
      <div class="cell"><div class="k">Topic</div><div class="v" id="inspName">&ndash;</div></div>
      <div class="cell"><div class="k">Type</div><div class="v" id="inspType">&ndash;</div></div>
      <div class="cell"><div class="k">Rate</div><div class="v big" id="inspRate">&ndash;</div></div>
      <div class="cell"><div class="k">Messages</div><div class="v" id="inspCount">0</div></div>
    </div>
    <div class="insp-layout">
      <div class="panel">
        <h3>Fields
          <span style="display:flex;gap:5px;">
            <button class="mini-btn" onclick="inspSetAllFields(true)">All</button>
            <button class="mini-btn" onclick="inspSetAllFields(false)">None</button>
          </span>
        </h3>
        <div class="field-list" id="inspFieldList">
          <div class="empty-state">Waiting for first message&hellip;</div>
        </div>
        <div class="insp-hint">Checked fields appear in the raw feed and are available to plot.</div>
      </div>
      <div style="display:flex;flex-direction:column;gap:18px;min-width:0;">
        <div class="panel">
          <h3>Raw Message
            <span style="display:flex;gap:5px;align-items:center;">
              <button class="mini-btn" id="rawPauseBtn" onclick="toggleRawPause()">Pause</button>
              <button class="mini-btn on" id="rawScrollBtn" onclick="toggleRawScroll()">Autoscroll</button>
              <button class="mini-btn" onclick="clearRawTerm()">Clear</button>
            </span>
          </h3>
          <div id="rawTerm" readonly></div>
          <div class="insp-hint">Read-only feed, throttled to <span id="rawThrottleLabel">10</span>/s &middot; last 200 messages retained.</div>
        </div>
        <div class="panel">
          <h3>Plot
            <span style="display:flex;gap:8px;align-items:center;">
              <label class="inline">Window
                <input type="number" class="num-in" id="plotWindow" value="20" min="1" max="600" step="1"
                       onchange="onPlotWindowChange()"> s
              </label>
              <button class="mini-btn" id="plotPauseBtn" onclick="togglePlotPause()">Pause</button>
            </span>
          </h3>
          <canvas class="plot" id="inspPlot"></canvas>
          <div class="legend" id="inspPlotLegend"></div>
          <div class="insp-hint" id="inspPlotHint">Check numeric fields on the left to plot them.</div>
        </div>
      </div>
    </div>
  </div>
  <div class="view" id="view-camera">
    <div class="toolbar">
      <label class="inline">Camera <select id="cameraSelect" onchange="onCameraChange()"></select></label>
      <span class="legend" id="camStat"></span>
    </div>
    <div class="cam-view" id="camView"><div class="empty-state">Select a camera topic&hellip;</div></div>
  </div>
  <div class="view" id="view-pointcloud">
    <div class="toolbar">
      <label class="inline">Lidar <select id="lidarSelect" onchange="onLidarChange()"></select></label>
      <label class="inline">Color by
        <select id="pcColorMode" onchange="pcViewer&amp;&amp;pcViewer.render()">
          <option value="distance">Distance</option>
          <option value="intensity">Intensity</option>
        </select>
      </label>
      <label class="inline"><input type="checkbox" id="pcShowRays" onchange="pcViewer&amp;&amp;pcViewer.setShowRays(this.checked)"> Show Ray Directions</label>
      <span class="legend" id="pcStat"></span>
    </div>
    <div id="pcContainer"><div class="hint">Drag to orbit &middot; scroll to zoom</div></div>
  </div>
  <div class="view" id="view-overlay">
    <div class="toolbar">
      <label class="inline">Camera <select id="ovCameraSelect" onchange="onOverlayChange()"></select></label>
      <label class="inline">Lidar <select id="ovLidarSelect" onchange="onOverlayChange()"></select></label>
      <span class="legend" id="ovStat"></span>
    </div>
    <div class="cam-view" id="ovView"><div class="empty-state">Select a camera and a lidar topic&hellip;</div></div>
  </div>
</main>
<script src="/static/three.min.js"></script>
<script>
const $ = (s) => document.querySelector(s);
const ROSBRIDGE_PORT = __ROSBRIDGE_PORT__;
let sensorConfig = {};
let currentVessel = null;

// ---------------------------------------------------------------------
// Minimal rosbridge protocol client (no roslibjs). Protocol is plain JSON
// over WebSocket: {op:"subscribe",topic,type} to subscribe, and
// {op:"publish",topic,msg} received per message. uint8[] fields
// (CompressedImage.data, PointCloud2.data) arrive base64-encoded in msg,
// which is how rosbridge_suite always serializes byte arrays.
// ---------------------------------------------------------------------
class RosBridgeClient {
  constructor(url, onStatusChange) {
    this.url = url;
    this.topics = new Map(); // topic -> {type, callbacks:Set}
    this.connected = false;
    this.onStatusChange = onStatusChange || (() => {});
    this._connect();
  }
  _connect() {
    try { this.ws = new WebSocket(this.url); } catch (e) { this._scheduleReconnect(); return; }
    this.ws.onopen = () => {
      this.connected = true; this.onStatusChange(true);
      for (const [topic, { type }] of this.topics) this._sendSubscribe(topic, type);
    };
    this.ws.onclose = () => { this.connected = false; this.onStatusChange(false); this._scheduleReconnect(); };
    this.ws.onerror = () => { try { this.ws.close(); } catch (e) {} };
    this.ws.onmessage = (ev) => {
      let msg; try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.op === 'publish') {
        const entry = this.topics.get(msg.topic);
        if (entry) entry.callbacks.forEach((cb) => { try { cb(msg.msg); } catch (e) { console.error(e); } });
      }
    };
  }
  _scheduleReconnect() { setTimeout(() => this._connect(), 2000); }
  _sendSubscribe(topic, type) {
    if (this.connected) this.ws.send(JSON.stringify({ op: 'subscribe', topic, type, throttle_rate: 0 }));
  }
  subscribe(topic, type, cb) {
    if (!this.topics.has(topic)) this.topics.set(topic, { type, callbacks: new Set() });
    this.topics.get(topic).callbacks.add(cb);
    this._sendSubscribe(topic, type);
  }
  unsubscribe(topic, cb) {
    const entry = this.topics.get(topic);
    if (!entry) return;
    entry.callbacks.delete(cb);
    if (entry.callbacks.size === 0) {
      this.topics.delete(topic);
      if (this.connected) this.ws.send(JSON.stringify({ op: 'unsubscribe', topic }));
    }
  }
}

function decodePointCloud2(msg) {
  const bin = atob(msg.data);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const dv = new DataView(bytes.buffer);
  const little = !msg.is_bigendian;
  const off = {};
  for (const f of msg.fields) off[f.name] = f.offset;
  const n = msg.width * msg.height;
  const step = msg.point_step;
  const positions = new Float32Array(n * 3);
  const intensities = new Float32Array(n);
  const hasI = 'intensity' in off;
  for (let i = 0; i < n; i++) {
    const base = i * step;
    positions[i * 3 + 0] = dv.getFloat32(base + off.x, little);
    positions[i * 3 + 1] = dv.getFloat32(base + off.y, little);
    positions[i * 3 + 2] = dv.getFloat32(base + off.z, little);
    intensities[i] = hasI ? dv.getFloat32(base + off.intensity, little) : 0;
  }
  return { positions, intensities, count: n };
}

let ros = null;
function initRos() {
  ros = new RosBridgeClient(`ws://${location.hostname}:${ROSBRIDGE_PORT}`, (connected) => {
    const el = $('#rosStatus');
    el.textContent = connected ? 'rosbridge: connected' : 'rosbridge: reconnecting…';
    el.className = 'status-badge ' + (connected ? 'connected' : 'disconnected');
    if (connected) fetchLiveTopics(refreshAllViews);
  });
}

// ---------------------------------------------------------------------
// Live topic discovery via rosapi (/rosapi/topics). The camera/lidar topic
// ID a sensor actually publishes under doesn't always match the config's
// sensor_id (a pre-existing quirk in how the sensor bridge assigns lidar
// IDs at publish time) - so topic names are never guessed/constructed here,
// only ever discovered from the live ROS graph and then matched to their
// sensor_config.json metadata positionally (Nth camera topic <-> Nth Camera
// entry for that vessel, sorted by name - stable for a session's lifetime
// since sensors don't appear/disappear mid-session).
// ---------------------------------------------------------------------
let liveTopics = []; // [{name, type}]
let _rosApiReqId = 0;
function fetchLiveTopics(callback) {
  if (!ros || !ros.connected || !ros.ws) { if (callback) callback(); return; }
  const id = 'rosapi_topics_' + (_rosApiReqId++);
  let done = false;
  const handler = (ev) => {
    let msg; try { msg = JSON.parse(ev.data); } catch (e) { return; }
    if (msg.op === 'service_response' && msg.id === id) {
      done = true;
      ros.ws.removeEventListener('message', handler);
      const names = (msg.values && msg.values.topics) || [];
      const types = (msg.values && msg.values.types) || [];
      liveTopics = names.map((n, i) => ({ name: n, type: types[i] }));
      if (callback) callback();
    }
  };
  ros.ws.addEventListener('message', handler);
  ros.ws.send(JSON.stringify({ op: 'call_service', service: '/rosapi/topics', id, args: {} }));
  setTimeout(() => { if (!done) { ros.ws.removeEventListener('message', handler); if (callback) callback(); } }, 4000);
}

function liveTopicsForVessel(vessel, namePattern, typeSuffix) {
  const re = new RegExp('^/' + vessel + namePattern + '$');
  return liveTopics.filter((t) => re.test(t.name) && t.type && t.type.endsWith(typeSuffix))
    .sort((a, b) => a.name.localeCompare(b.name));
}

// Merge live-discovered topics with sensor_config.json metadata for the
// same sensor type, positionally (see comment above liveTopics).
function mergedSensorsForVessel(vessel, sensorTypeLower, namePattern, typeSuffix) {
  const configEntries = ((sensorConfig[vessel] && sensorConfig[vessel].sensors) || [])
    .filter((s) => (s.sensor_type || '').toLowerCase() === sensorTypeLower);
  const topics = liveTopicsForVessel(vessel, namePattern, typeSuffix);
  const out = [];
  // Only ever include positions with an actual live-discovered topic - the
  // config's own sensor_topic field is unreliable (see comment above
  // liveTopics) and must never be used as a fallback, including while a
  // sensor's ROS2 publisher hasn't been lazily created yet (it's created on
  // first frame, so it can legitimately not exist for the first few seconds
  // after the headless observer connects - the periodic refresh picks it up).
  for (let i = 0; i < topics.length; i++) {
    const meta = Object.assign({}, configEntries[i] || {});
    meta.sensor_topic = topics[i].name;
    out.push(meta);
  }
  return out;
}

const PALETTE = ['#4f8ff7', '#f5a623', '#30a46c', '#e5484d', '#a970ff', '#26c6da', '#ff7043', '#c0ca33'];
// Time Histories' odometry subscriptions. The camera/lidar/overlay views and
// the Topic Inspector each hold their own single subscription and unsubscribe
// on change, so this list is only ever torn down when the set of vessels
// changes and the history panels are rebuilt.
const activeSubs = []; // {topic, cb}

function clearSubs() { for (const { topic, cb } of activeSubs) ros.unsubscribe(topic, cb); activeSubs.length = 0; }
function sub(topic, type, cb) { ros.subscribe(topic, type, cb); activeSubs.push({ topic, cb }); }

function quatToEuler(x, y, z, w) {
  const sinr_cosp = 2 * (w * x + y * z), cosr_cosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
  const siny_cosp = 2 * (w * z + x * y), cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);
  return [roll, pitch, yaw].map((r) => r * 180 / Math.PI);
}


// ---------------------------------------------------------------------
// Time Histories: odometry for EVERY accessible vessel, four plots each.
//
// Deliberately not scoped to the vessel dropdown - comparing vessels is the
// main reason to look at this page, and /<vessel>/odometry_sim is published
// for every vessel the session exposes (owned or observed), so there is no
// reason to show only one. The dropdown still drives the camera/lidar tabs,
// which are inherently single-vessel.
//
// Plots use TimePlot (defined with the Topic Inspector below), which plots
// against a real time axis with labelled ticks. It replaced an earlier
// strip-chart that drew a bare centreline and plotted against sample index,
// so a reader could not tell what any value or time actually was.
// ---------------------------------------------------------------------

const HIST_MAX_POINTS = 20000;

// The four groupings, straight off nav_msgs/Odometry. Angles are converted
// to degrees; everything else is passed through in SI units.
const ODO_PLOTS = [
  {
    key: 'position', title: 'Position (NED)', unit: 'm',
    series: ['x', 'y', 'z'],
    extract: (m) => [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z],
  },
  {
    key: 'orientation', title: 'Orientation', unit: '°',
    series: ['roll', 'pitch', 'yaw'],
    extract: (m) => {
      const q = m.pose.pose.orientation;
      return quatToEuler(q.x, q.y, q.z, q.w);
    },
  },
  {
    key: 'linvel', title: 'Linear Velocity (BODY)', unit: 'm/s',
    series: ['u', 'v', 'w'],
    extract: (m) => [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z],
  },
  {
    key: 'angvel', title: 'Angular Velocity (BODY)', unit: '°/s',
    series: ['p', 'q', 'r'],
    extract: (m) => [
      m.twist.twist.angular.x * 180 / Math.PI,
      m.twist.twist.angular.y * 180 / Math.PI,
      m.twist.twist.angular.z * 180 / Math.PI,
    ],
  },
];

let histWindowSec = 20;
let histPaused = false;
let histT0 = null;            // shared clock origin, so vessels stay aligned
let histVessels = new Map();  // vessel -> {plots:[{plot,series}], rateStamps:[]}

// Every vessel publishing odometry, from the live ROS graph rather than from
// sensor_config.json - a vessel with no configured sensors still has
// odometry, and would otherwise be missing from this page entirely.
function odometryVessels() {
  const out = [];
  for (const t of liveTopics) {
    const m = t.name.match(/^\/([^/]+)\/odometry_sim$/);
    if (m && t.type && t.type.endsWith('Odometry')) out.push(m[1]);
  }
  return [...new Set(out)].sort();
}

function buildHistoryPanels() {
  const grid = $('#historyGrid');
  const vessels = odometryVessels();
  if (!vessels.length) {
    grid.innerHTML = '<div class="empty-state">Waiting for odometry&hellip;</div>';
    histVessels = new Map();
    return;
  }

  grid.innerHTML = '';
  histVessels = new Map();

  for (const vessel of vessels) {
    const block = document.createElement('div');
    block.className = 'vessel-block';
    block.innerHTML = `<h2><span class="vname">${escapeHtml(vessel)}</span>
      <span class="vrate" data-rate>&ndash;</span></h2>
      <div class="odo-grid"></div>`;
    grid.appendChild(block);
    const odoGrid = block.querySelector('.odo-grid');

    const plots = ODO_PLOTS.map((spec) => {
      const panel = document.createElement('div');
      panel.className = 'panel';
      const series = spec.series.map((n, i) => ({ name: n, color: PALETTE[i % PALETTE.length] }));
      panel.innerHTML = `<h3>${spec.title}<span class="odo-unit">${spec.unit}</span></h3>
        <canvas class="odo"></canvas>
        <div class="legend">${series.map((s) =>
          `<span><span class="sw" style="background:${s.color}"></span>${s.name}</span>`).join('')}</div>`;
      odoGrid.appendChild(panel);
      const plot = new TimePlot(panel.querySelector('canvas'));
      // One entry per channel, in the same {t,v,color} shape TimePlot.render
      // already consumes for the inspector.
      const channels = new Map(series.map((s) => [s.name, { t: [], v: [], color: s.color }]));
      return { spec, plot, channels };
    });

    const entry = { plots, rateStamps: [], rateEl: block.querySelector('[data-rate]') };
    histVessels.set(vessel, entry);

    const topic = `/${vessel}/odometry_sim`;
    sub(topic, 'nav_msgs/Odometry', (msg) => onOdometry(vessel, msg));
  }
}

function onOdometry(vessel, msg) {
  const entry = histVessels.get(vessel);
  if (!entry) return;

  const nowMs = performance.now();
  if (histT0 === null) histT0 = nowMs;
  const t = (nowMs - histT0) / 1000;

  entry.rateStamps.push(nowMs);
  while (entry.rateStamps.length && nowMs - entry.rateStamps[0] > 3000) entry.rateStamps.shift();

  if (histPaused) return;

  const cutoff = t - Math.max(histWindowSec, 1) * 1.5;
  for (const { spec, channels } of entry.plots) {
    let values;
    try {
      values = spec.extract(msg);
    } catch (e) {
      continue;  // a malformed/partial message must not kill the feed
    }
    spec.series.forEach((name, i) => {
      const ch = channels.get(name);
      const v = values[i];
      if (typeof v !== 'number' || !isFinite(v)) return;
      ch.t.push(t); ch.v.push(v);
      let drop = 0;
      while (drop < ch.t.length && ch.t[drop] < cutoff) drop++;
      if (ch.t.length - drop > HIST_MAX_POINTS) drop = ch.t.length - HIST_MAX_POINTS;
      if (drop > 0) { ch.t.splice(0, drop); ch.v.splice(0, drop); }
    });
  }
}

function renderHistory() {
  if (!histVessels.size) return;
  const tNow = histT0 === null ? 0 : (performance.now() - histT0) / 1000;
  for (const entry of histVessels.values()) {
    for (const { plot, channels } of entry.plots) {
      plot.render(channels, tNow, histWindowSec);
    }
    if (entry.rateEl) {
      const s = entry.rateStamps;
      const stale = s.length && performance.now() - s[s.length - 1] > 3000;
      const hz = s.length > 1 ? (s.length - 1) / ((s[s.length - 1] - s[0]) / 1000) : 0;
      entry.rateEl.textContent = (!s.length || stale) ? '0.0 Hz' : hz.toFixed(1) + ' Hz';
    }
  }
}

// Rebuild the history panels only when the set of odometry-publishing
// vessels actually changes. The 5s topic refresh calls this on every tick,
// and an unconditional rebuild would discard every vessel's accumulated
// history several times a minute.
function refreshHistoryIfVesselsChanged() {
  const vessels = odometryVessels();
  const current = [...histVessels.keys()];
  const same = vessels.length === current.length && vessels.every((v, i) => v === current[i]);
  if (same) return;
  clearSubs();
  buildHistoryPanels();
}

function onHistWindowChange() {
  const v = parseFloat($('#histWindow').value);
  if (isFinite(v) && v > 0) histWindowSec = v;
  renderHistory();
}

function toggleHistPause() {
  histPaused = !histPaused;
  const b = $('#histPauseBtn');
  b.textContent = histPaused ? 'Resume' : 'Pause';
  b.classList.toggle('on', histPaused);
}

// ---------------------------------------------------------------------
// Camera view
// ---------------------------------------------------------------------
function cameraTopicsForVessel() {
  if (!currentVessel) return [];
  return mergedSensorsForVessel(currentVessel, 'camera', '/camera_\\\\d+/image/compressed', 'CompressedImage');
}
function lidarTopicsForVessel() {
  if (!currentVessel) return [];
  return mergedSensorsForVessel(currentVessel, 'lidar', '/lidar_\\\\d+/points', 'PointCloud2');
}

let camSub = null, camFrameCount = 0, camFpsTick = performance.now();
function onCameraChange() {
  if (camSub) { ros.unsubscribe(camSub.topic, camSub.cb); camSub = null; }
  const idx = $('#cameraSelect').value;
  const cams = cameraTopicsForVessel();
  const view = $('#camView');
  if (idx === '' || !cams[idx]) { view.innerHTML = '<div class="empty-state">Select a camera topic…</div>'; return; }
  const s = cams[idx];
  view.innerHTML = '<img id="camImg" alt="">';
  const img = $('#camImg');
  const cb = (msg) => {
    img.src = 'data:image/jpeg;base64,' + msg.data;
    camFrameCount++;
    const now = performance.now();
    if (now - camFpsTick >= 1000) { $('#camStat').textContent = camFrameCount + ' FPS'; camFrameCount = 0; camFpsTick = now; }
  };
  ros.subscribe(s.sensor_topic, 'sensor_msgs/CompressedImage', cb);
  camSub = { topic: s.sensor_topic, cb };
}

// ---------------------------------------------------------------------
// Point cloud view (Three.js, vendored core build only - orbit controls
// are hand-rolled to avoid vendoring the separate OrbitControls addon).
// ---------------------------------------------------------------------
class PointCloudViewer {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f1117);
    this.camera = new THREE.PerspectiveCamera(60, this._aspect(), 0.02, 1000);
    // Points are in the lidar's own local frame: X=forward, Y=left, Z=DOWN
    // (established from live data - see LIDAR_TO_BODY below). This is a
    // Z-down viewer to match, not Three.js's default Y-up, so the camera's
    // up-vector is -Z and the grid lies in the XY plane rather than Three's
    // default XZ plane. Rendering this Z-down data in a Z-up viewer put the
    // sea above the horizon and terrain below it.
    this.camera.up.set(0, 0, -1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);
    const grid = new THREE.GridHelper(10, 10, 0x444466, 0x222233);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(1));
    this.geometry = new THREE.BufferGeometry();
    this.material = new THREE.PointsMaterial({ size: 0.04, vertexColors: true });
    this.points = new THREE.Points(this.geometry, this.material);
    this.scene.add(this.points);
    // Ray directions: one line segment per point, from the sensor origin
    // (0,0,0 in this local-frame viewer) out to the point itself - lets you
    // see at a glance whether returns are landing at a plausible range or,
    // e.g., all clustered right on top of the origin.
    this.rayGeometry = new THREE.BufferGeometry();
    this.rayMaterial = new THREE.LineBasicMaterial({ color: 0x00cccc, transparent: true, opacity: 0.35 });
    this.rayLines = new THREE.LineSegments(this.rayGeometry, this.rayMaterial);
    this.rayLines.visible = false;
    this.scene.add(this.rayLines);
    this.theta = Math.PI / 4; this.phi = Math.PI / 3; this.radius = 6;
    this.target = new THREE.Vector3(0, 0, 0);
    this._updateCamera();
    this._setupControls();
    this._lastPositions = null; this._lastIntensities = null;
    this._animate();
    window.addEventListener('resize', () => this.resize());
  }
  _aspect() { return this.container.clientWidth / Math.max(1, this.container.clientHeight); }
  _updateCamera() {
    // Z-down spherical orbit: phi is measured from -Z, which is the sensor's
    // UP axis in this frame, so phi<pi/2 keeps the eye above the scene
    // looking down. theta is swept in the XY plane.
    this.camera.position.set(
      this.target.x + this.radius * Math.sin(this.phi) * Math.cos(this.theta),
      this.target.y + this.radius * Math.sin(this.phi) * Math.sin(this.theta),
      this.target.z - this.radius * Math.cos(this.phi),
    );
    this.camera.lookAt(this.target);
  }
  _setupControls() {
    let dragging = false, lastX = 0, lastY = 0;
    const dom = this.renderer.domElement;
    dom.addEventListener('mousedown', (e) => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
    window.addEventListener('mouseup', () => { dragging = false; });
    window.addEventListener('mousemove', (e) => {
      if (!dragging) return;
      const dx = e.clientX - lastX, dy = e.clientY - lastY; lastX = e.clientX; lastY = e.clientY;
      // Grab-the-scene feel on both axes: the scene follows the cursor.
      // Drag right -> the eye swings left around the target. Drag down ->
      // the eye RISES, which is what makes the scene appear to move down;
      // lowering the eye on a downward drag looks backwards even though it
      // sounds right. (Z-down frame with up = -Z, so screen-right is
      // reversed relative to a Z-up orbit and phi is measured from -Z.)
      this.theta += dx * 0.01;
      this.phi = Math.max(0.05, Math.min(Math.PI - 0.05, this.phi - dy * 0.01));
      this._updateCamera();
    });
    dom.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.radius = Math.max(0.2, Math.min(80, this.radius * (1 + e.deltaY * 0.001)));
      this._updateCamera();
    }, { passive: false });
  }
  setPoints(positions, intensities, colorMode, maxRange) {
    this._lastPositions = positions; this._lastIntensities = intensities; this._lastMaxRange = maxRange;
    const n = positions.length / 3;
    const colors = new Float32Array(n * 3);
    // Points are in the sensor-local frame (LidarSensor.js), so distance is
    // just each point's own magnitude from the origin - same as LidarPIP's
    // 'distance' color mode (LidarPIP.js's setPoints()).
    //
    // Distance is normalized against the sensor's fixed configured
    // max_range (matching LidarPIP), NOT a per-frame auto min/max - a
    // single stray ray that hit nothing nearby and travelled out to
    // max_range would otherwise dominate the auto-computed range and
    // crush every normal, nearby point down to one end of the colormap
    // (this is exactly what "everything shows up blue" looked like).
    // Intensity keeps per-frame auto min/max, unchanged.
    let lo = Infinity, hi = -Infinity;
    const maxR = maxRange || 100;
    const value = (i) => {
      if (colorMode === 'intensity') return intensities[i];
      const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
      return Math.sqrt(x * x + y * y + z * z);
    };
    if (colorMode !== 'intensity') { lo = 0; hi = maxR; } else {
      for (let i = 0; i < n; i++) { const v = value(i); if (v < lo) lo = v; if (v > hi) hi = v; }
    }
    const range = Math.max(1e-6, hi - lo);
    for (let i = 0; i < n; i++) {
      const v = Math.max(0, Math.min(1, (value(i) - lo) / range));
      const [r, g, b] = this._colormap(v);
      colors[i * 3] = r; colors[i * 3 + 1] = g; colors[i * 3 + 2] = b;
    }
    this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.geometry.computeBoundingSphere();

    // Origin -> point line for every point (2 vertices each).
    const rayPositions = new Float32Array(n * 6);
    for (let i = 0; i < n; i++) {
      rayPositions[i * 6 + 0] = 0; rayPositions[i * 6 + 1] = 0; rayPositions[i * 6 + 2] = 0;
      rayPositions[i * 6 + 3] = positions[i * 3]; rayPositions[i * 6 + 4] = positions[i * 3 + 1]; rayPositions[i * 6 + 5] = positions[i * 3 + 2];
    }
    this.rayGeometry.setAttribute('position', new THREE.BufferAttribute(rayPositions, 3));
    this.rayGeometry.computeBoundingSphere();
  }
  setShowRays(enabled) { this.rayLines.visible = !!enabled; }
  _colormap(t) {
    // Simple blue -> green -> red gradient.
    if (t < 0.5) { const u = t * 2; return [0, u, 1 - u]; }
    const u = (t - 0.5) * 2; return [u, 1 - u, 0];
  }
  render() { if (this._lastPositions) this.setPoints(this._lastPositions, this._lastIntensities, $('#pcColorMode').value, this._lastMaxRange); }
  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    this.camera.aspect = w / Math.max(1, h); this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }
  _animate() { requestAnimationFrame(() => this._animate()); this.renderer.render(this.scene, this.camera); }
}

let pcViewer = null, pcSub = null, pcCount = 0;
// Rolling 1s window of scan-message arrival times, same approach as
// SensorStreamManager.getFrameRate() - lets the actual received rate be
// compared directly against the sensor's configured publish_rate (e.g.
// LidarPIP shows 10 Hz because it reads the sensor in-process; this page
// only sees whatever rosbridge/JSON-over-websocket actually delivers).
let pcFrameTimestamps = [];
function onLidarChange() {
  if (pcSub) { ros.unsubscribe(pcSub.topic, pcSub.cb); pcSub = null; }
  const idx = $('#lidarSelect').value;
  const lidars = lidarTopicsForVessel();
  if (idx === '' || !lidars[idx]) { $('#pcStat').textContent = ''; return; }
  // Three.js is vendored from /static; if it failed to load, degrade to a
  // message in this one view rather than throwing through every caller.
  if (typeof THREE === 'undefined') {
    $('#pcStat').textContent = 'three.js unavailable - point cloud disabled';
    return;
  }
  if (!pcViewer) pcViewer = new PointCloudViewer($('#pcContainer'));
  const s = lidars[idx];
  // Robust lookup - matches the defensive chain LidarSensor.js itself uses
  // (lidarConfig.max_range ?? lidarConfig.maxRange ?? type-specific default)
  // - and logged so a mismatch between the color scale and the sensor's
  // real configured range is immediately visible instead of silently
  // falling back to a guessed default that may not match reality.
  const lidarCfg = s.lidar_config || {};
  const maxRange = lidarCfg.max_range ?? lidarCfg.maxRange ?? s.max_range ?? 100;
  console.log(`[PointCloud] Using max_range=${maxRange}m for distance color scale (lidar_config=`, lidarCfg, ')');
  pcFrameTimestamps = [];
  const cb = (msg) => {
    const { positions, intensities, count } = decodePointCloud2(msg);
    pcViewer.setPoints(positions, intensities, $('#pcColorMode').value, maxRange);
    pcCount = count;
    const now = performance.now();
    pcFrameTimestamps.push(now);
    const cutoff = now - 1000;
    while (pcFrameTimestamps.length && pcFrameTimestamps[0] < cutoff) pcFrameTimestamps.shift();
    $('#pcStat').textContent = `${count} points @ ${pcFrameTimestamps.length} Hz (color range 0-${maxRange}m)`;
  };
  ros.subscribe(s.sensor_topic, 'sensor_msgs/PointCloud2', cb);
  pcSub = { topic: s.sensor_topic, cb };
}

// ---------------------------------------------------------------------
// Overlay: project lidar points into the camera image via a pinhole model,
// reusing Three.js's own camera/projection pipeline (an off-screen
// PerspectiveCamera configured with the real sensor's fov/resolution, and
// Vector3.project()) rather than hand-deriving intrinsics. Sensor mounting
// pose (sensor_location/sensor_orientation, both relative to the vessel
// body frame) is applied the same way Three.js applies object
// position/rotation elsewhere in this codebase (XYZ Euler order, degrees).
// See projectLidarToCamera() for the lidar-frame axis convention, which is
// NOT the same handedness as the frame those mounting poses are expressed
// in and has to be converted before the poses are composed.
// ---------------------------------------------------------------------
let ovCamSub = null, ovLidarSub = null, ovLatestCloud = null, ovLatestCamMsg = null;
function onOverlayChange() {
  if (ovCamSub) { ros.unsubscribe(ovCamSub.topic, ovCamSub.cb); ovCamSub = null; }
  if (ovLidarSub) { ros.unsubscribe(ovLidarSub.topic, ovLidarSub.cb); ovLidarSub = null; }
  ovLatestCloud = null; ovLatestCamMsg = null;
  const camIdx = $('#ovCameraSelect').value, lidarIdx = $('#ovLidarSelect').value;
  const cams = cameraTopicsForVessel(), lidars = lidarTopicsForVessel();
  const view = $('#ovView');
  if (camIdx === '' || lidarIdx === '' || !cams[camIdx] || !lidars[lidarIdx]) {
    view.innerHTML = '<div class="empty-state">Select a camera and a lidar topic…</div>'; return;
  }
  const camSensor = cams[camIdx], lidarSensor = lidars[lidarIdx];
  view.innerHTML = '<img id="ovImg" alt=""><canvas class="overlay" id="ovCanvas"></canvas>';
  const img = $('#ovImg'), canvas = $('#ovCanvas');
  const draw = () => {
    if (!ovLatestCamMsg) return;
    img.src = 'data:image/jpeg;base64,' + ovLatestCamMsg.data;
    if (!img.clientWidth || !img.clientHeight) return;
    const [rw, rh] = camSensor.resolution || [img.naturalWidth || 640, img.naturalHeight || 480];
    canvas.width = rw; canvas.height = rh;
    // Align the canvas to the IMAGE's box, not the container's. The image is
    // centred (margin:0 auto) and capped at max-width:100%, so whenever it is
    // narrower or shorter than the panel the two are offset - which drew
    // points out over the letterboxing beside the picture.
    canvas.style.width = img.clientWidth + 'px';
    canvas.style.height = img.clientHeight + 'px';
    canvas.style.left = img.offsetLeft + 'px';
    canvas.style.top = img.offsetTop + 'px';
    const ctx = canvas.getContext('2d'); ctx.clearRect(0, 0, rw, rh);
    if (!ovLatestCloud) return;
    const pts = projectLidarToCamera(camSensor, lidarSensor, ovLatestCloud, rw, rh);
    ctx.fillStyle = 'rgba(79,143,247,0.85)';
    for (const p of pts) { ctx.beginPath(); ctx.arc(p.x, p.y, 2, 0, Math.PI * 2); ctx.fill(); }
    $('#ovStat').textContent = pts.length + ' / ' + ovLatestCloud.count + ' points in frame';
  };
  // Redraw once a frame has actually been laid out: clientWidth/offsetLeft
  // are 0 until the first image loads, so the draw that set img.src cannot
  // itself position the canvas correctly.
  img.addEventListener('load', draw);
  ovCamSub = { topic: camSensor.sensor_topic, cb: (msg) => { ovLatestCamMsg = msg; draw(); } };
  ovLidarSub = { topic: lidarSensor.sensor_topic, cb: (msg) => { ovLatestCloud = decodePointCloud2(msg); draw(); } };
  ros.subscribe(ovCamSub.topic, 'sensor_msgs/CompressedImage', ovCamSub.cb);
  ros.subscribe(ovLidarSub.topic, 'sensor_msgs/PointCloud2', ovLidarSub.cb);
}

// Port of web_platform/frontend/src/utils/sensorOrientation.js's
// bodySensorOrientationToThreeRotation(): sensor_orientation is stored as
// [roll, pitch, yaw] in degrees, ZYX order, body frame - NOT a raw XYZ-order
// Euler triple. Converting via a rotation-matrix round-trip (ZYX Euler ->
// matrix -> re-extract as XYZ Euler) is required; applying the same three
// values directly as 'XYZ' order produces a different, wrong rotation.
function bodySensorOrientationToThreeEuler(orientation) {
  const ori = orientation || [0, 0, 0];
  const roll = THREE.MathUtils.degToRad(ori[0] || 0);
  const pitch = THREE.MathUtils.degToRad(ori[1] || 0);
  const yaw = THREE.MathUtils.degToRad(ori[2] || 0);
  const bodyEuler = new THREE.Euler(roll, pitch, yaw, 'ZYX');
  const bodyMatrix = new THREE.Matrix4().makeRotationFromEuler(bodyEuler);
  return new THREE.Euler().setFromRotationMatrix(bodyMatrix, 'XYZ');
}

function makePoseObject(sensor) {
  const obj = new THREE.Object3D();
  const loc = sensor.sensor_location || [0, 0, 0];
  obj.position.set(loc[0], loc[1], loc[2]);
  obj.setRotationFromEuler(bodySensorOrientationToThreeEuler(sensor.sensor_orientation));
  obj.updateMatrixWorld(true);
  return obj;
}

// PointCloud2 points arrive in the lidar's OWN local frame: X=forward,
// Y=left, Z=DOWN. The Z sense was established from live data rather than
// from LidarSensor.js's ray formula, whose comment claims Z=up:
//
//   water returns (<10m, the surface 0.2m below a mast-mounted sensor)
//       ->  z = +0.04 .. +0.20   i.e. BELOW the sensor is +z
//   cliff returns (>40m, terrain towering over the vessel)
//       ->  z = -4.80 .. -0.30   i.e. ABOVE the sensor is -z
//
// Read as Z=down those become a water surface 0.04-0.20m below the sensor
// and cliffs up to 4.8m above it, which matches the scene; read as Z=up
// they are inverted. The same sample puts far cliff returns at y<0 (34 of
// 36) while the cliff is on the RIGHT of the camera image, confirming
// +Y=left.
//
// The mounting poses are expressed in the vessel's NED body frame -
// X=forward, Y=right, Z=down - which the camera's [-90, 0, 90] mounting
// confirms (forward +X, up -Z, right +Y). So the two frames agree on X and
// Z and differ only in the sign of Y, and the conversion is a mirror in Y,
// not a rotation: X=fwd/Y=left/Z=down is left-handed, so this legitimately
// has determinant -1 and cannot be written as makeRotationX/Y/Z.
//
// Flipping Z as well (a 180-degree roll) inverts the vertical: the cliff
// tops hang below the horizon instead of standing above it.
const LIDAR_TO_BODY = new THREE.Matrix4().makeScale(1, -1, 1);

function projectLidarToCamera(camSensor, lidarSensor, cloud, imgW, imgH) {
  const lidarObj = makePoseObject(lidarSensor);
  const projCam = new THREE.PerspectiveCamera(camSensor.fov || 60, imgW / imgH, 0.05, 1000);
  const camPose = makePoseObject(camSensor);
  projCam.position.copy(camPose.position);
  projCam.quaternion.copy(camPose.quaternion);
  projCam.updateMatrixWorld(true);

  const out = [];
  const v = new THREE.Vector3();
  const local = new THREE.Vector3();
  const n = cloud.count;
  for (let i = 0; i < n; i++) {
    v.set(cloud.positions[i * 3], cloud.positions[i * 3 + 1], cloud.positions[i * 3 + 2]);
    // Axis convention first, then the mounting pose ONCE. localToWorld() was
    // previously applied to points that were already in the lidar's frame,
    // which added the lidar's own location/rotation a second time.
    v.applyMatrix4(LIDAR_TO_BODY);
    v.applyMatrix4(lidarObj.matrixWorld);
    local.copy(v);
    projCam.worldToLocal(local);
    if (local.z >= 0) continue; // behind the camera (Three.js looks down -Z)
    const ndc = v.clone().project(projCam);
    if (ndc.x < -1 || ndc.x > 1 || ndc.y < -1 || ndc.y > 1) continue;
    out.push({ x: (ndc.x * 0.5 + 0.5) * imgW, y: (1 - (ndc.y * 0.5 + 0.5)) * imgH });
  }
  return out;
}

// ---------------------------------------------------------------------
// Topic Inspector
// ---------------------------------------------------------------------
// A fully generic, type-agnostic topic browser: pick any live topic, see its
// type and measured publish rate, watch raw messages scroll by in a read-only
// terminal, and plot any numeric field against time on a sliding window.
//
// Nothing here is hardcoded per message type. The field tree is derived by
// walking the FIRST received message (rosbridge hands us plain JSON), so it
// works for interfaces/msg/Actuator just as well as for sensor_msgs/msg/Imu.
// Two conveniences are layered on top of that walk:
//   - uint8[] blobs (CompressedImage.data, PointCloud2.data) arrive as long
//     base64 strings; they're summarised rather than dumped, so selecting a
//     camera topic can't wedge the page.
//   - any object exposing x/y/z/w is additionally offered as derived
//     roll/pitch/yaw channels, since a quaternion is rarely what you actually
//     want to look at on a chart.
// ---------------------------------------------------------------------

const RAW_MAX_LINES = 200;        // messages retained in the terminal
const RAW_THROTTLE_HZ = 10;       // cap on terminal appends per second
const PLOT_MAX_POINTS = 20000;    // hard cap on retained samples per channel

// Internal/infrastructure topics: hidden unless "Show internal topics" is on.
const INSP_HIDDEN_TOPICS = /^\/(rosout|parameter_events|client_count|connected_clients)$/;

let inspTopic = null;          // currently inspected topic name
let inspType = null;           // its ROS type string
let inspSub = null;            // {topic, cb} for cleanup
let inspFields = new Map();    // path -> {path, type, plottable, derived, checked}
let inspFieldsBuilt = false;
let inspSeries = new Map();    // path -> {t:[], v:[], color}
let inspMsgCount = 0;
let inspRateStamps = [];       // arrival times (ms) for the rate estimate
let inspT0 = null;             // page-clock origin for plot x-axis
let inspLastMsg = null;
let rawPaused = false, rawAutoscroll = true, rawLastAppend = 0;
let plotPaused = false, plotWindowSec = 20;
let inspPlot = null;

// --- field tree ------------------------------------------------------

function isQuatLike(o) {
  return o && typeof o === 'object' && !Array.isArray(o) &&
         ['x', 'y', 'z', 'w'].every((k) => typeof o[k] === 'number');
}

// Walk a decoded message into a flat list of {path, type, plottable, derived}.
// `path` is a dotted/bracketed accessor ("orientation.x", "data[3]") that
// inspGetValue() below can resolve against any later message of the same type.
function flattenMessage(obj, prefix, out, depth) {
  depth = depth || 0;
  if (depth > 6) return out;
  for (const key of Object.keys(obj)) {
    const val = obj[key];
    const path = prefix ? `${prefix}.${key}` : key;
    if (val === null || val === undefined) {
      out.push({ path, type: 'null', plottable: false });
    } else if (typeof val === 'number') {
      // A float field that happens to hold 0.0/1.0 is still a float; the JSON
      // wire format loses that distinction, so don't infer "int" from value.
      out.push({ path, type: 'number', plottable: true });
    } else if (typeof val === 'boolean') {
      out.push({ path, type: 'bool', plottable: true });
    } else if (typeof val === 'string') {
      // Long strings are base64 uint8[] blobs in practice - summarise only.
      out.push({ path, type: val.length > 256 ? `bytes[~${val.length}]` : 'string', plottable: false });
    } else if (Array.isArray(val)) {
      if (val.length && typeof val[0] === 'number') {
        // Numeric array: expose each element, but cap how many get their own
        // row so a 4096-point scan doesn't produce 4096 checkboxes.
        const shown = Math.min(val.length, 64);
        for (let i = 0; i < shown; i++) {
          out.push({ path: `${path}[${i}]`, type: 'float', plottable: true });
        }
        if (val.length > shown) {
          out.push({ path: `${path}[…]`, type: `+${val.length - shown} more`, plottable: false });
        }
      } else if (val.length && typeof val[0] === 'object') {
        const shown = Math.min(val.length, 8);
        for (let i = 0; i < shown; i++) flattenMessage(val[i], `${path}[${i}]`, out, depth + 1);
        if (val.length > shown) {
          out.push({ path: `${path}[…]`, type: `+${val.length - shown} more`, plottable: false });
        }
      } else {
        out.push({ path, type: `array[${val.length}]`, plottable: false });
      }
    } else if (typeof val === 'object') {
      flattenMessage(val, path, out, depth + 1);
      // Offer euler angles alongside any quaternion-shaped submessage.
      if (isQuatLike(val)) {
        for (const ang of ['roll', 'pitch', 'yaw']) {
          out.push({ path: `${path}.${ang}°`, type: 'derived', plottable: true, derived: true });
        }
      }
    }
  }
  return out;
}

// Resolve a flattened path against a message. Handles the derived euler
// channels by recomputing them from the parent quaternion on the fly.
function inspGetValue(msg, path) {
  const m = path.match(/^(.*)\.(roll|pitch|yaw)°$/);
  if (m) {
    const q = inspGetValue(msg, m[1]);
    if (!isQuatLike(q)) return undefined;
    const [roll, pitch, yaw] = quatToEuler(q.x, q.y, q.z, q.w);
    return { roll, pitch, yaw }[m[2]];
  }
  let cur = msg;
  // Split "a.b[2].c" into ['a','b',2,'c'].
  for (const tok of path.split('.')) {
    const parts = tok.split(/[[\]]/).filter((s) => s !== '');
    for (const p of parts) {
      if (cur === null || cur === undefined) return undefined;
      cur = /^\d+$/.test(p) ? cur[Number(p)] : cur[p];
    }
  }
  return cur;
}

function buildFieldList(msg) {
  const flat = flattenMessage(msg, '', [], 0);
  flattenedHasEuler = new Set(flat.filter((f) => f.derived).map((f) => f.path));
  const next = new Map();
  for (const f of flat) {
    const prev = inspFields.get(f.path);
    next.set(f.path, Object.assign({}, f, {
      // Preserve the user's checkbox state across reconnects/refreshes.
      checked: prev ? prev.checked : defaultChecked(f),
    }));
  }
  inspFields = next;
  inspFieldsBuilt = true;
  renderFieldList();
  syncSeries();
}

// Sensible first view: plottable scalars on, header/covariance noise off.
function defaultChecked(f) {
  if (!f.plottable) return false;
  if (f.derived) return true;
  if (/^header\./.test(f.path)) return false;
  if (/covariance/.test(f.path)) return false;
  // A quaternion's raw components are redundant once euler is offered.
  if (/\.(x|y|z|w)$/.test(f.path) && inspHasEulerSibling(f.path)) return false;
  return true;
}

// Paths of the derived euler channels found in the current message type,
// populated by buildFieldList() before any defaultChecked() call reads it.
let flattenedHasEuler = new Set();

function inspHasEulerSibling(path) {
  return flattenedHasEuler.has(path.replace(/\.(x|y|z|w)$/, '') + '.roll°');
}

function renderFieldList() {
  const el = $('#inspFieldList');
  if (!inspFields.size) {
    el.innerHTML = '<div class="empty-state">Waiting for first message…</div>';
    return;
  }
  let html = '', lastGroup = null;
  for (const f of inspFields.values()) {
    // Group by the containing submessage, treating "foo[3]" as belonging to
    // group "foo" so a 9-element covariance reads as 0..8 under one heading
    // rather than nine identically-truncated rows under "(root)".
    const arr = f.path.match(/^(.*?)\[(.+)\]$/);
    let group, leaf;
    if (arr) {
      group = arr[1];
      leaf = `[${arr[2]}]`;
    } else if (f.path.includes('.')) {
      group = f.path.split('.').slice(0, -1).join('.');
      leaf = f.path.split('.').slice(-1)[0];
    } else {
      group = '(root)';
      leaf = f.path;
    }
    if (group !== lastGroup) { html += `<div class="fl-group">${escapeHtml(group)}</div>`; lastGroup = group; }
    html += `<label class="fl-row ${f.plottable ? 'fl-plottable' : 'nonplot'}" title="${escapeHtml(f.path)}">
      <input type="checkbox" data-path="${escapeHtml(f.path)}" ${f.checked ? 'checked' : ''}
             onchange="onFieldToggle(this)">
      <span class="fl-name ${f.derived ? 'fl-derived' : ''}">${escapeHtml(leaf)}</span>
      <span class="fl-type">${escapeHtml(f.type)}</span>
    </label>`;
  }
  el.innerHTML = html;
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

function onFieldToggle(cb) {
  const f = inspFields.get(cb.dataset.path);
  if (f) f.checked = cb.checked;
  syncSeries();
  renderPlot();
}

function inspSetAllFields(on) {
  for (const f of inspFields.values()) if (f.plottable || !on) f.checked = on && f.plottable;
  renderFieldList();
  syncSeries();
  renderPlot();
}

// Keep the plot's series set in step with the checked plottable fields,
// preserving already-collected history for channels that stay selected.
function syncSeries() {
  const wanted = [...inspFields.values()].filter((f) => f.checked && f.plottable).map((f) => f.path);
  for (const path of [...inspSeries.keys()]) if (!wanted.includes(path)) inspSeries.delete(path);
  wanted.forEach((path) => {
    if (!inspSeries.has(path)) inspSeries.set(path, { t: [], v: [] });
  });
  let i = 0;
  for (const s of inspSeries.values()) s.color = PALETTE[i++ % PALETTE.length];
  renderPlotLegend();
}

function renderPlotLegend() {
  const el = $('#inspPlotLegend');
  el.innerHTML = [...inspSeries.entries()]
    .map(([path, s]) => `<span><span class="sw" style="background:${s.color}"></span>${escapeHtml(path)}</span>`)
    .join('');
  $('#inspPlotHint').textContent = inspSeries.size
    ? `${inspSeries.size} channel(s) · ${plotWindowSec}s window`
    : 'Check numeric fields on the left to plot them.';
}

// --- raw terminal ----------------------------------------------------

// Render a message as a compact one-line-per-field block, honouring the
// field checkboxes so the feed only carries what the user asked for.
function formatRawMessage(msg) {
  const checked = [...inspFields.values()].filter((f) => f.checked);
  const rows = (checked.length ? checked : [...inspFields.values()]).map((f) => {
    let v = inspGetValue(msg, f.path);
    if (typeof v === 'number') v = Number.isInteger(v) ? String(v) : v.toFixed(6);
    else if (typeof v === 'string') v = v.length > 96 ? `<${f.type}>` : JSON.stringify(v);
    else if (v === undefined) v = '—';
    else v = JSON.stringify(v);
    return `<span class="rk">${escapeHtml(f.path)}</span>=<span class="rv">${escapeHtml(v)}</span>`;
  });
  return rows.join('<span class="rs">  </span>');
}

function appendRaw(msg) {
  if (rawPaused) return;
  const now = performance.now();
  if (now - rawLastAppend < 1000 / RAW_THROTTLE_HZ) return;
  rawLastAppend = now;
  const term = $('#rawTerm');
  const div = document.createElement('div');
  div.className = 'rl';
  const ts = new Date().toISOString().substr(11, 12);
  div.innerHTML = `<span class="rt">[${ts}]</span> ${formatRawMessage(msg)}`;
  term.appendChild(div);
  while (term.childElementCount > RAW_MAX_LINES) term.removeChild(term.firstChild);
  if (rawAutoscroll) term.scrollTop = term.scrollHeight;
}

function toggleRawPause() {
  rawPaused = !rawPaused;
  const b = $('#rawPauseBtn');
  b.textContent = rawPaused ? 'Resume' : 'Pause';
  b.classList.toggle('on', rawPaused);
}

function toggleRawScroll() {
  rawAutoscroll = !rawAutoscroll;
  $('#rawScrollBtn').classList.toggle('on', rawAutoscroll);
}

function clearRawTerm() { $('#rawTerm').innerHTML = ''; }

// --- time-series plot ------------------------------------------------

// Hand-rolled Canvas 2D plot with real, human-readable axis ticks and a
// sliding time window. Shared by the Topic Inspector and the Time Histories
// tab: both plot (t, value) pairs and need to show what the values and times
// actually are, which the sample-index strip chart this replaced could not.
class TimePlot {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this._resize();
    window.addEventListener('resize', () => { this._resize(); this.render(); });
  }
  _resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.floor(rect.width * this.dpr));
    this.canvas.height = Math.max(1, Math.floor(rect.height * this.dpr));
  }
  // "Nice" tick step: 1/2/5 x 10^n covering the range in ~target divisions.
  _niceStep(range, target) {
    if (!(range > 0)) return 1;
    const raw = range / target;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const norm = raw / mag;
    const step = norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 5 ? 5 : 10;
    return step * mag;
  }
  _fmt(v, step) {
    const dec = Math.max(0, Math.min(6, Math.ceil(-Math.log10(step)) + 1));
    if (Math.abs(v) >= 1e5 || (v !== 0 && Math.abs(v) < 1e-4)) return v.toExponential(1);
    return v.toFixed(dec);
  }
  render(series, tNow, windowSec) {
    const { ctx } = this;
    const W = this.canvas.width, H = this.canvas.height, d = this.dpr;
    const padL = 58 * d, padR = 12 * d, padT = 10 * d, padB = 26 * d;
    const pw = Math.max(1, W - padL - padR), ph = Math.max(1, H - padT - padB);
    ctx.clearRect(0, 0, W, H);
    ctx.font = `${10 * d}px 'SF Mono','Consolas',monospace`;
    ctx.textBaseline = 'middle';

    const t1 = tNow, t0 = tNow - windowSec;
    let lo = Infinity, hi = -Infinity;
    if (series) {
      for (const s of series.values()) {
        for (let i = 0; i < s.t.length; i++) {
          if (s.t[i] < t0) continue;
          const v = s.v[i];
          if (!isFinite(v)) continue;
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
      }
    }
    if (!isFinite(lo) || !isFinite(hi)) { lo = -1; hi = 1; }
    if (hi - lo < 1e-9) { const c = (hi + lo) / 2; lo = c - 0.5; hi = c + 0.5; }
    const padY = (hi - lo) * 0.1; lo -= padY; hi += padY;

    const X = (t) => padL + ((t - t0) / (t1 - t0)) * pw;
    const Y = (v) => padT + (1 - (v - lo) / (hi - lo)) * ph;

    // Grid + ticks
    const yStep = this._niceStep(hi - lo, 5);
    ctx.strokeStyle = '#2d3348'; ctx.fillStyle = '#8b91a8'; ctx.lineWidth = 1 * d;
    ctx.textAlign = 'right';
    for (let v = Math.ceil(lo / yStep) * yStep; v <= hi; v += yStep) {
      const y = Y(v);
      ctx.globalAlpha = 0.5;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + pw, y); ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.fillText(this._fmt(v, yStep), padL - 6 * d, y);
    }
    const tStep = this._niceStep(windowSec, 6);
    ctx.textAlign = 'center';
    for (let k = Math.ceil(t0 / tStep) * tStep; k <= t1; k += tStep) {
      const x = X(k);
      ctx.globalAlpha = 0.5;
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + ph); ctx.stroke();
      ctx.globalAlpha = 1;
      // Label relative to now: 0 at the right edge, negative into the past.
      ctx.fillText(`${(k - t1).toFixed(tStep < 1 ? 1 : 0)}s`, x, padT + ph + 13 * d);
    }
    // Axis frame
    ctx.globalAlpha = 1; ctx.strokeStyle = '#2d3348';
    ctx.strokeRect(padL, padT, pw, ph);

    if (!series || !series.size) {
      ctx.fillStyle = '#8b91a8'; ctx.textAlign = 'center';
      ctx.fillText('No channels selected', padL + pw / 2, padT + ph / 2);
      return;
    }
    // Series
    ctx.save();
    ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
    for (const s of series.values()) {
      ctx.strokeStyle = s.color; ctx.lineWidth = 1.5 * d;
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < s.t.length; i++) {
        if (s.t[i] < t0) continue;
        const v = s.v[i];
        if (!isFinite(v)) { started = false; continue; }
        const x = X(s.t[i]), y = Y(v);
        if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    ctx.restore();
  }
}

function renderPlot() {
  if (!inspPlot) return;
  const tNow = inspT0 === null ? 0 : (performance.now() - inspT0) / 1000;
  inspPlot.render(inspSeries, tNow, plotWindowSec);
}

function onPlotWindowChange() {
  const v = parseFloat($('#plotWindow').value);
  if (isFinite(v) && v > 0) plotWindowSec = v;
  renderPlotLegend();
  renderPlot();
}

function togglePlotPause() {
  plotPaused = !plotPaused;
  const b = $('#plotPauseBtn');
  b.textContent = plotPaused ? 'Resume' : 'Pause';
  b.classList.toggle('on', plotPaused);
}

// --- subscription + wiring -------------------------------------------

function onInspectorMessage(msg) {
  const nowMs = performance.now();
  if (inspT0 === null) inspT0 = nowMs;
  const t = (nowMs - inspT0) / 1000;

  inspMsgCount++;
  inspLastMsg = msg;

  // Rate over a 3s trailing window - steadier than an inter-arrival estimate.
  inspRateStamps.push(nowMs);
  while (inspRateStamps.length && nowMs - inspRateStamps[0] > 3000) inspRateStamps.shift();

  if (!inspFieldsBuilt) buildFieldList(msg);

  if (!plotPaused) {
    const cutoff = t - Math.max(plotWindowSec, 1) * 1.5;
    for (const [path, s] of inspSeries) {
      let v = inspGetValue(msg, path);
      if (typeof v === 'boolean') v = v ? 1 : 0;
      if (typeof v !== 'number' || !isFinite(v)) continue;
      s.t.push(t); s.v.push(v);
      // Trim to the retained window (plus slack) and the hard point cap.
      let drop = 0;
      while (drop < s.t.length && s.t[drop] < cutoff) drop++;
      if (s.t.length - drop > PLOT_MAX_POINTS) drop = s.t.length - PLOT_MAX_POINTS;
      if (drop > 0) { s.t.splice(0, drop); s.v.splice(0, drop); }
    }
  }
  appendRaw(msg);
}

function inspUnsubscribe() {
  if (inspSub) { ros.unsubscribe(inspSub.topic, inspSub.cb); inspSub = null; }
}

function onInspectorTopicChange() {
  inspUnsubscribe();
  const sel = $('#inspTopicSelect');
  const topic = sel.value;
  const entry = liveTopics.find((t) => t.name === topic);

  inspTopic = topic || null;
  inspType = entry ? entry.type : null;
  inspFields = new Map();
  inspFieldsBuilt = false;
  inspSeries = new Map();
  inspMsgCount = 0;
  inspRateStamps = [];
  inspT0 = null;
  inspLastMsg = null;
  clearRawTerm();
  renderFieldList();
  renderPlotLegend();
  renderPlot();

  $('#inspName').textContent = inspTopic || '–';
  $('#inspType').textContent = inspType || '–';
  $('#inspCount').textContent = '0';
  $('#inspRate').textContent = '–';
  if (!inspTopic || !inspType) return;

  const cb = (msg) => onInspectorMessage(msg);
  ros.subscribe(inspTopic, inspType, cb);
  inspSub = { topic: inspTopic, cb };
}

function populateInspectorTopics() {
  const sel = $('#inspTopicSelect');
  const showAll = $('#inspAllVessels').checked;
  const items = liveTopics
    .filter((t) => showAll || !INSP_HIDDEN_TOPICS.test(t.name))
    .slice()
    .sort((a, b) => a.name.localeCompare(b.name));
  const prev = inspTopic;
  sel.innerHTML = items.length
    ? items.map((t) => `<option value="${escapeHtml(t.name)}">${escapeHtml(t.name)}  —  ${escapeHtml(t.type || '?')}</option>`).join('')
    : '<option value="">No topics discovered yet</option>';
  // Keep the current selection across periodic refreshes; only (re)subscribe
  // when the selection actually changes, so the plot history survives.
  if (prev && items.some((t) => t.name === prev)) { sel.value = prev; return; }
  if (items.length) { sel.value = items[0].name; onInspectorTopicChange(); }
}

function refreshInspectorTopics() { fetchLiveTopics(() => populateInspectorTopics()); }

function initInspector() {
  inspPlot = new TimePlot($('#inspPlot'));
  $('#rawThrottleLabel').textContent = String(RAW_THROTTLE_HZ);
  // Single animation-rate redraw loop: message arrival only appends data,
  // so a 100 Hz topic still costs exactly one repaint per frame.
  const tick = () => {
    if ($('#view-inspector').classList.contains('active')) {
      if (inspTopic) {
        const hz = inspRateStamps.length > 1
          ? (inspRateStamps.length - 1) / ((inspRateStamps[inspRateStamps.length - 1] - inspRateStamps[0]) / 1000)
          : 0;
        const stale = inspRateStamps.length && performance.now() - inspRateStamps[inspRateStamps.length - 1] > 3000;
        $('#inspRate').textContent = (!inspRateStamps.length || stale) ? '0.0 Hz' : hz.toFixed(1) + ' Hz';
        $('#inspCount').textContent = String(inspMsgCount);
      }
      renderPlot();
    }
    // Same rationale for Time Histories: odometry callbacks only append
    // samples, and every vessel's plots repaint once per frame here.
    if ($('#view-history').classList.contains('active')) renderHistory();
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}

// ---------------------------------------------------------------------
// Page wiring
// ---------------------------------------------------------------------
function setView(name) {
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.dataset.view === name));
  document.querySelectorAll('.view').forEach((v) => v.classList.toggle('active', v.id === 'view-' + name));
  if (name === 'pointcloud' && pcViewer) pcViewer.resize();
  if (name === 'inspector' && inspPlot) { inspPlot._resize(); renderPlot(); }
  if (name === 'history') {
    // Canvases sized while the tab was display:none have a zero-width
    // bounding box; re-measure now that they are actually laid out.
    for (const entry of histVessels.values()) {
      for (const { plot } of entry.plots) plot._resize();
    }
    renderHistory();
  }
}

function populateSelect(sel, items, labelFn) {
  sel.innerHTML = items.length
    ? items.map((s, i) => `<option value="${i}">${labelFn(s)}</option>`).join('')
    : '<option value="">None available</option>';
}

// Refresh every view from one freshly-fetched topic list, isolating each so
// that a failure in one cannot silently abort the others. That is not
// defensive padding: the camera/lidar/overlay views construct a Three.js
// viewer, and when three.min.js is unavailable the resulting ReferenceError
// used to propagate out of onVesselChange() and skip everything after it -
// which is exactly how the Time Histories tab ended up never being built.
function refreshAllViews() {
  const steps = [
    ['vessel views', () => { if (currentVessel !== null) onVesselChange(); }],
    ['topic inspector', populateInspectorTopics],
    ['time histories', refreshHistoryIfVesselsChanged],
  ];
  for (const [name, fn] of steps) {
    try {
      fn();
    } catch (e) {
      console.error(`[visualizer] refresh step "${name}" failed:`, e);
    }
  }
}

function onVesselChange() {
  currentVessel = $('#vesselSelect').value;
  // Note: the Time Histories tab is NOT rebuilt here. It covers every vessel,
  // so the dropdown does not scope it, and rebuilding would throw away the
  // accumulated history of all vessels every time the selection changed.
  // refreshHistoryIfVesselsChanged() owns its lifecycle instead.
  const cams = cameraTopicsForVessel(), lidars = lidarTopicsForVessel();
  populateSelect($('#cameraSelect'), cams, (s) => `Camera ${s.sensor_id}`);
  populateSelect($('#lidarSelect'), lidars, (s) => `Lidar ${s.sensor_id}`);
  populateSelect($('#ovCameraSelect'), cams, (s) => `Camera ${s.sensor_id}`);
  populateSelect($('#ovLidarSelect'), lidars, (s) => `Lidar ${s.sensor_id}`);
  onCameraChange(); onLidarChange(); onOverlayChange();
}

async function loadSensorConfig() {
  try {
    const r = await fetch('/api/sensor_config');
    const cfg = await r.json();
    sensorConfig = cfg || {};
    const vessels = Object.keys(sensorConfig);
    const sel = $('#vesselSelect');
    if (!vessels.length) { sel.innerHTML = '<option value="">No vessel yet</option>'; return; }
    // This runs on a 5s timer, so the option list must only be rebuilt when
    // the vessel set actually changed: assigning innerHTML discards the
    // <select>'s selection state, snapping it back to the first option. The
    // periodic topic refresh then reads that reset value back through
    // onVesselChange(), so an unconditional rebuild silently threw away the
    // user's choice (and their inspector/chart subscriptions with it) every
    // five seconds.
    const existing = [...sel.options].map((o) => o.value);
    if (existing.length !== vessels.length || existing.some((v, i) => v !== vessels[i])) {
      sel.innerHTML = vessels.map((v) => `<option value="${v}">${v}</option>`).join('');
      // Restore the prior pick if it survived the change; the <select> is
      // otherwise sitting on its first option after the rebuild.
      if (currentVessel && vessels.includes(currentVessel)) sel.value = currentVessel;
    }
    if (!currentVessel || !vessels.includes(currentVessel)) { currentVessel = vessels[0]; sel.value = currentVessel; onVesselChange(); }
  } catch (e) { console.error('Failed to load sensor config', e); }
}

initRos();
initInspector();
loadSensorConfig();
setInterval(loadSensorConfig, 5000);
// Refresh discovered topics periodically too (new vessels/sensors can finish
// their handshake and start publishing after the page has already loaded).
// The inspector's topic dropdown is refreshed from the same sweep, but it
// keeps its own subscription (not in activeSubs) so that the vessel-change
// clearSubs() above can't silently drop the topic being inspected.
setInterval(() => fetchLiveTopics(refreshAllViews), 5000);
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="mavsim Local ROS2 Topic Visualizer")
    parser.add_argument("--port", type=int, default=8899, help="Port to listen on")
    parser.add_argument("--rosbridge-port", type=int, default=9090,
                        help="Port the rosbridge websocket is listening on (must be reachable "
                             "from the browser at the same hostname this page is loaded from)")
    args = parser.parse_args()

    global _rosbridge_port
    _rosbridge_port = args.rosbridge_port

    logger.info("Starting ROS2 visualizer on port %d (rosbridge port %d)", args.port, args.rosbridge_port)
    app.run(host="0.0.0.0", port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
