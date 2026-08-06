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

app = Flask(__name__)

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
  canvas.strip{width:100%;height:140px;display:block;background:var(--bg);border-radius:4px;}
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
  <div class="tab" data-view="camera" onclick="setView('camera')">Camera</div>
  <div class="tab" data-view="pointcloud" onclick="setView('pointcloud')">Point Cloud</div>
  <div class="tab" data-view="overlay" onclick="setView('overlay')">Overlay</div>
</div>
<main>
  <div class="view active" id="view-history">
    <div class="grid" id="historyGrid"><div class="empty-state">Waiting for sensor config&hellip;</div></div>
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
    if (connected) fetchLiveTopics(onVesselChange);
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

// ---------------------------------------------------------------------
// Time-history strip chart (hand-rolled Canvas 2D, no chart library).
// ---------------------------------------------------------------------
class StripChart {
  constructor(canvas, series, maxPoints) {
    this.canvas = canvas; this.ctx = canvas.getContext('2d');
    this.series = series; this.maxPoints = maxPoints || 240;
    this.data = series.map(() => []);
    this._resize();
    window.addEventListener('resize', () => this._resize());
  }
  _resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, Math.floor(rect.width * (window.devicePixelRatio || 1)));
    this.canvas.height = Math.max(1, Math.floor(rect.height * (window.devicePixelRatio || 1)));
  }
  push(values) {
    for (let i = 0; i < this.series.length; i++) {
      const arr = this.data[i];
      arr.push(values[i] == null ? 0 : values[i]);
      if (arr.length > this.maxPoints) arr.shift();
    }
    this.render();
  }
  render() {
    const { ctx, canvas } = this; const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    let lo = Infinity, hi = -Infinity;
    for (const arr of this.data) for (const v of arr) { if (v < lo) lo = v; if (v > hi) hi = v; }
    if (!isFinite(lo)) { lo = -1; hi = 1; }
    if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
    const pad = (hi - lo) * 0.12; lo -= pad; hi += pad;
    ctx.strokeStyle = '#2d3348'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
    this.series.forEach((s, idx) => {
      const arr = this.data[idx];
      if (arr.length < 2) return;
      ctx.strokeStyle = s.color; ctx.lineWidth = 1.5 * (window.devicePixelRatio || 1);
      ctx.beginPath();
      for (let i = 0; i < arr.length; i++) {
        const x = (i / (this.maxPoints - 1)) * w;
        const y = h - ((arr[i] - lo) / (hi - lo)) * h;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    });
  }
}

const PALETTE = ['#4f8ff7', '#f5a623', '#30a46c', '#e5484d', '#a970ff', '#26c6da', '#ff7043', '#c0ca33'];
const activeSubs = []; // {topic, cb} pairs to clean up on vessel change

function clearSubs() { for (const { topic, cb } of activeSubs) ros.unsubscribe(topic, cb); activeSubs.length = 0; }
function sub(topic, type, cb) { ros.subscribe(topic, type, cb); activeSubs.push({ topic, cb }); }

function addChartPanel(container, title, seriesNames, topic, type, extractor) {
  const panel = document.createElement('div'); panel.className = 'panel';
  const series = seriesNames.map((n, i) => ({ name: n, color: PALETTE[i % PALETTE.length] }));
  panel.innerHTML = `<h3>${title}</h3><canvas class="strip"></canvas>
    <div class="legend">${series.map((s) => `<span><span class="sw" style="background:${s.color}"></span>${s.name}</span>`).join('')}</div>`;
  container.appendChild(panel);
  const chart = new StripChart(panel.querySelector('canvas'), series);
  sub(topic, type, (msg) => chart.push(extractor(msg)));
}

function quatToEuler(x, y, z, w) {
  const sinr_cosp = 2 * (w * x + y * z), cosr_cosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
  const siny_cosp = 2 * (w * z + x * y), cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);
  return [roll, pitch, yaw].map((r) => r * 180 / Math.PI);
}

// Vessel body-state array layout (matches _KINEMATIC_LEN / attitude handling
// used elsewhere in this codebase, e.g. user_repo_new/bridge_controller.py):
// [u,v,w,p,q,r,x,y,z,<attitude 3 or 4 vals>,<actuators...>]
const STATE_LABELS = ['u', 'v', 'w', 'p', 'q', 'r', 'x', 'y', 'z'];

function buildHistoryPanels() {
  const grid = $('#historyGrid'); grid.innerHTML = '';
  if (!currentVessel) { grid.innerHTML = '<div class="empty-state">No vessel selected.</div>'; return; }
  const vesselTopic = `/${currentVessel}`;
  addChartPanel(grid, 'Vessel State (first 9 channels)', STATE_LABELS, `${vesselTopic}/vessel_state`,
    'std_msgs/Float64MultiArray', (msg) => STATE_LABELS.map((_, i) => msg.data[i]));
  addChartPanel(grid, 'Vessel State Derivative (first 9 channels)', STATE_LABELS, `${vesselTopic}/vessel_state_der`,
    'std_msgs/Float64MultiArray', (msg) => STATE_LABELS.map((_, i) => msg.data[i]));

  const sensors = (sensorConfig[currentVessel] && sensorConfig[currentVessel].sensors) || [];
  for (const s of sensors) {
    const stype = (s.sensor_type || '').toLowerCase();
    if (stype === 'imu') {
      addChartPanel(grid, `IMU ${s.sensor_id} – Orientation (roll/pitch/yaw °)`, ['roll', 'pitch', 'yaw'],
        s.sensor_topic, 'sensor_msgs/Imu', (msg) => quatToEuler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
      addChartPanel(grid, `IMU ${s.sensor_id} – Angular Velocity`, ['p', 'q', 'r'], s.sensor_topic, 'sensor_msgs/Imu',
        (msg) => [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]);
      addChartPanel(grid, `IMU ${s.sensor_id} – Linear Acceleration`, ['ax', 'ay', 'az'], s.sensor_topic, 'sensor_msgs/Imu',
        (msg) => [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]);
    } else if (stype === 'gps') {
      addChartPanel(grid, `GPS ${s.sensor_id} – Lat/Lon/Alt`, ['lat', 'lon', 'alt'], s.sensor_topic, 'sensor_msgs/NavSatFix',
        (msg) => [msg.latitude, msg.longitude, msg.altitude]);
    } else if (stype === 'encoder') {
      const panel = document.createElement('div'); panel.className = 'panel';
      panel.innerHTML = `<h3>Encoder ${s.sensor_id}</h3><canvas class="strip"></canvas><div class="legend"></div>`;
      grid.appendChild(panel);
      let chart = null;
      sub(s.sensor_topic, 'std_msgs/Float64MultiArray', (msg) => {
        if (!chart) {
          const series = msg.data.map((_, i) => ({ name: `a${i}`, color: PALETTE[i % PALETTE.length] }));
          chart = new StripChart(panel.querySelector('canvas'), series);
          panel.querySelector('.legend').innerHTML = series.map((s2) => `<span><span class="sw" style="background:${s2.color}"></span>${s2.name}</span>`).join('');
        }
        chart.push(msg.data);
      });
    }
  }
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
    // Points are in the lidar's own local frame (X=forward, Y=left, Z=up -
    // see LidarSensor.js) - this is a Z-up viewer (like rviz), not Three.js's
    // default Y-up, so the camera needs an explicit up-vector and the grid
    // needs to lie in the XY plane instead of Three's default XZ plane.
    this.camera.up.set(0, 0, 1);
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
    // Z-up spherical orbit: phi measured from +Z (the sensor's up axis),
    // theta swept in the XY plane.
    this.camera.position.set(
      this.target.x + this.radius * Math.sin(this.phi) * Math.cos(this.theta),
      this.target.y + this.radius * Math.sin(this.phi) * Math.sin(this.theta),
      this.target.z + this.radius * Math.cos(this.phi),
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
      this.theta -= dx * 0.01;
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
// If the overlay looks rotated/mirrored relative to the real camera view,
// this Euler-order assumption is the first thing to revisit.
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
    const [rw, rh] = camSensor.resolution || [img.naturalWidth || 640, img.naturalHeight || 480];
    canvas.width = rw; canvas.height = rh;
    canvas.style.width = img.clientWidth + 'px'; canvas.style.height = img.clientHeight + 'px';
    const ctx = canvas.getContext('2d'); ctx.clearRect(0, 0, rw, rh);
    if (!ovLatestCloud) return;
    const pts = projectLidarToCamera(camSensor, lidarSensor, ovLatestCloud, rw, rh);
    ctx.fillStyle = 'rgba(79,143,247,0.85)';
    for (const p of pts) { ctx.beginPath(); ctx.arc(p.x, p.y, 2, 0, Math.PI * 2); ctx.fill(); }
    $('#ovStat').textContent = pts.length + ' / ' + ovLatestCloud.count + ' points in frame';
  };
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
    lidarObj.localToWorld(v);
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
// Page wiring
// ---------------------------------------------------------------------
function setView(name) {
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.dataset.view === name));
  document.querySelectorAll('.view').forEach((v) => v.classList.toggle('active', v.id === 'view-' + name));
  if (name === 'pointcloud' && pcViewer) pcViewer.resize();
}

function populateSelect(sel, items, labelFn) {
  sel.innerHTML = items.length
    ? items.map((s, i) => `<option value="${i}">${labelFn(s)}</option>`).join('')
    : '<option value="">None available</option>';
}

function onVesselChange() {
  currentVessel = $('#vesselSelect').value;
  clearSubs();
  buildHistoryPanels();
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
    sel.innerHTML = vessels.map((v) => `<option value="${v}">${v}</option>`).join('');
    if (!currentVessel || !vessels.includes(currentVessel)) { currentVessel = vessels[0]; sel.value = currentVessel; onVesselChange(); }
  } catch (e) { console.error('Failed to load sensor config', e); }
}

initRos();
loadSensorConfig();
setInterval(loadSensorConfig, 5000);
// Refresh discovered topics periodically too (new vessels/sensors can finish
// their handshake and start publishing after the page has already loaded).
setInterval(() => fetchLiveTopics(() => { if (currentVessel) onVesselChange(); }), 5000);
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
