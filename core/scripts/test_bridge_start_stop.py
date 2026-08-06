#!/usr/bin/env python3
"""
Test script for Task 2.7: Bridge start/stop functionality.
This script tests that the bridge can start and stop correctly.
"""
import asyncio
import sys
import time
import threading
sys.path.insert(0, '/app')
from python_controller import MavsimController

# Track received frames
received_frames = []
frame_lock = threading.Lock()

# Create controller
controller = MavsimController(
    backend_url='http://localhost:5000',
    session_id='test',
    api_token='test',
    rosbridge_url='ws://localhost:9090'
)

# Enable local sensors
controller.enable_local_sensors(camera_port=8765)

# Register callback
@controller.on_camera(vessel_id=1, camera_id=1)
def handle_frame(vessel_id, camera_id, timestamp, jpeg_data):
    with frame_lock:
        received_frames.append({
            'vessel_id': vessel_id,
            'camera_id': camera_id,
            'timestamp': timestamp,
            'size': len(jpeg_data)
        })

# Start bridge in background (simulate connect)
def run_bridge():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    controller._bridge_loop = loop
    try:
        loop.run_until_complete(controller._sensor_bridge.start())
    except Exception as e:
        print(f"Bridge error: {e}")

controller._bridge_thread = threading.Thread(target=run_bridge, daemon=True)
controller._bridge_thread.start()

# Wait for bridge to start
time.sleep(1.0)

# Check if bridge is running
if controller._bridge_thread.is_alive():
    print("OK: Bridge started")
else:
    print("FAIL: Bridge thread not running")
    sys.exit(1)

# Stop bridge
controller.close()

# Wait for thread to finish
if controller._bridge_thread.is_alive():
    controller._bridge_thread.join(timeout=2.0)

if not controller._bridge_thread.is_alive():
    print("OK: Bridge stopped")
else:
    print("FAIL: Bridge thread still running")
    sys.exit(1)

print("SUCCESS")

