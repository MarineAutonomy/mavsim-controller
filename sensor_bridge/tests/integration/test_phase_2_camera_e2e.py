"""
Phase 2 Integration Test: Camera End-to-End

Full end-to-end test of camera streaming in Docker environment.
Tests the complete flow: mock browser → bridge → controller callback

Test scenarios:
- Controller container starts with bridge
- Mock browser connects to bridge ports
- Camera frames flow: mock browser → bridge → controller callback
- Multiple cameras work simultaneously
- Bridge survives reconnection

Run with:
  docker-compose up -d sensor-bridge
  pytest sensor_bridge/tests/integration/test_phase_2_camera_e2e.py -v

Or use: bash sensor_bridge/scripts/test_task_2_8_docker.sh
"""

import asyncio
import os
import pytest
import threading
import time
import websockets
from mavsim_sensor_bridge.utils.binary import pack_camera_frame

# Bridge connection settings
BRIDGE_HOST = os.environ.get('BRIDGE_HOST', 'localhost')
BRIDGE_PORT = int(os.environ.get('BRIDGE_PORT', '8765'))
WS_URL = f"ws://{BRIDGE_HOST}:{BRIDGE_PORT}"

# Delay for bridge to be ready
BRIDGE_READY_DELAY = float(os.environ.get('BRIDGE_READY_DELAY', '2.0'))


# Shared state for tracking received frames
class FrameTracker:
    """Thread-safe frame tracker for callbacks."""
    def __init__(self):
        self.frames = []
        self.lock = threading.Lock()
    
    def add_frame(self, vessel_id, camera_id, timestamp, jpeg_data):
        """Add a received frame."""
        with self.lock:
            self.frames.append({
                'vessel_id': vessel_id,
                'camera_id': camera_id,
                'timestamp': timestamp,
                'jpeg_size': len(jpeg_data)
            })
    
    def get_frames(self, vessel_id=None, camera_id=None):
        """Get frames, optionally filtered by vessel_id and/or camera_id."""
        with self.lock:
            if vessel_id is None and camera_id is None:
                return self.frames.copy()
            return [
                f for f in self.frames
                if (vessel_id is None or f['vessel_id'] == vessel_id) and
                   (camera_id is None or f['camera_id'] == camera_id)
            ]
    
    def clear(self):
        """Clear all frames."""
        with self.lock:
            self.frames.clear()
    
    def count(self, vessel_id=None, camera_id=None):
        """Count frames, optionally filtered."""
        return len(self.get_frames(vessel_id, camera_id))


# Global frame tracker
frame_tracker = FrameTracker()


def create_camera_callback(vessel_id, camera_id):
    """Create a callback function for a specific vessel/camera."""
    def callback(v_id, c_id, timestamp, jpeg_data):
        frame_tracker.add_frame(v_id, c_id, timestamp, jpeg_data)
    return callback


@pytest.mark.asyncio
async def test_camera_frame_received():
    """Test that camera frames sent to bridge are received by controller callback."""
    # Wait for bridge to be ready (if using external bridge)
    await asyncio.sleep(BRIDGE_READY_DELAY)
    
    # Clear previous frames
    frame_tracker.clear()
    
    # Import and set up controller with bridge
    try:
        from mavsim_sensor_bridge import SensorBridge
        
        # Create bridge instance (simulating a controller starting its own bridge)
        # The controller starts its own bridge server to receive frames
        bridge = SensorBridge()
        bridge.on_camera(1, 1, create_camera_callback(1, 1))
        
        # Start bridge in background (simulates controller starting bridge)
        bridge_task = asyncio.create_task(bridge.start())
        await asyncio.sleep(0.5)  # Wait for bridge to start
        
        try:
            # Determine which bridge to connect to
            # If BRIDGE_HOST is set to a remote host, use that (docker-compose bridge)
            # Otherwise, use localhost (controller's own bridge)
            if BRIDGE_HOST != 'localhost' and BRIDGE_HOST != '127.0.0.1':
                # Testing against external bridge (docker-compose)
                test_url = WS_URL
            else:
                # Testing against controller's own bridge
                test_url = f"ws://localhost:{BRIDGE_PORT}"
            
            # Send frames from mock browser
            async with websockets.connect(test_url) as ws:
                for i in range(10):
                    frame = pack_camera_frame(
                        vessel_id=1,
                        camera_id=1,
                        timestamp=1706400000.0 + i * 0.033,
                        jpeg_data=b'\xff\xd8\xff' + os.urandom(50000)
                    )
                    await ws.send(frame)
                    await asyncio.sleep(0.033)  # 30 Hz
            
            # Wait for callbacks to execute
            await asyncio.sleep(0.5)
            
            # Verify frames were received
            frames = frame_tracker.get_frames(1, 1)
            assert len(frames) >= 8, f"Expected at least 8 frames, got {len(frames)}"
            
            # Verify frame data
            for frame in frames:
                assert frame['vessel_id'] == 1
                assert frame['camera_id'] == 1
                assert frame['jpeg_size'] > 0
        
        finally:
            # Stop bridge
            await bridge.stop()
            if not bridge_task.done():
                bridge_task.cancel()
                try:
                    await bridge_task
                except asyncio.CancelledError:
                    pass
    
    except ImportError:
        pytest.skip("SensorBridge not available - skipping integration test")


@pytest.mark.asyncio
async def test_multiple_cameras_concurrent():
    """Test 4 cameras streaming simultaneously."""
    await asyncio.sleep(BRIDGE_READY_DELAY)
    
    frame_tracker.clear()
    
    try:
        from mavsim_sensor_bridge import SensorBridge
        
        bridge = SensorBridge()
        
        # Register callbacks for multiple cameras
        bridge.on_camera(1, 1, create_camera_callback(1, 1))
        bridge.on_camera(1, 2, create_camera_callback(1, 2))
        bridge.on_camera(2, 1, create_camera_callback(2, 1))
        bridge.on_camera(2, 2, create_camera_callback(2, 2))
        
        bridge_task = asyncio.create_task(bridge.start())
        await asyncio.sleep(0.5)
        
        try:
            # Determine which bridge to connect to
            if BRIDGE_HOST != 'localhost' and BRIDGE_HOST != '127.0.0.1':
                test_url = WS_URL
            else:
                test_url = f"ws://localhost:{BRIDGE_PORT}"
            
            async def stream_camera(v, c, frames=30):
                """Stream frames for a specific camera."""
                async with websockets.connect(test_url) as ws:
                    for i in range(frames):
                        frame = pack_camera_frame(
                            v, c, 1706400000.0 + i * 0.033, 
                            os.urandom(30000)
                        )
                        await ws.send(frame)
                        await asyncio.sleep(0.033)
            
            # Stream from 4 cameras concurrently
            await asyncio.gather(
                stream_camera(1, 1),
                stream_camera(1, 2),
                stream_camera(2, 1),
                stream_camera(2, 2),
            )
            
            # Wait for callbacks
            await asyncio.sleep(1.0)
            
            # Verify all cameras received frames
            assert frame_tracker.count(1, 1) >= 25, "Camera 1,1 should receive frames"
            assert frame_tracker.count(1, 2) >= 25, "Camera 1,2 should receive frames"
            assert frame_tracker.count(2, 1) >= 25, "Camera 2,1 should receive frames"
            assert frame_tracker.count(2, 2) >= 25, "Camera 2,2 should receive frames"
            
        finally:
            await bridge.stop()
            if not bridge_task.done():
                bridge_task.cancel()
                try:
                    await bridge_task
                except asyncio.CancelledError:
                    pass
    
    except ImportError:
        pytest.skip("SensorBridge not available - skipping integration test")


@pytest.mark.asyncio
async def test_reconnection():
    """Test that bridge handles client reconnection."""
    await asyncio.sleep(BRIDGE_READY_DELAY)
    
    frame_tracker.clear()
    
    try:
        from mavsim_sensor_bridge import SensorBridge
        
        bridge = SensorBridge()
        bridge.on_camera(1, 1, create_camera_callback(1, 1))
        
        bridge_task = asyncio.create_task(bridge.start())
        await asyncio.sleep(0.5)
        
        try:
            # Determine which bridge to connect to
            if BRIDGE_HOST != 'localhost' and BRIDGE_HOST != '127.0.0.1':
                test_url = WS_URL
            else:
                test_url = f"ws://localhost:{BRIDGE_PORT}"
            
            # Test multiple reconnections
            for reconnect_count in range(3):
                async with websockets.connect(test_url) as ws:
                    frame = pack_camera_frame(
                        1, 1, 1706400000.0 + reconnect_count, 
                        b'\xff' * 1000
                    )
                    await ws.send(frame)
                    await asyncio.sleep(0.1)
                
                # Disconnect and wait before reconnecting
                await asyncio.sleep(0.2)
            
            # Wait for callbacks
            await asyncio.sleep(0.5)
            
            # Verify frames were received across reconnections
            frames = frame_tracker.get_frames(1, 1)
            assert len(frames) >= 2, f"Expected at least 2 frames across reconnections, got {len(frames)}"
        
        finally:
            await bridge.stop()
            if not bridge_task.done():
                bridge_task.cancel()
                try:
                    await bridge_task
                except asyncio.CancelledError:
                    pass
    
    except ImportError:
        pytest.skip("SensorBridge not available - skipping integration test")


@pytest.mark.asyncio
async def test_controller_bridge_integration():
    """Test full controller integration: controller starts bridge, receives frames."""
    await asyncio.sleep(BRIDGE_READY_DELAY)
    
    frame_tracker.clear()
    
    # Try to import controller (may not be available in sensor_bridge container)
    try:
        import sys
        sys.path.insert(0, '/app')
        from python_controller import MavsimController
    except ImportError:
        pytest.skip("MavsimController not available - skipping controller integration test")
    
    try:
        from mavsim_sensor_bridge import SensorBridge
        
        # Simulate controller setup
        controller = MavsimController(
            backend_url='http://localhost:5000',
            session_id='test',
            api_token='test',
            rosbridge_url='ws://localhost:9090'
        )
        
        # Enable local sensors
        controller.enable_local_sensors(camera_port=BRIDGE_PORT)
        
        # Register callback using decorator pattern
        @controller.on_camera(vessel_id=1, camera_id=1)
        def handle_frame(vessel_id, camera_id, timestamp, jpeg_data):
            frame_tracker.add_frame(vessel_id, camera_id, timestamp, jpeg_data)
        
        # Start bridge in background thread (simulate controller.connect())
        def run_bridge():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            controller._bridge_loop = loop
            try:
                loop.run_until_complete(controller._sensor_bridge.start())
            except Exception as e:
                print(f"Bridge error: {e}")
            finally:
                loop.close()
        
        bridge_thread = threading.Thread(target=run_bridge, daemon=True)
        bridge_thread.start()
        
        # Wait for bridge to start
        await asyncio.sleep(1.0)
        
        try:
            # Determine which bridge to connect to
            if BRIDGE_HOST != 'localhost' and BRIDGE_HOST != '127.0.0.1':
                test_url = WS_URL
            else:
                test_url = f"ws://localhost:{BRIDGE_PORT}"
            
            # Send frames from mock browser
            async with websockets.connect(test_url) as ws:
                for i in range(5):
                    frame = pack_camera_frame(
                        vessel_id=1,
                        camera_id=1,
                        timestamp=1706400000.0 + i * 0.033,
                        jpeg_data=b'\xff\xd8\xff\xe0' + os.urandom(10000)
                    )
                    await ws.send(frame)
                    await asyncio.sleep(0.033)
            
            # Wait for callbacks
            await asyncio.sleep(0.5)
            
            # Verify frames were received
            frames = frame_tracker.get_frames(1, 1)
            assert len(frames) >= 4, f"Expected at least 4 frames, got {len(frames)}"
        
        finally:
            # Stop bridge (simulate controller.close())
            if controller._sensor_bridge and controller._bridge_loop:
                async def stop_bridge():
                    await controller._sensor_bridge.stop()
                
                if controller._bridge_loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(
                        stop_bridge(), controller._bridge_loop
                    )
                    future.result(timeout=2.0)
            
            if bridge_thread.is_alive():
                bridge_thread.join(timeout=2.0)
    
    except ImportError:
        pytest.skip("Required modules not available - skipping controller integration test")

