"""
MAVSIM Local Sensor Bridge

A Python package for high-bandwidth perception sensor streaming from browser
to external controllers via local WebSocket connections.

The bridge provides separate WebSocket servers for different sensor types:
- Camera (port 8765)
- Lidar (port 8766)
- Imaging Sonar (port 8767)
- Depth Camera (port 8768)
- Auxiliary sensors (port 8769)
"""

__version__ = "0.1.0"

# Main exports - will be implemented in later tasks
# For now, we create a placeholder class to satisfy the test requirements
class SensorBridge:
    """
    Main entry point for the Local Sensor Bridge.
    
    This class will be fully implemented in Task 1.6.
    For now, this is a placeholder to allow package imports.
    
    Usage (future):
        bridge = SensorBridge()
        
        # Register callbacks for sensor data
        bridge.on_camera(vessel_id=1, camera_id=1, callback=my_camera_handler)
        bridge.on_lidar(vessel_id=1, callback=my_lidar_handler)
        
        # Or enable ROS2 publishing
        bridge.enable_ros2(node_name='sensor_bridge')
        
        # Start the bridge (blocking)
        await bridge.start()
    
    Ports:
        8765: Camera streams
        8766: Lidar streams
        8767: Imaging sonar streams
        8768: Depth camera streams
        8769: Auxiliary sensors (JSON)
    """
    pass

__all__ = ["SensorBridge", "__version__"]
