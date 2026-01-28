"""
Sensor server implementations for the MAVSIM Local Sensor Bridge.

This package contains WebSocket server implementations for different sensor types:
- BaseSensorServer: Abstract base class for all sensor servers
- CameraSensorServer: Camera frame streaming (to be implemented in Task 1.4)
- LidarSensorServer: Lidar scan streaming (to be implemented in later tasks)
- SonarSensorServer: Imaging sonar streaming (to be implemented in later tasks)
- DepthCameraSensorServer: Depth camera streaming (to be implemented in later tasks)
- AuxiliarySensorServer: Auxiliary sensor data streaming (to be implemented in later tasks)
"""

from mavsim_sensor_bridge.servers.base import BaseSensorServer

__all__ = ["BaseSensorServer"]
