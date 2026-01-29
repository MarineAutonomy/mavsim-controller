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

# Main exports
from mavsim_sensor_bridge.bridge import SensorBridge, BridgeConfig

__all__ = ["SensorBridge", "BridgeConfig", "__version__"]
