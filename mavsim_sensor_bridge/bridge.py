"""
Main SensorBridge Class

Main entry point that coordinates all sensor servers. Provides a unified interface
for starting/stopping servers, registering callbacks, and managing configuration.
"""

import asyncio
import logging
from typing import Callable, Dict, List, Optional

import numpy as np

from mavsim_sensor_bridge.config import BridgeConfig
from mavsim_sensor_bridge.servers.base import BaseSensorServer
from mavsim_sensor_bridge.servers.camera import CameraSensorServer
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer


class SensorBridge:
    """
    Main entry point for the Local Sensor Bridge.
    
    Coordinates all sensor servers and provides a unified interface for:
    - Starting/stopping all servers concurrently
    - Registering callbacks for sensor data
    - Managing configuration
    
    Attributes:
        config: BridgeConfig instance with configuration
        _servers: Dictionary of server instances keyed by sensor type
        _is_running: Flag indicating if bridge is currently running
        _server_tasks: Dictionary of asyncio tasks for running servers
        logger: Logger instance for this bridge
    """
    
    def __init__(self, config: Optional[BridgeConfig] = None):
        """
        Initialize the SensorBridge.
        
        Args:
            config: Optional BridgeConfig instance. If None, uses default config.
        """
        self.config = config or BridgeConfig()
        self._servers: Dict[str, BaseSensorServer] = {}
        self._is_running = False
        self._server_tasks: Dict[str, asyncio.Task] = {}
        # Optional local ROS2 camera publisher (Task 2.5) - only this vessel's sensors
        self._ros2_camera_publisher = None
        
        # Set up logging
        self.logger = logging.getLogger(f"{__name__}.SensorBridge")
        self.logger.setLevel(self.config.log_level)
        
        # Create console handler if no handlers exist
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            handler.setLevel(self.config.log_level)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
        
        # Set up servers based on configuration
        self._setup_servers()
    
    def _setup_servers(self) -> None:
        """Set up sensor servers based on configuration."""
        if self.config.camera_enabled:
            self._servers['camera'] = CameraSensorServer(
                port=self.config.camera_port,
                log_level=self.config.log_level
            )
            self.logger.info(f"Camera server configured on port {self.config.camera_port}")
        if self.config.lidar_enabled:
            self._servers['lidar'] = LidarSensorServer(
                port=self.config.lidar_port,
                log_level=self.config.log_level
            )
            self.logger.info(f"Lidar server configured on port {self.config.lidar_port}")
    
    @property
    def is_running(self) -> bool:
        """Check if the bridge is currently running."""
        return self._is_running
    
    def on_camera(
        self,
        vessel_id: int,
        camera_id: int,
        callback: Callable[[int, int, float, bytes], None]
    ) -> None:
        """
        Register a callback for camera frames from a specific vessel and camera.
        
        The callback will be invoked whenever a camera frame is received for
        the specified vessel_id and camera_id.
        
        Args:
            vessel_id: Vessel identifier (0-255)
            camera_id: Camera identifier (0-255)
            callback: Callback function with signature: callback(vessel_id, camera_id, timestamp, jpeg_data)
        
        Raises:
            ValueError: If camera server is not enabled
        """
        if 'camera' not in self._servers:
            raise ValueError("Camera server is not enabled. Set camera_enabled=True in config.")
        
        self._servers['camera'].on_frame(vessel_id, camera_id, callback)
        self.logger.info(f"Registered camera callback for vessel_id={vessel_id}, camera_id={camera_id}")

    def on_lidar(
        self,
        vessel_id: int,
        callback: Callable[[np.ndarray, float], None]
    ) -> None:
        """
        Register a callback for lidar scans from a specific vessel.

        The callback will be invoked whenever a lidar scan is received for
        the specified vessel_id. Callback signature: callback(points, timestamp)
        where points is a numpy array of shape (N, 4) with dtype float32
        (x, y, z, intensity).

        Args:
            vessel_id: Vessel identifier (0-255)
            callback: Callback function with signature: callback(points, timestamp)

        Raises:
            ValueError: If lidar server is not enabled
        """
        if 'lidar' not in self._servers:
            raise ValueError("Lidar server is not enabled. Set lidar_enabled=True in config.")
        # Type narrow for lidar server's on_scan
        lidar_server = self._servers['lidar']
        assert isinstance(lidar_server, LidarSensorServer)
        lidar_server.on_scan(vessel_id, callback)
        self.logger.info(f"Registered lidar callback for vessel_id={vessel_id}")

    def enable_ros2(
        self,
        controlled_vessel_id: int,
        namespace: str,
        vessel_name: str,
        node_name: str = "mavsim_sensor_bridge",
        camera_ids: Optional[List[int]] = None,
    ) -> None:
        """
        Publish this vessel's camera frames to local ROS2 topics (Task 2.5).
        
        Only frames for the given controlled_vessel_id are published. Frames from
        other vessels are not published. Call before start().
        
        Args:
            controlled_vessel_id: Only publish frames for this vessel_id (int 0-255).
            namespace: ROS namespace (e.g. 'sim_abc123').
            vessel_name: Vessel name for topic (e.g. 'matsya_01' from config).
            node_name: Name for the rclpy node.
            camera_ids: Optional list of camera IDs from config; if provided, these
                topics are pre-created at start. If None, publishers created on demand.
        """
        if 'camera' not in self._servers:
            self.logger.warning("Cannot enable ROS2: camera server not enabled")
            return
        try:
            from mavsim_sensor_bridge.ros2_publisher import LocalROS2CameraPublisher
            self._ros2_camera_publisher = LocalROS2CameraPublisher(
                controlled_vessel_id=controlled_vessel_id,
                namespace=namespace,
                vessel_name=vessel_name,
                node_name=node_name,
                camera_ids=camera_ids,
            )
            self._servers['camera'].set_global_frame_callback(
                self._ros2_camera_publisher.publish_frame
            )
            self.logger.info(
                "ROS2 local publish enabled for vessel_id=%s (namespace=%s, vessel=%s)",
                controlled_vessel_id, namespace or "(default)", vessel_name,
            )
        except Exception as e:
            self.logger.warning("ROS2 publish not available (rclpy missing?): %s", e)
            self._ros2_camera_publisher = None
    
    async def start(self) -> None:
        """
        Start all enabled sensor servers concurrently.
        
        This method starts all configured servers and runs them concurrently
        using asyncio tasks. The method will block until stop() is called.
        
        Raises:
            RuntimeError: If bridge is already running
            OSError: If any server port is already in use
        """
        if self._is_running:
            raise RuntimeError("Bridge is already running")
        
        if not self._servers:
            self.logger.warning("No servers enabled. Nothing to start.")
            return
        
        self.logger.info("Starting SensorBridge...")
        self._is_running = True
        
        # Start all servers concurrently
        self._server_tasks = {}
        for sensor_type, server in self._servers.items():
            self.logger.info(f"Starting {sensor_type} server...")
            task = asyncio.create_task(server.start())
            self._server_tasks[sensor_type] = task
        
        # Start local ROS2 camera publisher if enabled (Task 2.5)
        if self._ros2_camera_publisher is not None:
            try:
                self._ros2_camera_publisher.start()
            except Exception as e:
                self.logger.warning("Failed to start ROS2 camera publisher: %s", e)
                self._ros2_camera_publisher = None
                self._servers['camera'].set_global_frame_callback(None)
        
        # Give servers a moment to start and check for immediate startup errors
        await asyncio.sleep(0.1)
        startup_errors = []
        for sensor_type, task in self._server_tasks.items():
            if task.done():
                try:
                    await task
                except Exception as e:
                    startup_errors.append((sensor_type, e))
                    self.logger.error(f"Failed to start {sensor_type} server: {e}")
        
        # If any server failed to start, stop all and raise error
        if startup_errors:
            self._is_running = False
            # Cancel remaining tasks
            for task in self._server_tasks.values():
                if not task.done():
                    task.cancel()
            self._server_tasks.clear()
            error_msg = "; ".join([f"{st}: {str(e)}" for st, e in startup_errors])
            raise RuntimeError(f"Failed to start servers: {error_msg}")
        
        try:
            # Wait for all servers to complete (they run until stopped)
            await asyncio.gather(*self._server_tasks.values(), return_exceptions=True)
        except Exception as e:
            self.logger.error(f"Error in server tasks: {e}")
            raise
        finally:
            self._is_running = False
    
    async def stop(self) -> None:
        """
        Stop all sensor servers gracefully.
        
        This method stops all running servers and waits for them to shut down.
        """
        if not self._is_running:
            self.logger.warning("Bridge is not running")
            return
        
        self.logger.info("Stopping SensorBridge...")
        self._is_running = False
        
        # Stop local ROS2 camera publisher (Task 2.5)
        if self._ros2_camera_publisher is not None:
            try:
                self._ros2_camera_publisher.stop()
            except Exception as e:
                self.logger.debug("Error stopping ROS2 camera publisher: %s", e)
            self._ros2_camera_publisher = None
            if 'camera' in self._servers:
                self._servers['camera'].set_global_frame_callback(None)
        
        # Stop all servers
        stop_tasks = []
        for sensor_type, server in self._servers.items():
            if server.is_running:
                self.logger.info(f"Stopping {sensor_type} server...")
                stop_tasks.append(server.stop())
        
        # Wait for all servers to stop
        if stop_tasks:
            await asyncio.gather(*stop_tasks, return_exceptions=True)
        
        # Cancel any running server tasks
        for sensor_type, task in self._server_tasks.items():
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        
        self._server_tasks.clear()
        self.logger.info("SensorBridge stopped")
    
    def get_server_stats(self, sensor_type: Optional[str] = None) -> Dict:
        """
        Get statistics from sensor servers.
        
        Args:
            sensor_type: Optional sensor type to get stats for. If None, returns stats for all servers.
        
        Returns:
            Dictionary of statistics. If sensor_type is specified, returns stats for that server.
            Otherwise returns a dictionary keyed by sensor type.
        """
        if sensor_type:
            if sensor_type not in self._servers:
                raise ValueError(f"Unknown sensor type: {sensor_type}")
            return self._servers[sensor_type].get_stats()
        
        return {
            sensor_type: server.get_stats()
            for sensor_type, server in self._servers.items()
        }
