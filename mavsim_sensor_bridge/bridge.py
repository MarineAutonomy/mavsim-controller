"""
Main SensorBridge Class

Main entry point that coordinates all sensor servers. Provides a unified interface
for starting/stopping servers, registering callbacks, and managing configuration.
"""

import asyncio
import logging
from typing import Callable, Dict, Optional

from mavsim_sensor_bridge.config import BridgeConfig
from mavsim_sensor_bridge.servers.camera import CameraSensorServer


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
        self._servers: Dict[str, CameraSensorServer] = {}
        self._is_running = False
        self._server_tasks: Dict[str, asyncio.Task] = {}
        
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
