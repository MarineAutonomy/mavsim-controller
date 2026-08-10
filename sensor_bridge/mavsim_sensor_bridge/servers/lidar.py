"""
Lidar Sensor Server

WebSocket server for receiving lidar point cloud scans from browser-based sensors.
Implements binary message parsing, callback registration, and point count validation.
"""

import asyncio
import logging
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Dict, Optional, Tuple

import numpy as np

from mavsim_sensor_bridge.servers.base import BaseSensorServer
from mavsim_sensor_bridge.utils.binary import BinaryMessageError, unpack_lidar_scan

# Callback signature: callback(vessel_id, lidar_id, points, timestamp)
ScanCallback = Callable[[int, int, np.ndarray, float], None]


class LidarSensorServer(BaseSensorServer):
    """
    WebSocket server for lidar scan streaming.
    
    Receives binary lidar scans from browser-based sensors via WebSocket,
    unpacks them using the binary protocol, and invokes registered callbacks
    in a thread pool to avoid blocking the async event loop.
    
    Features:
    - Binary message unpacking (vessel_id, lidar_id, timestamp, point cloud)
    - Callback registration per (vessel_id, lidar_id)
    - Thread pool for non-blocking callback execution
    - Point count validation
    - Support for 3D point format (x, y, z, intensity) as Float32
    
    Attributes:
        port (int): Port number for the WebSocket server
        name (str): Name identifier ("LidarSensorServer")
        callbacks (Dict[Tuple[int, int], Callable]): Registered callbacks keyed by (vessel_id, lidar_id)
        executor (ThreadPoolExecutor): Thread pool for callback execution
        max_points_per_scan (int): Maximum allowed points per scan (for validation)
    """

    # Lidar scans can be large (e.g. 100K points × 4 floats × 4 bytes ≈ 1.6 MB).
    # Remove the default WebSocket frame size limit so large scans are accepted.
    _ws_max_size = None
    
    def __init__(
        self,
        port: int = 8766,
        max_workers: int = 4,
        max_points_per_scan: int = 1000000,  # 1M points max
        log_level: int = logging.INFO
    ):
        """
        Initialize the lidar sensor server.
        
        Args:
            port: Port number for the WebSocket server (default: 8766)
            max_workers: Maximum number of worker threads for callbacks (default: 4)
            max_points_per_scan: Maximum allowed points per scan (default: 1000000)
            log_level: Logging level (default: logging.INFO)
        """
        super().__init__(port=port, name="LidarSensorServer", log_level=log_level)
        
        # Callback registry: (vessel_id, lidar_id) -> callback function
        self.callbacks: Dict[Tuple[int, int], Callable] = {}
        self._callbacks_lock = threading.Lock()
        
        # Thread pool for callback execution
        self.executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="lidar-callback")
        
        # Configuration
        self.max_points_per_scan = max_points_per_scan
        
        # Optional global callback invoked for every scan (e.g. local ROS2 publish)
        self._global_scan_callback: Optional[Callable] = None
        self._global_callback_lock = threading.Lock()
        
        # Scan counters for debugging: check if any data is arriving on the WebSocket
        self._scan_counts: Dict[Tuple[int, int], int] = {}
        self._total_scans = 0
        self._first_scan_logged: set = set()  # (vessel_id, lidar_id) tuples logged
        self._stats_lock = threading.Lock()
        # Log every N-th scan at DEBUG for ongoing activity (set to 0 to disable)
        self._log_every_n_scans = 100
        
        self.logger.info(f"LidarSensorServer initialized on port {port} (max_workers={max_workers}, max_points={max_points_per_scan})")
    
    def on_scan(
        self,
        vessel_id: int,
        callback: Callable[[int, int, np.ndarray, float], None],
        lidar_id: int = 0,
    ) -> None:
        """
        Register a callback for lidar scans from a specific vessel and lidar.
        
        The callback will be invoked in a thread pool (non-blocking) whenever
        a scan is received for the specified (vessel_id, lidar_id).
        
        Args:
            vessel_id: Vessel identifier (0-255)
            callback: Callback function with signature:
                callback(vessel_id, lidar_id, points, timestamp)
                where points is numpy array of shape (N, 4) with dtype float32,
                each row is (x, y, z, intensity).
                This will be called from a worker thread, so it should be thread-safe.
            lidar_id: Lidar sensor identifier (0-255), default 0
        
        Example:
            def handle_scan(vessel_id, lidar_id, points, timestamp):
                print(f"Vessel {vessel_id} lidar {lidar_id}: {len(points)} points")
            
            server.on_scan(vessel_id=1, callback=handle_scan, lidar_id=0)
        """
        if not (0 <= vessel_id <= 255):
            raise ValueError(f"vessel_id must be in range [0, 255], got {vessel_id}")
        if not (0 <= lidar_id <= 255):
            raise ValueError(f"lidar_id must be in range [0, 255], got {lidar_id}")
        if not callable(callback):
            raise TypeError("callback must be callable")
        
        with self._callbacks_lock:
            self.callbacks[(vessel_id, lidar_id)] = callback
        
        self.logger.info(f"Registered callback for vessel_id={vessel_id}, lidar_id={lidar_id}")
    
    def remove_callback(self, vessel_id: int, lidar_id: int = 0) -> None:
        """
        Remove callback for a specific vessel and lidar.
        
        Args:
            vessel_id: Vessel identifier
            lidar_id: Lidar sensor identifier
        """
        key = (vessel_id, lidar_id)
        with self._callbacks_lock:
            if key in self.callbacks:
                del self.callbacks[key]
                self.logger.info(f"Removed callback for vessel_id={vessel_id}, lidar_id={lidar_id}")
    
    def set_global_scan_callback(self, callback: Optional[Callable]) -> None:
        """
        Set an optional callback invoked for every received scan (any vessel).

        Used by the bridge to publish to local ROS2 only for the controlled vessel.
        Callback signature: callback(vessel_id, points, timestamp).

        Args:
            callback: Callable or None to clear.
        """
        with self._global_callback_lock:
            self._global_scan_callback = callback
        if callback is not None:
            self.logger.info("Global scan callback set (e.g. for local ROS2 publish)")
        else:
            self.logger.info("Global scan callback cleared")

    async def _process_message(self, message) -> None:
        """
        Process an incoming binary lidar scan message.
        
        This method is called by BaseSensorServer for each incoming message.
        It unpacks the binary data, validates point count, and invokes
        registered callbacks in a thread pool.
        
        Args:
            message: Binary message data (bytes)
        """
        # Ensure message is bytes
        if not isinstance(message, bytes):
            self.logger.warning(f"Received non-binary message (type: {type(message)})")
            return
        
        try:
            # Unpack binary message
            vessel_id, lidar_id, timestamp, points = unpack_lidar_scan(message)
            
            # Validate point count
            point_count = points.shape[0]
            if point_count == 0:
                self.logger.warning(f"Received empty point cloud for vessel_id={vessel_id}, lidar_id={lidar_id}")
                return
            
            if point_count > self.max_points_per_scan:
                self.logger.warning(
                    f"Point count {point_count} exceeds maximum {self.max_points_per_scan} "
                    f"for vessel_id={vessel_id}, lidar_id={lidar_id}, truncating scan"
                )
                points = points[:self.max_points_per_scan]
            
            # Validate point format: should be (N, 4) with float32
            if points.shape[1] != 4:
                self.logger.warning(
                    f"Invalid point format: expected shape (N, 4), got {points.shape} "
                    f"for vessel_id={vessel_id}, lidar_id={lidar_id}"
                )
                return
            
            if points.dtype != np.float32:
                self.logger.warning(
                    f"Invalid point dtype: expected float32, got {points.dtype} "
                    f"for vessel_id={vessel_id}, lidar_id={lidar_id}, converting"
                )
                points = points.astype(np.float32)
            
            # Update scan counters for debugging
            key = (vessel_id, lidar_id)
            with self._stats_lock:
                self._scan_counts[key] = self._scan_counts.get(key, 0) + 1
                self._total_scans += 1
                count = self._scan_counts[key]
            
            # Log first scan per (vessel, lidar) at INFO
            if key not in self._first_scan_logged:
                self._first_scan_logged.add(key)
                self.logger.info(
                    "Received lidar scan: vessel_id=%s, lidar_id=%s, points=%s, size=%s bytes (first for this sensor)",
                    vessel_id, lidar_id, point_count, len(message),
                )
            elif self._log_every_n_scans and count % self._log_every_n_scans == 0:
                self.logger.debug(
                    "Lidar scans: vessel_id=%s, lidar_id=%s, points=%s, total=%s",
                    vessel_id, lidar_id, point_count, count,
                )
            
            # Find callback for this (vessel_id, lidar_id)
            with self._callbacks_lock:
                callback = self.callbacks.get(key)
            
            if callback:
                loop = asyncio.get_event_loop()
                loop.run_in_executor(
                    self.executor,
                    self._invoke_callback,
                    callback,
                    vessel_id,
                    lidar_id,
                    points,
                    timestamp
                )
            else:
                self.logger.debug(
                    f"No callback registered for vessel_id={vessel_id}, lidar_id={lidar_id}"
                )
            
            # Optional global callback (e.g. local ROS2 publish for controlled vessel only)
            with self._global_callback_lock:
                global_cb = self._global_scan_callback
            if global_cb:
                loop = asyncio.get_event_loop()
                loop.run_in_executor(
                    self.executor,
                    self._invoke_global_callback,
                    global_cb,
                    vessel_id,
                    lidar_id,
                    points,
                    timestamp
                )
        
        except BinaryMessageError as e:
            self.logger.warning(f"Failed to unpack lidar scan: {e}")
        except Exception as e:
            self.logger.error(f"Error processing lidar scan: {e}", exc_info=True)
    
    @staticmethod
    def _invoke_callback(
        callback: Callable,
        vessel_id: int,
        lidar_id: int,
        points: np.ndarray,
        timestamp: float
    ) -> None:
        """
        Invoke callback function with scan data.
        
        This is a static method that runs in the thread pool.
        It catches exceptions to prevent worker thread crashes.
        
        Args:
            callback: Callback function to invoke
            vessel_id: Vessel identifier
            lidar_id: Lidar sensor identifier
            points: Point cloud array of shape (N, 4) with dtype float32
            timestamp: Scan timestamp
        """
        try:
            callback(vessel_id, lidar_id, points, timestamp)
        except Exception as e:
            logging.getLogger(__name__).error(
                f"Callback error for lidar scan: {e}",
                exc_info=True
            )
    
    @staticmethod
    def _invoke_global_callback(
        callback: Callable,
        vessel_id: int,
        lidar_id: int,
        points: np.ndarray,
        timestamp: float
    ) -> None:
        """
        Invoke global callback function with scan data (e.g. ROS2 publish).
        
        Passes vessel_id and lidar_id so the publisher can filter and route.
        
        Args:
            callback: Callback function to invoke
            vessel_id: Vessel identifier
            lidar_id: Lidar sensor identifier
            points: Point cloud array of shape (N, 4) with dtype float32
            timestamp: Scan timestamp
        """
        try:
            callback(vessel_id, lidar_id, points, timestamp)
        except Exception as e:
            logging.getLogger(__name__).error(
                f"Global callback error for lidar scan: {e}",
                exc_info=True
            )
    
    def get_stats(self) -> dict:
        """
        Get current statistics including base (messages, bytes, connections) and
        lidar scan counts so you can check if any data is arriving on the WebSocket.
        
        Returns:
            Dict with 'messages', 'bytes', 'connections', 'total_scans',
            and 'scans_per_vessel' (e.g. {"1": 150, "2": 0}).
        """
        base = super().get_stats()
        with self._stats_lock:
            scans_per_sensor = {
                f"{vid}:{lid}": v for (vid, lid), v in self._scan_counts.items()
            }
            total = self._total_scans
        base["total_scans"] = total
        base["scans_per_sensor"] = scans_per_sensor
        return base
    
    async def stop(self) -> None:
        """
        Stop the server and shutdown thread pool.
        
        This extends the base class stop() method to also shutdown
        the thread pool executor.
        """
        # Stop the server first
        await super().stop()
        
        # Shutdown thread pool
        self.logger.info("Shutting down thread pool executor...")
        self.executor.shutdown(wait=True)
        self.logger.info("Thread pool executor shut down")


