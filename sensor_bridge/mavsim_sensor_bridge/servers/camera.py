"""
Camera Sensor Server

WebSocket server for receiving camera frames from browser-based sensors.
Implements binary message parsing, callback registration, and frame drop detection.
"""

import asyncio
import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Dict, Optional, Tuple

# Optional: callback invoked for every frame (e.g. for local ROS2 publish)
FrameCallback = Callable[[int, int, float, bytes], None]

from mavsim_sensor_bridge.servers.base import BaseSensorServer
from mavsim_sensor_bridge.utils.binary import BinaryMessageError, unpack_camera_frame


class CameraSensorServer(BaseSensorServer):
    """
    WebSocket server for camera frame streaming.
    
    Receives binary camera frames from browser-based sensors via WebSocket,
    unpacks them using the binary protocol, and invokes registered callbacks
    in a thread pool to avoid blocking the async event loop.
    
    Features:
    - Binary message unpacking (vessel_id, camera_id, timestamp, JPEG data)
    - Callback registration per (vessel_id, camera_id) pair
    - Thread pool for non-blocking callback execution
    - Frame drop detection via sequence number tracking
    
    Attributes:
        port (int): Port number for the WebSocket server
        name (str): Name identifier ("CameraSensorServer")
        callbacks (Dict[Tuple[int, int], Callable]): Registered callbacks keyed by (vessel_id, camera_id)
        executor (ThreadPoolExecutor): Thread pool for callback execution
        sequence_numbers (Dict[Tuple[int, int], int]): Last sequence number per (vessel_id, camera_id)
        dropped_frames (Dict[Tuple[int, int], int]): Count of dropped frames per (vessel_id, camera_id)
    """
    
    def __init__(
        self,
        port: int = 8765,
        max_workers: int = 4,
        log_level: int = logging.INFO
    ):
        """
        Initialize the camera sensor server.
        
        Args:
            port: Port number for the WebSocket server (default: 8765)
            max_workers: Maximum number of worker threads for callbacks (default: 4)
            log_level: Logging level (default: logging.INFO)
        """
        super().__init__(port=port, name="CameraSensorServer", log_level=log_level)
        
        # Callback registry: (vessel_id, camera_id) -> callback function
        self.callbacks: Dict[Tuple[int, int], Callable] = {}
        self._callbacks_lock = threading.Lock()
        
        # Thread pool for callback execution
        self.executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="camera-callback")
        
        # Frame sequence tracking for drop detection
        # Note: The binary protocol doesn't include sequence numbers, so we'll track
        # by timestamp gaps instead (frames arriving out of order or with large time gaps)
        self.last_timestamps: Dict[Tuple[int, int], float] = {}
        self.dropped_frames: Dict[Tuple[int, int], int] = {}
        self._sequence_lock = threading.Lock()
        
        # Configuration
        self.max_timestamp_gap_seconds = 0.5  # Consider frames dropped if timestamp gap > 0.5s
        
        # Optional global callback invoked for every frame (Task 2.5: e.g. local ROS2 publish)
        self._global_frame_callback: Optional[FrameCallback] = None
        self._global_callback_lock = threading.Lock()
        
        # Frame counters for debugging: check if any data is arriving on the WebSocket
        self._frame_counts: Dict[Tuple[int, int], int] = {}
        self._total_frames = 0
        self._first_frame_logged: set = set()  # (vessel_id, camera_id) we've logged "first frame" for
        self._stats_lock = threading.Lock()
        # Log every N-th frame at DEBUG for ongoing activity (set to 0 to disable)
        self._log_every_n_frames = 100
        
        self.logger.info(f"CameraSensorServer initialized on port {port} (max_workers={max_workers})")
    
    def on_frame(
        self,
        vessel_id: int,
        camera_id: int,
        callback: Callable[[int, int, float, bytes], None]
    ) -> None:
        """
        Register a callback for camera frames from a specific vessel and camera.
        
        The callback will be invoked in a thread pool (non-blocking) whenever
        a frame is received for the specified vessel_id and camera_id.
        
        Args:
            vessel_id: Vessel identifier (0-255)
            camera_id: Camera identifier (0-255)
            callback: Callback function with signature: callback(vessel_id, camera_id, timestamp, jpeg_data)
                     This will be called from a worker thread, so it should be thread-safe.
        
        Example:
            def handle_frame(vessel_id, camera_id, timestamp, jpeg_data):
                print(f"Frame from vessel {vessel_id}, camera {camera_id}")
                # Process JPEG data...
            
            server.on_frame(vessel_id=1, camera_id=1, callback=handle_frame)
        """
        if not (0 <= vessel_id <= 255):
            raise ValueError(f"vessel_id must be in range [0, 255], got {vessel_id}")
        if not (0 <= camera_id <= 255):
            raise ValueError(f"camera_id must be in range [0, 255], got {camera_id}")
        if not callable(callback):
            raise TypeError("callback must be callable")
        
        key = (vessel_id, camera_id)
        
        with self._callbacks_lock:
            self.callbacks[key] = callback
        
        # Initialize sequence tracking for this (vessel_id, camera_id)
        with self._sequence_lock:
            if key not in self.last_timestamps:
                self.last_timestamps[key] = 0.0
                self.dropped_frames[key] = 0
        
        self.logger.info(f"Registered callback for vessel_id={vessel_id}, camera_id={camera_id}")
    
    def set_global_frame_callback(self, callback: Optional[FrameCallback]) -> None:
        """
        Set an optional callback invoked for every received frame (any vessel/camera).

        Used by the bridge to publish to local ROS2 only for the controlled vessel.
        Callback signature: callback(vessel_id, camera_id, timestamp, jpeg_data).
        """
        with self._global_callback_lock:
            self._global_frame_callback = callback
        if callback is not None:
            self.logger.info("Global frame callback set (e.g. for local ROS2 publish)")
        else:
            self.logger.info("Global frame callback cleared")

    def remove_callback(self, vessel_id: int, camera_id: int) -> None:
        """
        Remove callback for a specific vessel and camera.
        
        Args:
            vessel_id: Vessel identifier
            camera_id: Camera identifier
        """
        key = (vessel_id, camera_id)
        
        with self._callbacks_lock:
            if key in self.callbacks:
                del self.callbacks[key]
                self.logger.info(f"Removed callback for vessel_id={vessel_id}, camera_id={camera_id}")
        
        with self._sequence_lock:
            if key in self.last_timestamps:
                del self.last_timestamps[key]
            if key in self.dropped_frames:
                del self.dropped_frames[key]
    
    async def _process_message(self, message) -> None:
        """
        Process an incoming binary camera frame message.
        
        This method is called by BaseSensorServer for each incoming message.
        It unpacks the binary data, checks for frame drops, and invokes
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
            vessel_id, camera_id, timestamp, jpeg_data = unpack_camera_frame(message)
            
            key = (vessel_id, camera_id)
            # Update frame counters for debugging (check if data is arriving on WebSocket)
            with self._stats_lock:
                self._frame_counts[key] = self._frame_counts.get(key, 0) + 1
                self._total_frames += 1
                count = self._frame_counts[key]
            # Log first frame per stream at INFO so you can see "is any data coming in?"
            if key not in self._first_frame_logged:
                self._first_frame_logged.add(key)
                self.logger.info(
                    "Received camera frame: vessel_id=%s, camera_id=%s, size=%s bytes (first for this stream)",
                    vessel_id, camera_id, len(jpeg_data),
                )
            elif self._log_every_n_frames and count % self._log_every_n_frames == 0:
                self.logger.debug(
                    "Camera frames: vessel_id=%s, camera_id=%s, total=%s",
                    vessel_id, camera_id, count,
                )
            
            # Check for frame drops (large timestamp gaps)
            with self._sequence_lock:
                last_timestamp = self.last_timestamps.get(key, 0.0)
                if last_timestamp > 0.0:
                    time_gap = timestamp - last_timestamp
                    if time_gap > self.max_timestamp_gap_seconds:
                        # Potential frame drop detected
                        self.dropped_frames[key] = self.dropped_frames.get(key, 0) + 1
                        self.logger.debug(
                            f"Frame drop detected for vessel_id={vessel_id}, camera_id={camera_id}: "
                            f"timestamp gap {time_gap:.3f}s (total drops: {self.dropped_frames[key]})"
                        )
                self.last_timestamps[key] = timestamp
            
            # Find callback for this (vessel_id, camera_id)
            with self._callbacks_lock:
                callback = self.callbacks.get(key)
            
            if callback:
                # Execute callback in thread pool (non-blocking)
                # Use run_in_executor to avoid blocking the event loop
                loop = asyncio.get_event_loop()
                loop.run_in_executor(
                    self.executor,
                    self._invoke_callback,
                    callback,
                    vessel_id,
                    camera_id,
                    timestamp,
                    jpeg_data
                )
            else:
                self.logger.debug(
                    f"No callback registered for vessel_id={vessel_id}, camera_id={camera_id}"
                )
            
            # Optional global callback (e.g. local ROS2 publish for controlled vessel only)
            with self._global_callback_lock:
                global_cb = self._global_frame_callback
            if global_cb:
                loop = asyncio.get_event_loop()
                loop.run_in_executor(
                    self.executor,
                    self._invoke_callback,
                    global_cb,
                    vessel_id,
                    camera_id,
                    timestamp,
                    jpeg_data
                )
        
        except BinaryMessageError as e:
            self.logger.warning(f"Failed to unpack camera frame: {e}")
        except Exception as e:
            self.logger.error(f"Error processing camera frame: {e}", exc_info=True)
    
    @staticmethod
    def _invoke_callback(
        callback: Callable,
        vessel_id: int,
        camera_id: int,
        timestamp: float,
        jpeg_data: bytes
    ) -> None:
        """
        Invoke callback function with frame data.
        
        This is a static method that runs in the thread pool.
        It catches exceptions to prevent worker thread crashes.
        
        Args:
            callback: Callback function to invoke
            vessel_id: Vessel identifier
            camera_id: Camera identifier
            timestamp: Frame timestamp
            jpeg_data: JPEG image data
        """
        try:
            callback(vessel_id, camera_id, timestamp, jpeg_data)
        except Exception as e:
            # Log error but don't crash the worker thread
            logging.getLogger(__name__).error(
                f"Callback error for vessel_id={vessel_id}, camera_id={camera_id}: {e}",
                exc_info=True
            )
    
    def get_stats(self) -> dict:
        """
        Get current statistics including base (messages, bytes, connections) and
        camera frame counts so you can check if any data is arriving on the WebSocket.
        
        Returns:
            Dict with 'messages', 'bytes', 'connections', 'total_frames',
            and 'frames_per_stream' (e.g. {"(1, 1)": 150, "(1, 2)": 0}).
        """
        base = super().get_stats()
        with self._stats_lock:
            frames_per_stream = {str(k): v for k, v in self._frame_counts.items()}
            total = self._total_frames
        base["total_frames"] = total
        base["frames_per_stream"] = frames_per_stream
        return base

    def get_dropped_frames(self, vessel_id: int, camera_id: int) -> int:
        """
        Get count of dropped frames for a specific vessel and camera.
        
        Args:
            vessel_id: Vessel identifier
            camera_id: Camera identifier
        
        Returns:
            Number of dropped frames detected
        """
        key = (vessel_id, camera_id)
        with self._sequence_lock:
            return self.dropped_frames.get(key, 0)
    
    def reset_dropped_frames(self, vessel_id: Optional[int] = None, camera_id: Optional[int] = None) -> None:
        """
        Reset dropped frame counters.
        
        Args:
            vessel_id: Vessel identifier (None = reset all vessels)
            camera_id: Camera identifier (None = reset all cameras for vessel)
        """
        with self._sequence_lock:
            if vessel_id is None:
                # Reset all
                self.dropped_frames.clear()
                self.logger.info("Reset dropped frame counters for all vessels/cameras")
            elif camera_id is None:
                # Reset all cameras for this vessel
                keys_to_remove = [k for k in self.dropped_frames.keys() if k[0] == vessel_id]
                for key in keys_to_remove:
                    del self.dropped_frames[key]
                self.logger.info(f"Reset dropped frame counters for vessel_id={vessel_id}")
            else:
                # Reset specific (vessel_id, camera_id)
                key = (vessel_id, camera_id)
                if key in self.dropped_frames:
                    self.dropped_frames[key] = 0
                    self.logger.info(f"Reset dropped frame counter for vessel_id={vessel_id}, camera_id={camera_id}")
    
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
