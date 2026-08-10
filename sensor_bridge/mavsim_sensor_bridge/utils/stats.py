"""
Statistics and Monitoring Module

Provides real-time throughput monitoring and statistics reporting for the sensor bridge.
Tracks per-sensor-type and per-vessel statistics including message counts, bytes,
and throughput rates.
"""

import asyncio
import logging
import threading
import time
from collections import defaultdict
from typing import Dict, Optional, Tuple


class StatsCollector:
    """
    Real-time statistics collector for sensor bridge.
    
    Tracks:
    - Per-sensor-type statistics (camera, lidar, sonar, etc.)
    - Per-vessel statistics for each sensor type
    - Message counts and byte counts
    - Throughput rates (messages/sec, bytes/sec)
    
    Supports periodic logging of statistics at configurable intervals.
    
    Attributes:
        logger (logging.Logger): Logger instance for statistics output
        _lock (threading.Lock): Thread lock for thread-safe operations
        _sensor_stats (dict): Per-sensor-type statistics
        _vessel_stats (dict): Per-vessel statistics (nested: sensor_type -> vessel_id)
        _start_time (float): Timestamp when collector was created/reset
        _last_log_time (float): Timestamp of last periodic log
        _log_interval (float): Interval between periodic logs (seconds)
        _log_task (Optional[asyncio.Task]): Background task for periodic logging
        _log_loop (Optional[asyncio.AbstractEventLoop]): Event loop for logging task
    """
    
    def __init__(self, log_interval: float = 10.0, log_level: int = logging.INFO):
        """
        Initialize statistics collector.
        
        Args:
            log_interval: Interval between periodic stats logs (seconds, 0 = disabled)
            log_level: Logging level for statistics output
        """
        self.logger = logging.getLogger(f"{__name__}.StatsCollector")
        self.logger.setLevel(log_level)
        
        # Create console handler if no handlers exist
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            handler.setLevel(log_level)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
        
        self._lock = threading.Lock()
        self._sensor_stats: Dict[str, Dict[str, int]] = defaultdict(lambda: {'count': 0, 'bytes': 0})
        self._vessel_stats: Dict[str, Dict[int, Dict[str, int]]] = defaultdict(
            lambda: defaultdict(lambda: {'count': 0, 'bytes': 0})
        )
        self._start_time = time.time()
        self._last_log_time = self._start_time
        self._log_interval = log_interval
        self._log_task: Optional[asyncio.Task] = None
        self._log_loop: Optional[asyncio.AbstractEventLoop] = None
    
    def record_message(
        self,
        sensor_type: str,
        vessel_id: int,
        bytes: int
    ) -> None:
        """
        Record a message for statistics tracking.
        
        Args:
            sensor_type: Type of sensor (e.g., 'camera', 'lidar', 'sonar')
            vessel_id: Vessel ID (0-255)
            bytes: Number of bytes in the message
        """
        with self._lock:
            # Update per-sensor-type stats
            self._sensor_stats[sensor_type]['count'] += 1
            self._sensor_stats[sensor_type]['bytes'] += bytes
            
            # Update per-vessel stats
            self._vessel_stats[sensor_type][vessel_id]['count'] += 1
            self._vessel_stats[sensor_type][vessel_id]['bytes'] += bytes
    
    def get_count(self, sensor_type: str, vessel_id: Optional[int] = None) -> int:
        """
        Get message count for a sensor type (optionally for a specific vessel).
        
        Args:
            sensor_type: Type of sensor (e.g., 'camera', 'lidar')
            vessel_id: Optional vessel ID. If None, returns total for sensor type.
            
        Returns:
            Message count
        """
        with self._lock:
            if vessel_id is None:
                return self._sensor_stats[sensor_type]['count']
            else:
                return self._vessel_stats[sensor_type][vessel_id]['count']
    
    def get_bytes(self, sensor_type: str, vessel_id: Optional[int] = None) -> int:
        """
        Get byte count for a sensor type (optionally for a specific vessel).
        
        Args:
            sensor_type: Type of sensor (e.g., 'camera', 'lidar')
            vessel_id: Optional vessel ID. If None, returns total for sensor type.
            
        Returns:
            Byte count
        """
        with self._lock:
            if vessel_id is None:
                return self._sensor_stats[sensor_type]['bytes']
            else:
                return self._vessel_stats[sensor_type][vessel_id]['bytes']
    
    def get_throughput(
        self,
        sensor_type: str,
        vessel_id: Optional[int] = None
    ) -> Tuple[float, float]:
        """
        Get throughput rates (messages/sec, bytes/sec) for a sensor type.
        
        Args:
            sensor_type: Type of sensor (e.g., 'camera', 'lidar')
            vessel_id: Optional vessel ID. If None, returns total for sensor type.
            
        Returns:
            Tuple of (messages_per_second, bytes_per_second)
        """
        with self._lock:
            elapsed = time.time() - self._start_time
            if elapsed == 0:
                return 0.0, 0.0
            
            # Access stats directly to avoid deadlock (we already have the lock)
            if vessel_id is None:
                count = self._sensor_stats[sensor_type]['count']
                bytes_count = self._sensor_stats[sensor_type]['bytes']
            else:
                count = self._vessel_stats[sensor_type][vessel_id]['count']
                bytes_count = self._vessel_stats[sensor_type][vessel_id]['bytes']
            
            msg_per_sec = count / elapsed
            bytes_per_sec = bytes_count / elapsed
            
            return msg_per_sec, bytes_per_sec
    
    def get_all_stats(self) -> Dict:
        """
        Get all statistics in a dictionary format.
        
        Returns:
            Dictionary containing:
            - 'sensors': Per-sensor-type stats (count, bytes, throughput)
            - 'vessels': Per-vessel stats (nested by sensor_type and vessel_id)
            - 'elapsed_time': Total elapsed time since reset
        """
        with self._lock:
            elapsed = time.time() - self._start_time
            
            # Build sensor stats with throughput
            sensor_stats = {}
            for sensor_type, stats in self._sensor_stats.items():
                # Calculate throughput directly (we already have the lock)
                count = stats['count']
                bytes_count = stats['bytes']
                msg_per_sec = count / elapsed if elapsed > 0 else 0.0
                bytes_per_sec = bytes_count / elapsed if elapsed > 0 else 0.0
                sensor_stats[sensor_type] = {
                    'count': count,
                    'bytes': bytes_count,
                    'messages_per_second': msg_per_sec,
                    'bytes_per_second': bytes_per_sec
                }
            
            # Build vessel stats
            vessel_stats = {}
            for sensor_type, vessel_dict in self._vessel_stats.items():
                vessel_stats[sensor_type] = {}
                for vessel_id, stats in vessel_dict.items():
                    # Calculate throughput directly (we already have the lock)
                    count = stats['count']
                    bytes_count = stats['bytes']
                    msg_per_sec = count / elapsed if elapsed > 0 else 0.0
                    bytes_per_sec = bytes_count / elapsed if elapsed > 0 else 0.0
                    vessel_stats[sensor_type][vessel_id] = {
                        'count': count,
                        'bytes': bytes_count,
                        'messages_per_second': msg_per_sec,
                        'bytes_per_second': bytes_per_sec
                    }
            
            return {
                'sensors': sensor_stats,
                'vessels': vessel_stats,
                'elapsed_time': elapsed
            }
    
    def reset(self) -> None:
        """
        Reset all statistics counters and restart timing.
        """
        with self._lock:
            self._sensor_stats.clear()
            self._vessel_stats.clear()
            self._start_time = time.time()
            self._last_log_time = self._start_time
            self.logger.debug("Statistics reset")
    
    def log_stats(self) -> None:
        """
        Log current statistics to the logger.
        
        This method formats and logs all statistics in a human-readable format.
        """
        stats = self.get_all_stats()
        
        self.logger.info("=" * 60)
        self.logger.info(f"Statistics Report (elapsed: {stats['elapsed_time']:.1f}s)")
        self.logger.info("=" * 60)
        
        # Log per-sensor-type stats
        if stats['sensors']:
            self.logger.info("Per-Sensor-Type Statistics:")
            for sensor_type, sensor_data in sorted(stats['sensors'].items()):
                self.logger.info(
                    f"  {sensor_type:12s}: "
                    f"{sensor_data['count']:6d} msgs, "
                    f"{sensor_data['bytes']:12,d} bytes, "
                    f"{sensor_data['messages_per_second']:6.1f} msg/s, "
                    f"{sensor_data['bytes_per_second']/1024/1024:6.2f} MB/s"
                )
        else:
            self.logger.info("Per-Sensor-Type Statistics: (none)")
        
        # Log per-vessel stats
        if stats['vessels']:
            self.logger.info("Per-Vessel Statistics:")
            for sensor_type in sorted(stats['vessels'].keys()):
                vessel_dict = stats['vessels'][sensor_type]
                if vessel_dict:
                    self.logger.info(f"  {sensor_type}:")
                    for vessel_id in sorted(vessel_dict.keys()):
                        vessel_data = vessel_dict[vessel_id]
                        self.logger.info(
                            f"    vessel_{vessel_id:03d}: "
                            f"{vessel_data['count']:6d} msgs, "
                            f"{vessel_data['bytes']:12,d} bytes, "
                            f"{vessel_data['messages_per_second']:6.1f} msg/s, "
                            f"{vessel_data['bytes_per_second']/1024/1024:6.2f} MB/s"
                        )
        else:
            self.logger.info("Per-Vessel Statistics: (none)")
        
        self.logger.info("=" * 60)
    
    async def _periodic_log_loop(self) -> None:
        """
        Background task that periodically logs statistics.
        
        This runs in an asyncio task and logs stats at the configured interval.
        """
        while True:
            try:
                await asyncio.sleep(self._log_interval)
                self.log_stats()
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"Error in periodic log loop: {e}")
    
    def start_periodic_logging(self, loop: Optional[asyncio.AbstractEventLoop] = None) -> None:
        """
        Start periodic statistics logging.
        
        Creates a background task that logs statistics at the configured interval.
        
        Args:
            loop: Optional event loop. If None, tries to use the current event loop.
                  If no event loop is running, periodic logging will not start.
        """
        if self._log_interval <= 0:
            self.logger.debug("Periodic logging disabled (log_interval <= 0)")
            return
        
        if self._log_task is not None:
            self.logger.warning("Periodic logging already started")
            return
        
        if loop is None:
            try:
                # Try to get the running event loop (preferred)
                loop = asyncio.get_running_loop()
            except RuntimeError:
                # No event loop running - can't start periodic logging
                self.logger.warning(
                    "No event loop running - periodic logging requires an active event loop. "
                    "Start periodic logging from within an async context or pass a loop parameter."
                )
                return
        
        self._log_loop = loop
        self._log_task = loop.create_task(self._periodic_log_loop())
        self.logger.info(f"Started periodic statistics logging (interval: {self._log_interval}s)")
    
    def stop_periodic_logging(self) -> None:
        """
        Stop periodic statistics logging.
        
        Cancels the background logging task.
        Note: This is a synchronous method. If you need to wait for the task
        to finish, use stop_periodic_logging_async() instead.
        """
        if self._log_task is None:
            return
        
        task = self._log_task
        self._log_task = None
        self._log_loop = None
        
        # Cancel the task (it will clean up asynchronously)
        task.cancel()
        
        self.logger.info("Stopped periodic statistics logging")
    
    async def stop_periodic_logging_async(self) -> None:
        """
        Stop periodic statistics logging and wait for the task to finish.
        
        This is an async version that properly waits for the task cancellation.
        """
        if self._log_task is None:
            return
        
        task = self._log_task
        self._log_task = None
        self._log_loop = None
        
        # Cancel the task and wait for it to finish
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        
        self.logger.info("Stopped periodic statistics logging")
