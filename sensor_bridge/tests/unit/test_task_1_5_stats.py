"""
Unit tests for Task 1.5: Statistics and Monitoring

Tests verify that the StatsCollector:
- Accurately tracks message counts and bytes per sensor type
- Tracks per-vessel statistics correctly
- Calculates throughput rates (messages/sec, bytes/sec)
- Supports periodic logging with configurable intervals
- Provides stats reset functionality
"""

import asyncio
import pytest
import time
from mavsim_sensor_bridge.utils.stats import StatsCollector


def test_stats_collector_counts():
    """Test that stats collector accurately tracks message counts and bytes."""
    stats = StatsCollector(log_interval=0)  # Disable periodic logging for tests
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('lidar', vessel_id=2, bytes=100000)
    
    assert stats.get_count('camera') == 2
    assert stats.get_bytes('camera') == 100000
    assert stats.get_count('lidar') == 1
    assert stats.get_bytes('lidar') == 100000


def test_stats_per_vessel():
    """Test that stats collector tracks per-vessel statistics correctly."""
    stats = StatsCollector(log_interval=0)
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('camera', vessel_id=2, bytes=50000)
    
    assert stats.get_count('camera', vessel_id=1) == 1
    assert stats.get_count('camera', vessel_id=2) == 1
    assert stats.get_bytes('camera', vessel_id=1) == 50000
    assert stats.get_bytes('camera', vessel_id=2) == 50000
    
    # Total should be sum of both vessels
    assert stats.get_count('camera') == 2
    assert stats.get_bytes('camera') == 100000


def test_stats_multiple_sensor_types():
    """Test stats collector handles multiple sensor types correctly."""
    stats = StatsCollector(log_interval=0)
    
    # Record messages for different sensor types
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('lidar', vessel_id=1, bytes=100000)
    stats.record_message('sonar', vessel_id=2, bytes=200000)
    
    assert stats.get_count('camera') == 2
    assert stats.get_bytes('camera') == 100000
    assert stats.get_count('lidar') == 1
    assert stats.get_bytes('lidar') == 100000
    assert stats.get_count('sonar') == 1
    assert stats.get_bytes('sonar') == 200000


def test_stats_throughput():
    """Test that throughput calculation works correctly."""
    stats = StatsCollector(log_interval=0)
    
    # Record some messages
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('camera', vessel_id=1, bytes=50000)
    
    # Wait a bit to get meaningful throughput
    time.sleep(0.1)
    
    msg_per_sec, bytes_per_sec = stats.get_throughput('camera')
    
    # Should have some throughput (at least 2 messages in 0.1s = 20 msg/s)
    assert msg_per_sec > 0
    assert bytes_per_sec > 0
    
    # Check per-vessel throughput
    msg_per_sec_v1, bytes_per_sec_v1 = stats.get_throughput('camera', vessel_id=1)
    assert msg_per_sec_v1 > 0
    assert bytes_per_sec_v1 > 0


def test_stats_reset():
    """Test that reset clears all statistics."""
    stats = StatsCollector(log_interval=0)
    
    # Record some messages
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('lidar', vessel_id=2, bytes=100000)
    
    # Verify stats exist
    assert stats.get_count('camera') == 1
    assert stats.get_count('lidar') == 1
    
    # Reset
    stats.reset()
    
    # Verify stats are cleared
    assert stats.get_count('camera') == 0
    assert stats.get_count('lidar') == 0
    assert stats.get_bytes('camera') == 0
    assert stats.get_bytes('lidar') == 0


def test_stats_get_all_stats():
    """Test that get_all_stats returns complete statistics."""
    stats = StatsCollector(log_interval=0)
    
    # Record messages
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('camera', vessel_id=2, bytes=50000)
    stats.record_message('lidar', vessel_id=1, bytes=100000)
    
    all_stats = stats.get_all_stats()
    
    # Check structure
    assert 'sensors' in all_stats
    assert 'vessels' in all_stats
    assert 'elapsed_time' in all_stats
    
    # Check sensor stats
    assert 'camera' in all_stats['sensors']
    assert 'lidar' in all_stats['sensors']
    assert all_stats['sensors']['camera']['count'] == 2
    assert all_stats['sensors']['camera']['bytes'] == 100000
    assert 'messages_per_second' in all_stats['sensors']['camera']
    assert 'bytes_per_second' in all_stats['sensors']['camera']
    
    # Check vessel stats
    assert 'camera' in all_stats['vessels']
    assert 'lidar' in all_stats['vessels']
    assert 1 in all_stats['vessels']['camera']
    assert 2 in all_stats['vessels']['camera']
    assert all_stats['vessels']['camera'][1]['count'] == 1
    assert all_stats['vessels']['camera'][2]['count'] == 1


def test_stats_empty_sensor_type():
    """Test that querying non-existent sensor types returns 0."""
    stats = StatsCollector(log_interval=0)
    
    assert stats.get_count('nonexistent') == 0
    assert stats.get_bytes('nonexistent') == 0
    assert stats.get_count('nonexistent', vessel_id=1) == 0
    assert stats.get_bytes('nonexistent', vessel_id=1) == 0


def test_stats_thread_safety():
    """Test that stats collector is thread-safe."""
    import threading
    
    stats = StatsCollector(log_interval=0)
    results = []
    
    def record_messages():
        for i in range(100):
            stats.record_message('camera', vessel_id=1, bytes=1000)
    
    # Create multiple threads
    threads = []
    for _ in range(10):
        thread = threading.Thread(target=record_messages)
        threads.append(thread)
        thread.start()
    
    # Wait for all threads
    for thread in threads:
        thread.join()
    
    # Should have 1000 messages total (10 threads * 100 messages)
    assert stats.get_count('camera') == 1000
    assert stats.get_bytes('camera') == 1000000


@pytest.mark.asyncio
async def test_stats_periodic_logging():
    """Test that periodic logging works correctly."""
    stats = StatsCollector(log_interval=0.5)  # Log every 0.5 seconds
    
    # Start periodic logging
    stats.start_periodic_logging()
    
    # Verify task was created
    assert stats._log_task is not None
    
    # Record some messages
    stats.record_message('camera', vessel_id=1, bytes=50000)
    
    # Wait a short time (less than one log cycle to avoid hanging)
    await asyncio.sleep(0.1)
    
    # Stop periodic logging and wait for task to finish
    await stats.stop_periodic_logging_async()
    
    # Verify it stopped
    assert stats._log_task is None


def test_stats_periodic_logging_stop_before_start():
    """Test that stopping periodic logging before starting doesn't crash."""
    stats = StatsCollector(log_interval=1.0)
    
    # Should not crash
    stats.stop_periodic_logging()
    
    # Verify no task was created
    assert stats._log_task is None


def test_stats_periodic_logging_disabled():
    """Test that periodic logging doesn't start when interval is 0."""
    stats = StatsCollector(log_interval=0)
    
    stats.start_periodic_logging()
    
    # Should not have created a task
    assert stats._log_task is None


def test_stats_log_stats():
    """Test that log_stats method works without crashing."""
    stats = StatsCollector(log_interval=0)
    
    # Record some messages
    stats.record_message('camera', vessel_id=1, bytes=50000)
    stats.record_message('lidar', vessel_id=2, bytes=100000)
    
    # Should not raise exception
    stats.log_stats()


def test_stats_throughput_zero_time():
    """Test that throughput calculation handles zero elapsed time."""
    stats = StatsCollector(log_interval=0)
    
    # Immediately after creation, elapsed time might be very small
    # Throughput should still return valid values (0 or very small)
    msg_per_sec, bytes_per_sec = stats.get_throughput('camera')
    assert msg_per_sec == 0.0
    assert bytes_per_sec == 0.0


def test_stats_multiple_vessels_same_sensor():
    """Test tracking multiple vessels for the same sensor type."""
    stats = StatsCollector(log_interval=0)
    
    # Record messages for multiple vessels
    for vessel_id in [1, 2, 3]:
        for _ in range(10):
            stats.record_message('camera', vessel_id=vessel_id, bytes=50000)
    
    # Check totals
    assert stats.get_count('camera') == 30
    assert stats.get_bytes('camera') == 1500000
    
    # Check per-vessel
    for vessel_id in [1, 2, 3]:
        assert stats.get_count('camera', vessel_id=vessel_id) == 10
        assert stats.get_bytes('camera', vessel_id=vessel_id) == 500000
