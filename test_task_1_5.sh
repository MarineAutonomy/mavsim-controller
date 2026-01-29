#!/bin/bash
# Test script for Task 1.5: Statistics and Monitoring
# This script verifies that the StatsCollector class works correctly

set -e

echo "Testing Task 1.5: Statistics and Monitoring"
echo "============================================="

# Change to sensor_bridge directory
cd "$(dirname "$0")"

echo ""
echo "1. Testing StatsCollector import..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.stats import StatsCollector
print(f'   ✓ StatsCollector imported successfully')
print(f'   ✓ StatsCollector class: {StatsCollector}')
"

echo ""
echo "2. Testing basic statistics tracking..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.stats import StatsCollector

stats = StatsCollector(log_interval=0)
stats.record_message('camera', vessel_id=1, bytes=50000)
stats.record_message('camera', vessel_id=1, bytes=50000)
stats.record_message('lidar', vessel_id=2, bytes=100000)

assert stats.get_count('camera') == 2, 'Camera count should be 2'
assert stats.get_bytes('camera') == 100000, 'Camera bytes should be 100000'
assert stats.get_count('lidar') == 1, 'Lidar count should be 1'
assert stats.get_bytes('lidar') == 100000, 'Lidar bytes should be 100000'

print('   ✓ Basic statistics tracking works')
"

echo ""
echo "3. Testing per-vessel statistics..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.stats import StatsCollector

stats = StatsCollector(log_interval=0)
stats.record_message('camera', vessel_id=1, bytes=50000)
stats.record_message('camera', vessel_id=2, bytes=50000)

assert stats.get_count('camera', vessel_id=1) == 1, 'Vessel 1 count should be 1'
assert stats.get_count('camera', vessel_id=2) == 1, 'Vessel 2 count should be 1'
assert stats.get_count('camera') == 2, 'Total count should be 2'

print('   ✓ Per-vessel statistics tracking works')
"

echo ""
echo "4. Testing statistics reset..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.stats import StatsCollector

stats = StatsCollector(log_interval=0)
stats.record_message('camera', vessel_id=1, bytes=50000)
assert stats.get_count('camera') == 1, 'Count should be 1 before reset'

stats.reset()
assert stats.get_count('camera') == 0, 'Count should be 0 after reset'
assert stats.get_bytes('camera') == 0, 'Bytes should be 0 after reset'

print('   ✓ Statistics reset works')
"

echo ""
echo "5. Testing throughput calculation..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.stats import StatsCollector

stats = StatsCollector(log_interval=0)
stats.record_message('camera', vessel_id=1, bytes=50000)
stats.record_message('camera', vessel_id=1, bytes=50000)

msg_per_sec, bytes_per_sec = stats.get_throughput('camera')
assert msg_per_sec >= 0, 'Throughput should be non-negative'
assert bytes_per_sec >= 0, 'Bytes per second should be non-negative'

print('   ✓ Throughput calculation works (msg/s: {:.2f}, bytes/s: {:.2f})'.format(msg_per_sec, bytes_per_sec))
"

echo ""
echo "6. Testing get_all_stats..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.stats import StatsCollector

stats = StatsCollector(log_interval=0)
stats.record_message('camera', vessel_id=1, bytes=50000)
stats.record_message('lidar', vessel_id=2, bytes=100000)

all_stats = stats.get_all_stats()
assert 'sensors' in all_stats, 'Should have sensors key'
assert 'vessels' in all_stats, 'Should have vessels key'
assert 'elapsed_time' in all_stats, 'Should have elapsed_time key'
assert 'camera' in all_stats['sensors'], 'Should have camera in sensors'
assert 'lidar' in all_stats['sensors'], 'Should have lidar in sensors'

print('   ✓ get_all_stats returns complete statistics')
"

echo ""
echo "7. Running pytest tests..."
if command -v pytest &> /dev/null; then
    if timeout 30 pytest tests/unit/test_task_1_5_stats.py -v --tb=short; then
        echo "   ✓ All pytest tests passed"
    else
        echo "   ✗ Some pytest tests failed or timed out"
        exit 1
    fi
else
    echo "   ⚠ pytest not found, skipping pytest tests"
    echo "   Install pytest with: pip install pytest pytest-asyncio"
    echo "   Note: Basic functionality tests above all passed ✓"
fi

echo ""
echo "============================================="
echo "All tests passed! ✓"
echo ""
echo "To run pytest tests manually:"
echo "  cd sensor_bridge && pytest tests/unit/test_task_1_5_stats.py -v"
