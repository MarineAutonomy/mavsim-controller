#!/bin/bash
# Test script for Task 1.3: Binary Message Utilities
# This script verifies that binary message packing/unpacking works correctly

set -e

echo "Testing Task 1.3: Binary Message Utilities"
echo "=========================================="

# Run from sensor_bridge root (script may be in scripts/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SENSOR_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SENSOR_BRIDGE_DIR"

echo ""
echo "1. Testing binary utilities import..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.binary import (
    pack_camera_frame,
    unpack_camera_frame,
    pack_lidar_scan,
    unpack_lidar_scan,
    pack_sonar_image,
    unpack_sonar_image,
    pack_depth_camera,
    unpack_depth_camera,
    BinaryMessageError,
)
print('   ✓ Binary utilities imported successfully')
"

echo ""
echo "2. Testing camera frame pack/unpack..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.binary import pack_camera_frame, unpack_camera_frame

# Test round-trip
vessel_id, camera_id, timestamp, jpeg_data = 1, 2, 1706400000.123, b'\xff\xd8\xff\xe0'
packed = pack_camera_frame(vessel_id, camera_id, timestamp, jpeg_data)
unpacked = unpack_camera_frame(packed)

assert unpacked[0] == vessel_id, f'vessel_id mismatch: {unpacked[0]} != {vessel_id}'
assert unpacked[1] == camera_id, f'camera_id mismatch: {unpacked[1]} != {camera_id}'
assert abs(unpacked[2] - timestamp) < 1e-10, f'timestamp mismatch: {unpacked[2]} != {timestamp}'
assert unpacked[3] == jpeg_data, 'jpeg_data mismatch'

print('   ✓ Camera frame pack/unpack works correctly')
"

echo ""
echo "3. Testing lidar scan pack/unpack..."
python3 -c "
import sys
import numpy as np
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.binary import pack_lidar_scan, unpack_lidar_scan

# Test round-trip
vessel_id, timestamp = 1, 1706400000.0
points = np.random.randn(1000, 4).astype(np.float32)
packed = pack_lidar_scan(vessel_id, timestamp, points)
unpacked_vessel_id, unpacked_timestamp, unpacked_points = unpack_lidar_scan(packed)

assert unpacked_vessel_id == vessel_id, f'vessel_id mismatch: {unpacked_vessel_id} != {vessel_id}'
assert abs(unpacked_timestamp - timestamp) < 1e-10, f'timestamp mismatch: {unpacked_timestamp} != {timestamp}'
assert unpacked_points.shape == points.shape, f'shape mismatch: {unpacked_points.shape} != {points.shape}'
assert np.allclose(points, unpacked_points), 'points data mismatch'

print('   ✓ Lidar scan pack/unpack works correctly')
"

echo ""
echo "4. Testing sonar image pack/unpack..."
python3 -c "
import sys
import numpy as np
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.binary import pack_sonar_image, unpack_sonar_image

# Test round-trip
vessel_id, timestamp, beams, range_bins = 1, 1706400000.123, 512, 1024
intensity_data = np.random.randint(0, 255, size=(beams, range_bins), dtype=np.uint8)
packed = pack_sonar_image(vessel_id, timestamp, beams, range_bins, intensity_data)
unpacked_vessel_id, unpacked_timestamp, unpacked_beams, unpacked_bins, unpacked_data = unpack_sonar_image(packed)

assert unpacked_vessel_id == vessel_id, f'vessel_id mismatch: {unpacked_vessel_id} != {vessel_id}'
assert abs(unpacked_timestamp - timestamp) < 1e-10, f'timestamp mismatch: {unpacked_timestamp} != {timestamp}'
assert unpacked_beams == beams, f'beams mismatch: {unpacked_beams} != {beams}'
assert unpacked_bins == range_bins, f'range_bins mismatch: {unpacked_bins} != {range_bins}'
assert np.array_equal(intensity_data, unpacked_data), 'intensity_data mismatch'

print('   ✓ Sonar image pack/unpack works correctly')
"

echo ""
echo "5. Testing depth camera pack/unpack..."
python3 -c "
import sys
import numpy as np
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.binary import pack_depth_camera, unpack_depth_camera

# Test round-trip
vessel_id, timestamp, width, height = 1, 1706400000.123, 640, 480
depth_data = np.random.randint(0, 65535, size=(height, width), dtype=np.uint16)
packed = pack_depth_camera(vessel_id, timestamp, width, height, depth_data)
unpacked_vessel_id, unpacked_timestamp, unpacked_width, unpacked_height, unpacked_data = unpack_depth_camera(packed)

assert unpacked_vessel_id == vessel_id, f'vessel_id mismatch: {unpacked_vessel_id} != {vessel_id}'
assert abs(unpacked_timestamp - timestamp) < 1e-10, f'timestamp mismatch: {unpacked_timestamp} != {timestamp}'
assert unpacked_width == width, f'width mismatch: {unpacked_width} != {width}'
assert unpacked_height == height, f'height mismatch: {unpacked_height} != {height}'
assert np.array_equal(depth_data, unpacked_data), 'depth_data mismatch'

print('   ✓ Depth camera pack/unpack works correctly')
"

echo ""
echo "6. Testing error handling..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.utils.binary import (
    pack_camera_frame,
    unpack_camera_frame,
    BinaryMessageError,
)

# Test invalid vessel_id
try:
    pack_camera_frame(-1, 0, 0.0, b'\xff\xd8')
    assert False, 'Should have raised BinaryMessageError'
except BinaryMessageError:
    pass

# Test empty JPEG data
try:
    pack_camera_frame(1, 1, 0.0, b'')
    assert False, 'Should have raised ValueError'
except ValueError:
    pass

# Test unpacking too-short data
try:
    unpack_camera_frame(b'')
    assert False, 'Should have raised BinaryMessageError'
except BinaryMessageError:
    pass

print('   ✓ Error handling works correctly')
"

echo ""
echo "7. Checking file structure..."
if [ -f "mavsim_sensor_bridge/utils/__init__.py" ]; then
    echo "   ✓ mavsim_sensor_bridge/utils/__init__.py exists"
else
    echo "   ✗ mavsim_sensor_bridge/utils/__init__.py missing"
    exit 1
fi

if [ -f "mavsim_sensor_bridge/utils/binary.py" ]; then
    echo "   ✓ mavsim_sensor_bridge/utils/binary.py exists"
else
    echo "   ✗ mavsim_sensor_bridge/utils/binary.py missing"
    exit 1
fi

if [ -f "tests/unit/test_task_1_3_binary.py" ]; then
    echo "   ✓ tests/unit/test_task_1_3_binary.py exists"
else
    echo "   ✗ tests/unit/test_task_1_3_binary.py missing"
    exit 1
fi

echo ""
echo "=========================================="
echo "All tests passed! ✓"
echo ""
echo "To run pytest tests (requires pytest installed):"
echo "  cd sensor_bridge && pytest tests/unit/test_task_1_3_binary.py -v"
