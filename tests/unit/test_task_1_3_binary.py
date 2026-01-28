"""
Unit tests for Task 1.3: Binary Message Utilities

Tests verify that binary message packing and unpacking works correctly:
- Camera frame pack/unpack round-trip
- Lidar scan pack/unpack round-trip
- Sonar image pack/unpack round-trip
- Depth camera pack/unpack round-trip
- Error handling for invalid messages
- Edge cases (empty data, out of range values, etc.)
"""

import pytest
import numpy as np
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
    CAMERA_HEADER_SIZE,
    LIDAR_HEADER_SIZE,
    SONAR_HEADER_SIZE,
    DEPTH_HEADER_SIZE,
)


# ============================================================================
# Camera Frame Tests
# ============================================================================

def test_camera_frame_roundtrip():
    """Test that camera frame pack/unpack preserves data correctly."""
    vessel_id = 1
    camera_id = 2
    timestamp = 1706400000.123
    jpeg_data = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb'
    
    packed = pack_camera_frame(vessel_id, camera_id, timestamp, jpeg_data)
    unpacked = unpack_camera_frame(packed)
    
    assert unpacked[0] == vessel_id
    assert unpacked[1] == camera_id
    assert abs(unpacked[2] - timestamp) < 1e-10  # Float comparison
    assert unpacked[3] == jpeg_data


def test_camera_frame_large_jpeg():
    """Test camera frame with large JPEG data."""
    vessel_id = 5
    camera_id = 10
    timestamp = 1234567890.987654
    jpeg_data = b'\xff\xd8' + b'\x00' * 100000  # Large JPEG
    
    packed = pack_camera_frame(vessel_id, camera_id, timestamp, jpeg_data)
    unpacked = unpack_camera_frame(packed)
    
    assert unpacked[0] == vessel_id
    assert unpacked[1] == camera_id
    assert unpacked[3] == jpeg_data
    assert len(unpacked[3]) == 100002


def test_camera_frame_edge_vessel_ids():
    """Test camera frame with edge case vessel IDs."""
    jpeg_data = b'\xff\xd8\xff'
    
    # Minimum vessel_id
    packed = pack_camera_frame(0, 0, 0.0, jpeg_data)
    unpacked = unpack_camera_frame(packed)
    assert unpacked[0] == 0
    assert unpacked[1] == 0
    
    # Maximum vessel_id
    packed = pack_camera_frame(255, 255, 0.0, jpeg_data)
    unpacked = unpack_camera_frame(packed)
    assert unpacked[0] == 255
    assert unpacked[1] == 255


def test_camera_frame_invalid_vessel_id():
    """Test that invalid vessel_id raises error."""
    jpeg_data = b'\xff\xd8\xff'
    
    with pytest.raises(BinaryMessageError):
        pack_camera_frame(-1, 0, 0.0, jpeg_data)
    
    with pytest.raises(BinaryMessageError):
        pack_camera_frame(256, 0, 0.0, jpeg_data)


def test_camera_frame_invalid_camera_id():
    """Test that invalid camera_id raises error."""
    jpeg_data = b'\xff\xd8\xff'
    
    with pytest.raises(BinaryMessageError):
        pack_camera_frame(0, -1, 0.0, jpeg_data)
    
    with pytest.raises(BinaryMessageError):
        pack_camera_frame(0, 256, 0.0, jpeg_data)


def test_camera_frame_empty_jpeg():
    """Test that empty JPEG data raises error."""
    with pytest.raises(ValueError, match="cannot be empty"):
        pack_camera_frame(1, 1, 0.0, b'')


def test_camera_frame_unpack_too_short():
    """Test that unpacking too-short data raises error."""
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_camera_frame(b'')
    
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_camera_frame(b'\x01')  # Only 1 byte


def test_camera_frame_unpack_invalid_format():
    """Test that unpacking invalid format raises error."""
    # Valid length but invalid struct format
    invalid_data = b'\x00' * CAMERA_HEADER_SIZE
    # This should still unpack (struct will succeed), but we test with corrupted data
    # Actually, let's test with data that's too short after header
    with pytest.raises(BinaryMessageError, match="JPEG data is empty"):
        unpack_camera_frame(b'\x01\x02' + b'\x00' * 8)  # Header only, no JPEG data


# ============================================================================
# Lidar Scan Tests
# ============================================================================

def test_lidar_scan_roundtrip():
    """Test that lidar scan pack/unpack preserves data correctly."""
    vessel_id = 1
    timestamp = 1706400000.0
    points = np.random.randn(1000, 4).astype(np.float32)
    
    packed = pack_lidar_scan(vessel_id, timestamp, points)
    unpacked_vessel_id, unpacked_timestamp, unpacked_points = unpack_lidar_scan(packed)
    
    assert unpacked_vessel_id == vessel_id
    assert abs(unpacked_timestamp - timestamp) < 1e-10
    assert unpacked_points.shape == points.shape
    assert unpacked_points.dtype == np.float32
    assert np.allclose(points, unpacked_points)


def test_lidar_scan_empty_points():
    """Test that empty points array raises error."""
    with pytest.raises(ValueError, match="cannot be empty"):
        pack_lidar_scan(1, 0.0, np.array([]).reshape(0, 4))


def test_lidar_scan_wrong_shape():
    """Test that wrong shape points array raises error."""
    # Wrong number of columns
    with pytest.raises(ValueError, match="shape"):
        pack_lidar_scan(1, 0.0, np.random.randn(100, 3).astype(np.float32))
    
    # 1D array
    with pytest.raises(ValueError, match="shape"):
        pack_lidar_scan(1, 0.0, np.random.randn(400).astype(np.float32))


def test_lidar_scan_dtype_conversion():
    """Test that points array dtype is converted to float32."""
    vessel_id = 1
    timestamp = 0.0
    points = np.random.randn(100, 4).astype(np.float64)  # float64 input
    
    packed = pack_lidar_scan(vessel_id, timestamp, points)
    _, _, unpacked_points = unpack_lidar_scan(packed)
    
    assert unpacked_points.dtype == np.float32


def test_lidar_scan_list_input():
    """Test that list input is converted to numpy array."""
    vessel_id = 1
    timestamp = 0.0
    points = [[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]]
    
    packed = pack_lidar_scan(vessel_id, timestamp, points)
    _, _, unpacked_points = unpack_lidar_scan(packed)
    
    assert isinstance(unpacked_points, np.ndarray)
    assert unpacked_points.shape == (2, 4)


def test_lidar_scan_large_point_cloud():
    """Test lidar scan with large point cloud."""
    vessel_id = 2
    timestamp = 1234567890.0
    points = np.random.randn(100000, 4).astype(np.float32)
    
    packed = pack_lidar_scan(vessel_id, timestamp, points)
    unpacked_vessel_id, unpacked_timestamp, unpacked_points = unpack_lidar_scan(packed)
    
    assert unpacked_vessel_id == vessel_id
    assert unpacked_points.shape == (100000, 4)
    assert np.allclose(points, unpacked_points)


def test_lidar_scan_unpack_too_short():
    """Test that unpacking too-short data raises error."""
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_lidar_scan(b'')
    
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_lidar_scan(b'\x01' + b'\x00' * 8)  # Only header, no point_count


def test_lidar_scan_unpack_incomplete_points():
    """Test that unpacking incomplete points data raises error."""
    # Pack a valid scan
    points = np.random.randn(100, 4).astype(np.float32)
    packed = pack_lidar_scan(1, 0.0, points)
    
    # Truncate the data
    truncated = packed[:-100]  # Remove some bytes
    
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_lidar_scan(truncated)


# ============================================================================
# Sonar Image Tests
# ============================================================================

def test_sonar_image_roundtrip():
    """Test that sonar image pack/unpack preserves data correctly."""
    vessel_id = 1
    timestamp = 1706400000.123
    beams = 512
    range_bins = 1024
    intensity_data = np.random.randint(0, 255, size=(beams, range_bins), dtype=np.uint8)
    
    packed = pack_sonar_image(vessel_id, timestamp, beams, range_bins, intensity_data)
    unpacked_vessel_id, unpacked_timestamp, unpacked_beams, unpacked_bins, unpacked_data = unpack_sonar_image(packed)
    
    assert unpacked_vessel_id == vessel_id
    assert abs(unpacked_timestamp - timestamp) < 1e-10
    assert unpacked_beams == beams
    assert unpacked_bins == range_bins
    assert unpacked_data.shape == (beams, range_bins)
    assert unpacked_data.dtype == np.uint8
    assert np.array_equal(intensity_data, unpacked_data)


def test_sonar_image_flattened_input():
    """Test that flattened intensity data works correctly."""
    vessel_id = 1
    timestamp = 0.0
    beams = 256
    range_bins = 512
    intensity_data = np.random.randint(0, 255, size=beams * range_bins, dtype=np.uint8)
    
    packed = pack_sonar_image(vessel_id, timestamp, beams, range_bins, intensity_data)
    _, _, unpacked_beams, unpacked_bins, unpacked_data = unpack_sonar_image(packed)
    
    assert unpacked_beams == beams
    assert unpacked_bins == range_bins
    assert unpacked_data.shape == (beams, range_bins)


def test_sonar_image_size_mismatch():
    """Test that size mismatch raises error."""
    vessel_id = 1
    timestamp = 0.0
    beams = 100
    range_bins = 200
    intensity_data = np.random.randint(0, 255, size=(50, 100), dtype=np.uint8)  # Wrong size
    
    with pytest.raises(ValueError, match="size mismatch"):
        pack_sonar_image(vessel_id, timestamp, beams, range_bins, intensity_data)


def test_sonar_image_empty_data():
    """Test that empty intensity data raises error."""
    with pytest.raises(ValueError, match="cannot be empty"):
        pack_sonar_image(1, 0.0, 0, 0, np.array([], dtype=np.uint8))


def test_sonar_image_dtype_conversion():
    """Test that intensity data dtype is converted to uint8."""
    vessel_id = 1
    timestamp = 0.0
    beams = 10
    range_bins = 20
    intensity_data = np.random.randint(0, 255, size=(beams, range_bins), dtype=np.uint16)
    
    packed = pack_sonar_image(vessel_id, timestamp, beams, range_bins, intensity_data)
    _, _, _, _, unpacked_data = unpack_sonar_image(packed)
    
    assert unpacked_data.dtype == np.uint8


def test_sonar_image_invalid_beams():
    """Test that invalid beams value raises error."""
    intensity_data = np.random.randint(0, 255, size=(100, 200), dtype=np.uint8)
    
    with pytest.raises(BinaryMessageError):
        pack_sonar_image(1, 0.0, -1, 200, intensity_data)
    
    with pytest.raises(BinaryMessageError):
        pack_sonar_image(1, 0.0, 65536, 200, intensity_data)


def test_sonar_image_unpack_too_short():
    """Test that unpacking too-short data raises error."""
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_sonar_image(b'')
    
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_sonar_image(b'\x01' + b'\x00' * 8)  # Only partial header


def test_sonar_image_unpack_incomplete_data():
    """Test that unpacking incomplete image data raises error."""
    # Pack a valid image
    beams = 100
    range_bins = 200
    intensity_data = np.random.randint(0, 255, size=(beams, range_bins), dtype=np.uint8)
    packed = pack_sonar_image(1, 0.0, beams, range_bins, intensity_data)
    
    # Truncate the data
    truncated = packed[:-1000]  # Remove some bytes
    
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_sonar_image(truncated)


# ============================================================================
# Depth Camera Tests
# ============================================================================

def test_depth_camera_roundtrip():
    """Test that depth camera pack/unpack preserves data correctly."""
    vessel_id = 1
    timestamp = 1706400000.123
    width = 640
    height = 480
    depth_data = np.random.randint(0, 65535, size=(height, width), dtype=np.uint16)
    
    packed = pack_depth_camera(vessel_id, timestamp, width, height, depth_data)
    unpacked_vessel_id, unpacked_timestamp, unpacked_width, unpacked_height, unpacked_data = unpack_depth_camera(packed)
    
    assert unpacked_vessel_id == vessel_id
    assert abs(unpacked_timestamp - timestamp) < 1e-10
    assert unpacked_width == width
    assert unpacked_height == height
    assert unpacked_data.shape == (height, width)
    assert unpacked_data.dtype == np.uint16
    assert np.array_equal(depth_data, unpacked_data)


def test_depth_camera_flattened_input():
    """Test that flattened depth data works correctly."""
    vessel_id = 1
    timestamp = 0.0
    width = 320
    height = 240
    depth_data = np.random.randint(0, 65535, size=width * height, dtype=np.uint16)
    
    packed = pack_depth_camera(vessel_id, timestamp, width, height, depth_data)
    _, _, unpacked_width, unpacked_height, unpacked_data = unpack_depth_camera(packed)
    
    assert unpacked_width == width
    assert unpacked_height == height
    assert unpacked_data.shape == (height, width)


def test_depth_camera_size_mismatch():
    """Test that size mismatch raises error."""
    vessel_id = 1
    timestamp = 0.0
    width = 640
    height = 480
    depth_data = np.random.randint(0, 65535, size=(320, 240), dtype=np.uint16)  # Wrong size
    
    with pytest.raises(ValueError, match="size mismatch"):
        pack_depth_camera(vessel_id, timestamp, width, height, depth_data)


def test_depth_camera_empty_data():
    """Test that empty depth data raises error."""
    with pytest.raises(ValueError, match="cannot be empty"):
        pack_depth_camera(1, 0.0, 0, 0, np.array([], dtype=np.uint16))


def test_depth_camera_dtype_conversion():
    """Test that depth data dtype is converted to uint16."""
    vessel_id = 1
    timestamp = 0.0
    width = 100
    height = 100
    depth_data = np.random.randint(0, 65535, size=(height, width), dtype=np.uint32)
    
    packed = pack_depth_camera(vessel_id, timestamp, width, height, depth_data)
    _, _, _, _, unpacked_data = unpack_depth_camera(packed)
    
    assert unpacked_data.dtype == np.uint16


def test_depth_camera_invalid_width():
    """Test that invalid width value raises error."""
    depth_data = np.random.randint(0, 65535, size=(100, 100), dtype=np.uint16)
    
    with pytest.raises(BinaryMessageError):
        pack_depth_camera(1, 0.0, -1, 100, depth_data)
    
    with pytest.raises(BinaryMessageError):
        pack_depth_camera(1, 0.0, 65536, 100, depth_data)


def test_depth_camera_unpack_too_short():
    """Test that unpacking too-short data raises error."""
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_depth_camera(b'')
    
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_depth_camera(b'\x01' + b'\x00' * 8)  # Only partial header


def test_depth_camera_unpack_incomplete_data():
    """Test that unpacking incomplete depth data raises error."""
    # Pack a valid depth image
    width = 640
    height = 480
    depth_data = np.random.randint(0, 65535, size=(height, width), dtype=np.uint16)
    packed = pack_depth_camera(1, 0.0, width, height, depth_data)
    
    # Truncate the data
    truncated = packed[:-1000]  # Remove some bytes
    
    with pytest.raises(BinaryMessageError, match="too short"):
        unpack_depth_camera(truncated)


# ============================================================================
# Edge Cases and Integration Tests
# ============================================================================

def test_all_message_types_together():
    """Test that all message types can be packed/unpacked correctly in sequence."""
    # Camera
    camera_packed = pack_camera_frame(1, 1, 1000.0, b'\xff\xd8\xff')
    camera_unpacked = unpack_camera_frame(camera_packed)
    assert camera_unpacked[0] == 1
    
    # Lidar
    lidar_points = np.random.randn(10, 4).astype(np.float32)
    lidar_packed = pack_lidar_scan(2, 2000.0, lidar_points)
    lidar_unpacked = unpack_lidar_scan(lidar_packed)
    assert lidar_unpacked[0] == 2
    
    # Sonar
    sonar_data = np.random.randint(0, 255, size=(10, 20), dtype=np.uint8)
    sonar_packed = pack_sonar_image(3, 3000.0, 10, 20, sonar_data)
    sonar_unpacked = unpack_sonar_image(sonar_packed)
    assert sonar_unpacked[0] == 3
    
    # Depth
    depth_data = np.random.randint(0, 65535, size=(10, 20), dtype=np.uint16)
    depth_packed = pack_depth_camera(4, 4000.0, 20, 10, depth_data)
    depth_unpacked = unpack_depth_camera(depth_packed)
    assert depth_unpacked[0] == 4


def test_message_sizes_match_specification():
    """Test that message sizes match the specification."""
    # Camera: header should be 10 bytes
    jpeg_data = b'\xff\xd8\xff'
    camera_packed = pack_camera_frame(1, 1, 0.0, jpeg_data)
    assert len(camera_packed) == CAMERA_HEADER_SIZE + len(jpeg_data)
    
    # Lidar: header should be 13 bytes
    points = np.random.randn(100, 4).astype(np.float32)
    lidar_packed = pack_lidar_scan(1, 0.0, points)
    assert len(lidar_packed) == LIDAR_HEADER_SIZE + 100 * 4 * 4  # 13 + (100 points × 4 floats × 4 bytes)
    
    # Sonar: header should be 17 bytes
    sonar_data = np.random.randint(0, 255, size=(10, 20), dtype=np.uint8)
    sonar_packed = pack_sonar_image(1, 0.0, 10, 20, sonar_data)
    assert len(sonar_packed) == SONAR_HEADER_SIZE + 10 * 20  # 17 + (10×20 uint8 values)
    
    # Depth: header should be 13 bytes
    depth_data = np.random.randint(0, 65535, size=(10, 20), dtype=np.uint16)
    depth_packed = pack_depth_camera(1, 0.0, 20, 10, depth_data)
    assert len(depth_packed) == DEPTH_HEADER_SIZE + 10 * 20 * 2  # 13 + (10×20 uint16 values × 2 bytes)
