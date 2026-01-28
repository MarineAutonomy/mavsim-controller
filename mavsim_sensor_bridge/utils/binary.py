"""
Binary Message Utilities

Utilities for packing and unpacking binary sensor messages according to the
MAVSIM sensor bridge protocol specification.

Message formats:
- Camera Frame: vessel_id (uint8) + camera_id (uint8) + timestamp (float64) + JPEG data
- Lidar Scan: vessel_id (uint8) + timestamp (float64) + point_count (uint32) + points (float32 × 4 × N)
- Sonar Image: vessel_id (uint8) + timestamp (float64) + beams (uint16) + range_bins (uint16) + intensity (uint8 × N)
- Depth Camera: vessel_id (uint8) + timestamp (float64) + width (uint16) + height (uint16) + depth data (uint16 × N)
"""

import struct
from typing import Tuple
import numpy as np


# Message format constants
CAMERA_HEADER_SIZE = 10  # 1 + 1 + 8 bytes (vessel_id + camera_id + timestamp)
LIDAR_HEADER_SIZE = 13   # 1 + 8 + 4 bytes (vessel_id + timestamp + point_count)
SONAR_HEADER_SIZE = 13   # 1 + 8 + 2 + 2 bytes (vessel_id + timestamp + beams + range_bins)
DEPTH_HEADER_SIZE = 13   # 1 + 8 + 2 + 2 bytes (vessel_id + timestamp + width + height)


class BinaryMessageError(Exception):
    """Exception raised for binary message packing/unpacking errors."""
    pass


def pack_camera_frame(vessel_id: int, camera_id: int, timestamp: float, jpeg_data: bytes) -> bytes:
    """
    Pack a camera frame into binary format.
    
    Format:
        Byte 0: vessel_id (uint8)
        Byte 1: camera_id (uint8)
        Bytes 2-9: timestamp (float64)
        Bytes 10+: JPEG data (variable)
    
    Args:
        vessel_id: Vessel identifier (0-255)
        camera_id: Camera identifier (0-255)
        timestamp: Timestamp in seconds (float64)
        jpeg_data: JPEG image data as bytes
    
    Returns:
        Packed binary message
    
    Raises:
        BinaryMessageError: If vessel_id or camera_id is out of range
        ValueError: If jpeg_data is empty or invalid
    """
    if not (0 <= vessel_id <= 255):
        raise BinaryMessageError(f"vessel_id must be in range [0, 255], got {vessel_id}")
    if not (0 <= camera_id <= 255):
        raise BinaryMessageError(f"camera_id must be in range [0, 255], got {camera_id}")
    if not jpeg_data:
        raise ValueError("jpeg_data cannot be empty")
    
    # Pack header: vessel_id (uint8), camera_id (uint8), timestamp (float64)
    header = struct.pack('>BBd', vessel_id, camera_id, timestamp)
    
    # Combine header and JPEG data
    return header + jpeg_data


def unpack_camera_frame(data: bytes) -> Tuple[int, int, float, bytes]:
    """
    Unpack a camera frame from binary format.
    
    Args:
        data: Binary message data
    
    Returns:
        Tuple of (vessel_id, camera_id, timestamp, jpeg_data)
    
    Raises:
        BinaryMessageError: If data is too short or invalid format
    """
    if len(data) < CAMERA_HEADER_SIZE:
        raise BinaryMessageError(
            f"Camera frame data too short: expected at least {CAMERA_HEADER_SIZE} bytes, got {len(data)}"
        )
    
    try:
        # Unpack header
        vessel_id, camera_id, timestamp = struct.unpack('>BBd', data[:CAMERA_HEADER_SIZE])
        jpeg_data = data[CAMERA_HEADER_SIZE:]
        
        # Validate JPEG data is not empty
        if not jpeg_data:
            raise BinaryMessageError("JPEG data is empty")
        
        return int(vessel_id), int(camera_id), float(timestamp), jpeg_data
    
    except struct.error as e:
        raise BinaryMessageError(f"Failed to unpack camera frame: {e}") from e


def pack_lidar_scan(vessel_id: int, timestamp: float, points: np.ndarray) -> bytes:
    """
    Pack a lidar scan into binary format.
    
    Format:
        Byte 0: vessel_id (uint8)
        Bytes 1-8: timestamp (float64)
        Bytes 9-12: point_count (uint32)
        Bytes 13+: points (float32 × 4 × N) - each point is (x, y, z, intensity)
    
    Args:
        vessel_id: Vessel identifier (0-255)
        timestamp: Timestamp in seconds (float64)
        points: NumPy array of shape (N, 4) with dtype float32, where each row is (x, y, z, intensity)
    
    Returns:
        Packed binary message
    
    Raises:
        BinaryMessageError: If vessel_id is out of range or points array is invalid
        ValueError: If points array is empty or has wrong shape/dtype
    """
    if not (0 <= vessel_id <= 255):
        raise BinaryMessageError(f"vessel_id must be in range [0, 255], got {vessel_id}")
    
    # Ensure points is a numpy array first
    if not isinstance(points, np.ndarray):
        points = np.array(points, dtype=np.float32)
    
    # Ensure correct dtype
    if points.dtype != np.float32:
        points = points.astype(np.float32)
    
    # Ensure correct shape: (N, 4)
    if len(points.shape) != 2 or points.shape[1] != 4:
        raise ValueError(f"points must have shape (N, 4), got {points.shape}")
    
    if points.size == 0:
        raise ValueError("points array cannot be empty")
    
    point_count = points.shape[0]
    
    # Pack header: vessel_id (uint8), timestamp (float64), point_count (uint32)
    header = struct.pack('>BdI', vessel_id, timestamp, point_count)
    
    # Pack points as float32 array (big-endian)
    points_bytes = points.tobytes()
    
    return header + points_bytes


def unpack_lidar_scan(data: bytes) -> Tuple[int, float, np.ndarray]:
    """
    Unpack a lidar scan from binary format.
    
    Args:
        data: Binary message data
    
    Returns:
        Tuple of (vessel_id, timestamp, points) where points is numpy array of shape (N, 4) with dtype float32
    
    Raises:
        BinaryMessageError: If data is too short or invalid format
    """
    if len(data) < LIDAR_HEADER_SIZE:
        raise BinaryMessageError(
            f"Lidar scan data too short: expected at least {LIDAR_HEADER_SIZE} bytes, got {len(data)}"
        )
    
    try:
        # Unpack header
        vessel_id, timestamp, point_count = struct.unpack('>BdI', data[:LIDAR_HEADER_SIZE])
        
        # Calculate expected data size
        expected_size = LIDAR_HEADER_SIZE + point_count * 4 * 4  # 4 floats × 4 bytes each
        if len(data) < expected_size:
            raise BinaryMessageError(
                f"Lidar scan data too short: expected {expected_size} bytes for {point_count} points, got {len(data)}"
            )
        
        # Unpack points
        points_bytes = data[LIDAR_HEADER_SIZE:expected_size]
        points = np.frombuffer(points_bytes, dtype=np.float32).reshape(point_count, 4)
        
        return int(vessel_id), float(timestamp), points
    
    except struct.error as e:
        raise BinaryMessageError(f"Failed to unpack lidar scan header: {e}") from e
    except ValueError as e:
        raise BinaryMessageError(f"Failed to unpack lidar scan points: {e}") from e


def pack_sonar_image(vessel_id: int, timestamp: float, beams: int, range_bins: int, intensity_data: np.ndarray) -> bytes:
    """
    Pack an imaging sonar image into binary format.
    
    Format:
        Byte 0: vessel_id (uint8)
        Bytes 1-8: timestamp (float64)
        Bytes 9-10: beams (uint16)
        Bytes 11-12: range_bins (uint16)
        Bytes 13+: intensity data (uint8 × beams × range_bins)
    
    Args:
        vessel_id: Vessel identifier (0-255)
        timestamp: Timestamp in seconds (float64)
        beams: Number of beams (0-65535)
        range_bins: Number of range bins (0-65535)
        intensity_data: NumPy array of shape (beams, range_bins) or flattened, with dtype uint8
    
    Returns:
        Packed binary message
    
    Raises:
        BinaryMessageError: If vessel_id, beams, or range_bins is out of range
        ValueError: If intensity_data is empty or has wrong shape/dtype
    """
    if not (0 <= vessel_id <= 255):
        raise BinaryMessageError(f"vessel_id must be in range [0, 255], got {vessel_id}")
    if not (0 <= beams <= 65535):
        raise BinaryMessageError(f"beams must be in range [0, 65535], got {beams}")
    if not (0 <= range_bins <= 65535):
        raise BinaryMessageError(f"range_bins must be in range [0, 65535], got {range_bins}")
    
    # Ensure intensity_data is a numpy array
    if not isinstance(intensity_data, np.ndarray):
        intensity_data = np.array(intensity_data, dtype=np.uint8)
    
    # Ensure correct dtype
    if intensity_data.dtype != np.uint8:
        intensity_data = intensity_data.astype(np.uint8)
    
    # Flatten if needed and validate size
    if intensity_data.size == 0:
        raise ValueError("intensity_data cannot be empty")
    
    expected_size = beams * range_bins
    if intensity_data.size != expected_size:
        raise ValueError(
            f"intensity_data size mismatch: expected {expected_size} elements (beams={beams} × range_bins={range_bins}), "
            f"got {intensity_data.size}"
        )
    
    # Flatten to 1D array
    intensity_data = intensity_data.flatten()
    
    # Pack header: vessel_id (uint8), timestamp (float64), beams (uint16), range_bins (uint16)
    header = struct.pack('>BdHH', vessel_id, timestamp, beams, range_bins)
    
    # Pack intensity data as uint8 array
    intensity_bytes = intensity_data.tobytes()
    
    return header + intensity_bytes


def unpack_sonar_image(data: bytes) -> Tuple[int, float, int, int, np.ndarray]:
    """
    Unpack an imaging sonar image from binary format.
    
    Args:
        data: Binary message data
    
    Returns:
        Tuple of (vessel_id, timestamp, beams, range_bins, intensity_data) where intensity_data
        is numpy array of shape (beams, range_bins) with dtype uint8
    
    Raises:
        BinaryMessageError: If data is too short or invalid format
    """
    if len(data) < SONAR_HEADER_SIZE:
        raise BinaryMessageError(
            f"Sonar image data too short: expected at least {SONAR_HEADER_SIZE} bytes, got {len(data)}"
        )
    
    try:
        # Unpack header
        vessel_id, timestamp, beams, range_bins = struct.unpack('>BdHH', data[:SONAR_HEADER_SIZE])
        
        # Calculate expected data size
        expected_size = SONAR_HEADER_SIZE + beams * range_bins  # 1 byte per intensity value
        if len(data) < expected_size:
            raise BinaryMessageError(
                f"Sonar image data too short: expected {expected_size} bytes for {beams}×{range_bins} image, got {len(data)}"
            )
        
        # Unpack intensity data
        intensity_bytes = data[SONAR_HEADER_SIZE:expected_size]
        intensity_data = np.frombuffer(intensity_bytes, dtype=np.uint8).reshape(beams, range_bins)
        
        return int(vessel_id), float(timestamp), int(beams), int(range_bins), intensity_data
    
    except struct.error as e:
        raise BinaryMessageError(f"Failed to unpack sonar image header: {e}") from e
    except ValueError as e:
        raise BinaryMessageError(f"Failed to unpack sonar image data: {e}") from e


def pack_depth_camera(vessel_id: int, timestamp: float, width: int, height: int, depth_data: np.ndarray) -> bytes:
    """
    Pack a depth camera frame into binary format.
    
    Format:
        Byte 0: vessel_id (uint8)
        Bytes 1-8: timestamp (float64)
        Bytes 9-10: width (uint16)
        Bytes 11-12: height (uint16)
        Bytes 13+: depth data (uint16 × width × height)
    
    Args:
        vessel_id: Vessel identifier (0-255)
        timestamp: Timestamp in seconds (float64)
        width: Image width in pixels (0-65535)
        height: Image height in pixels (0-65535)
        depth_data: NumPy array of shape (height, width) or flattened, with dtype uint16
    
    Returns:
        Packed binary message
    
    Raises:
        BinaryMessageError: If vessel_id, width, or height is out of range
        ValueError: If depth_data is empty or has wrong shape/dtype
    """
    if not (0 <= vessel_id <= 255):
        raise BinaryMessageError(f"vessel_id must be in range [0, 255], got {vessel_id}")
    if not (0 <= width <= 65535):
        raise BinaryMessageError(f"width must be in range [0, 65535], got {width}")
    if not (0 <= height <= 65535):
        raise BinaryMessageError(f"height must be in range [0, 65535], got {height}")
    
    # Ensure depth_data is a numpy array
    if not isinstance(depth_data, np.ndarray):
        depth_data = np.array(depth_data, dtype=np.uint16)
    
    # Ensure correct dtype
    if depth_data.dtype != np.uint16:
        depth_data = depth_data.astype(np.uint16)
    
    # Flatten if needed and validate size
    if depth_data.size == 0:
        raise ValueError("depth_data cannot be empty")
    
    expected_size = width * height
    if depth_data.size != expected_size:
        raise ValueError(
            f"depth_data size mismatch: expected {expected_size} elements (width={width} × height={height}), "
            f"got {depth_data.size}"
        )
    
    # Flatten to 1D array
    depth_data = depth_data.flatten()
    
    # Pack header: vessel_id (uint8), timestamp (float64), width (uint16), height (uint16)
    header = struct.pack('>BdHH', vessel_id, timestamp, width, height)
    
    # Pack depth data as uint16 array (big-endian)
    depth_bytes = depth_data.tobytes()
    
    return header + depth_bytes


def unpack_depth_camera(data: bytes) -> Tuple[int, float, int, int, np.ndarray]:
    """
    Unpack a depth camera frame from binary format.
    
    Args:
        data: Binary message data
    
    Returns:
        Tuple of (vessel_id, timestamp, width, height, depth_data) where depth_data
        is numpy array of shape (height, width) with dtype uint16
    
    Raises:
        BinaryMessageError: If data is too short or invalid format
    """
    if len(data) < DEPTH_HEADER_SIZE:
        raise BinaryMessageError(
            f"Depth camera data too short: expected at least {DEPTH_HEADER_SIZE} bytes, got {len(data)}"
        )
    
    try:
        # Unpack header
        vessel_id, timestamp, width, height = struct.unpack('>BdHH', data[:DEPTH_HEADER_SIZE])
        
        # Calculate expected data size
        expected_size = DEPTH_HEADER_SIZE + width * height * 2  # 2 bytes per depth value (uint16)
        if len(data) < expected_size:
            raise BinaryMessageError(
                f"Depth camera data too short: expected {expected_size} bytes for {width}×{height} image, got {len(data)}"
            )
        
        # Unpack depth data
        depth_bytes = data[DEPTH_HEADER_SIZE:expected_size]
        depth_data = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(height, width)
        
        return int(vessel_id), float(timestamp), int(width), int(height), depth_data
    
    except struct.error as e:
        raise BinaryMessageError(f"Failed to unpack depth camera header: {e}") from e
    except ValueError as e:
        raise BinaryMessageError(f"Failed to unpack depth camera data: {e}") from e
