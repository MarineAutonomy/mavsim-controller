"""
Camera frames on local ROS2 (2026-11-23): each frame carries its camera's
frame_id (it used to be empty), and image/point data is handed to rclpy as
array('B'), its native type for uint8[], instead of a list it would check
and convert byte by byte on every frame.

No ROS needed: the message types are replaced by stand-ins.
"""

import array
from unittest.mock import Mock

import numpy as np
import pytest

from mavsim_sensor_bridge import ros2_publisher as rp


class Auto:
    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        value = Auto()
        setattr(self, name, value)
        return value


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setattr(rp, '_HAS_ROS2', True)
    monkeypatch.setattr(rp, 'CompressedImage', type('CompressedImage', (Auto,), {}), raising=False)
    monkeypatch.setattr(rp, 'PointCloud2', type('PointCloud2', (Auto,), {}), raising=False)
    monkeypatch.setattr(rp, 'PointField', Mock(FLOAT32=7), raising=False)


def published(pub_owner, camera_or_lidar_id):
    return pub_owner._publishers[camera_or_lidar_id].publish.call_args.args[0]


def test_camera_frame_has_its_frame_id_and_byte_array(ros):
    cam = rp.LocalROS2CameraPublisher(controlled_vessel_id=1, namespace='', vessel_name='sookshma_01')
    cam._node, cam._running = Mock(), True
    cam._publishers[2] = Mock()
    jpeg = b'\xff\xd8\xff' + bytes(range(256)) * 400
    cam.publish_frame(vessel_id=1, camera_id=2, timestamp=1700000000.25, jpeg_data=jpeg)
    msg = published(cam, 2)
    assert msg.header.frame_id == 'sookshma_01/camera_02'
    assert isinstance(msg.data, array.array) and msg.data.typecode == 'B'
    assert msg.data.tobytes() == jpeg
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1700000000, 250000000)


def test_other_vessels_frames_are_not_published(ros):
    cam = rp.LocalROS2CameraPublisher(controlled_vessel_id=1, namespace='', vessel_name='sookshma_01')
    cam._node, cam._running = Mock(), True
    cam._publishers[2] = Mock()
    cam.publish_frame(vessel_id=3, camera_id=2, timestamp=0.0, jpeg_data=b'x')
    cam._publishers[2].publish.assert_not_called()


def test_lidar_points_are_a_byte_array(ros):
    lidar = rp.LocalROS2LidarPublisher(controlled_vessel_id=1, namespace='', vessel_name='sookshma_01',
                                       node=Mock())
    lidar._running = True
    lidar._publishers[0] = Mock()
    points = np.arange(20, dtype=np.float32).reshape(5, 4)
    lidar.publish_scan(vessel_id=1, lidar_id=0, points=points, timestamp=1.0)
    msg = published(lidar, 0)
    assert isinstance(msg.data, array.array) and msg.data.typecode == 'B'
    assert msg.data.tobytes() == points.tobytes()
