#!/usr/bin/env python3
"""
Unit tests for Task 2.5: Publish Sensor Data to Local ROS2 (Client Controller Only)

Verifies that only this controller's vessel sensors are published to local ROS2;
other vessels' frames are not published. Must run in Docker with ROS2 for
full integration test (test_camera_published_to_local_topic).
"""

import sys
import os
import unittest
from unittest.mock import Mock, patch, MagicMock

# Add parent paths for imports (repo root and sensor_bridge for local/Docker)
_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
sys.path.insert(0, _root)
# Local repo: sensor_bridge/ contains mavsim_sensor_bridge. Docker: /app/sensor_bridge_setup/
for subdir in ('sensor_bridge', 'sensor_bridge_setup'):
    d = os.path.join(_root, subdir)
    if os.path.isdir(d):
        sys.path.insert(0, d)
        break
sys.path.insert(0, '/app')


class TestLocalROS2PublishFilter(unittest.TestCase):
    """Test that only controlled vessel's frames are published."""

    def test_publish_frame_ignores_other_vessel(self):
        """publish_frame must not publish when vessel_id != controlled_vessel_id."""
        try:
            from mavsim_sensor_bridge.ros2_publisher import LocalROS2CameraPublisher, _HAS_ROS2
        except ImportError:
            self.skipTest("sensor_bridge not installed")
        if not _HAS_ROS2:
            self.skipTest("rclpy not available (run in Docker with ROS2 for full tests)")
        # We test the filter logic: with a real publisher, start it then publish_frame
        # for another vessel and ensure no publish happens (or mock the node)
        with patch.object(LocalROS2CameraPublisher, 'start', return_value=None):
            with patch.object(LocalROS2CameraPublisher, '_get_publisher') as mock_get_pub:
                pub = LocalROS2CameraPublisher(
                    controlled_vessel_id=1,
                    namespace='sim_test',
                    vessel_name='vessel_01',
                )
                pub._node = MagicMock()
                pub._running = True
                mock_publisher = MagicMock()
                mock_get_pub.return_value = mock_publisher
                # Frame from vessel 2 (other vessel) -> must not publish
                pub.publish_frame(vessel_id=2, camera_id=1, timestamp=1.0, jpeg_data=b'\xff\xd8\xff')
                mock_get_pub.assert_not_called()
                mock_publisher.publish.assert_not_called()
                # Frame from vessel 1 (controlled) -> must publish
                pub.publish_frame(vessel_id=1, camera_id=1, timestamp=1.0, jpeg_data=b'\xff\xd8\xff')
                mock_get_pub.assert_called_once_with(1)
                mock_publisher.publish.assert_called_once()
        print("✓ publish_frame ignores other vessel and publishes only controlled vessel")

    def test_bridge_enable_ros2_sets_global_callback(self):
        """enable_ros2 sets global frame callback on camera server."""
        try:
            from mavsim_sensor_bridge import SensorBridge, BridgeConfig
        except ImportError:
            self.skipTest("sensor_bridge not installed")
        config = BridgeConfig(camera_port=8765, camera_enabled=True)
        bridge = SensorBridge(config=config)
        with patch('mavsim_sensor_bridge.ros2_publisher.LocalROS2CameraPublisher') as mock_cls:
            mock_pub = MagicMock()
            mock_cls.return_value = mock_pub
            bridge.enable_ros2(
                controlled_vessel_id=1,
                namespace='sim_test',
                vessel_name='vessel_01',
            )
            self.assertIsNotNone(bridge._ros2_camera_publisher)
            self.assertEqual(mock_cls.call_count, 1)
            self.assertEqual(mock_cls.call_args[1]['controlled_vessel_id'], 1)
            self.assertEqual(mock_cls.call_args[1]['namespace'], 'sim_test')
            self.assertEqual(mock_cls.call_args[1]['vessel_name'], 'vessel_01')
            # Camera server should have global callback set
            self.assertIsNotNone(bridge._servers['camera']._global_frame_callback)
        print("✓ enable_ros2 sets global callback and creates publisher for controlled vessel")

    def test_only_this_vessels_sensors_published(self):
        """enable_ros2 configures publisher with controlled_vessel_id so only this vessel is published."""
        try:
            from mavsim_sensor_bridge import SensorBridge, BridgeConfig
        except ImportError:
            self.skipTest("sensor_bridge not installed")
        config = BridgeConfig(camera_port=8765, camera_enabled=True)
        bridge = SensorBridge(config=config)
        with patch('mavsim_sensor_bridge.ros2_publisher.LocalROS2CameraPublisher') as mock_cls:
            mock_pub = MagicMock()
            mock_cls.return_value = mock_pub
            bridge.enable_ros2(
                controlled_vessel_id=1,
                namespace='sim_test',
                vessel_name='vessel_01',
            )
        # Global callback must be the publisher's publish_frame (filter is inside it)
        self.assertIs(
            bridge._servers['camera']._global_frame_callback,
            mock_pub.publish_frame,
        )
        mock_cls.assert_called_once_with(
            controlled_vessel_id=1,
            namespace='sim_test',
            vessel_name='vessel_01',
            node_name='mavsim_sensor_bridge',
        )
        # Filtering of other vessels is in LocalROS2CameraPublisher.publish_frame (test_publish_frame_ignores_other_vessel)
        print("✓ Only this vessel's sensors published (publisher filters by controlled_vessel_id)")

    def test_no_publish_over_rosbridge(self):
        """Camera data is published to local ROS2 only, not over rosbridge."""
        # Task 2.5: we use LocalROS2CameraPublisher (rclpy) in the controller container.
        # We do not republish over the existing rosbridge connection.
        try:
            from mavsim_sensor_bridge.ros2_publisher import LocalROS2CameraPublisher
        except ImportError:
            self.skipTest("sensor_bridge not installed")
        self.assertEqual(LocalROS2CameraPublisher.__module__, 'mavsim_sensor_bridge.ros2_publisher')
        # No rosbridge or roslibpy in this module
        import mavsim_sensor_bridge.ros2_publisher as mod
        self.assertFalse(hasattr(mod, 'roslibpy') or 'rosbridge' in dir(mod))
        print("✓ Local ROS2 publish uses rclpy only (no rosbridge)")


class TestCameraPublishedToLocalTopic(unittest.TestCase):
    """Integration test: subscribe to topic and verify one message (requires ROS2 in Docker)."""

    def test_camera_published_to_local_topic(self):
        """Start publisher, publish one frame, subscribe and verify one CompressedImage (requires ROS2)."""
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import CompressedImage
            from mavsim_sensor_bridge.ros2_publisher import LocalROS2CameraPublisher, _HAS_ROS2
        except ImportError:
            self.skipTest("rclpy not available (run in Docker with ROS2)")
        if not _HAS_ROS2:
            self.skipTest("rclpy not available")
        received = []
        class SubNode(Node):
            def __init__(self, topic):
                super().__init__('test_sub_node')
                self.sub = self.create_subscription(
                    CompressedImage, topic,
                    self.cb, 10,
                )
            def cb(self, msg):
                received.append(msg)
        if not rclpy.ok():
            rclpy.init()
        pub = LocalROS2CameraPublisher(
            controlled_vessel_id=1,
            namespace='sim_task25_test',
            vessel_name='vessel_01',
            node_name='test_pub_node',
        )
        pub.start()
        topic = pub._topic_name(1)
        sub_node = SubNode(topic)
        try:
            # Publish one frame
            pub.publish_frame(
                vessel_id=1,
                camera_id=1,
                timestamp=100.0,
                jpeg_data=b'\xff\xd8\xff\xe0\x00\x10JFIF',
            )
            # Spin a few times to allow message to be received
            for _ in range(50):
                rclpy.spin_once(pub._node, timeout_sec=0.01)
                rclpy.spin_once(sub_node, timeout_sec=0.01)
                if received:
                    break
            self.assertGreaterEqual(len(received), 1, "Expected at least one CompressedImage")
            self.assertEqual(received[0].format, 'jpeg')
            self.assertEqual(bytes(received[0].data), b'\xff\xd8\xff\xe0\x00\x10JFIF')
        finally:
            pub.stop()
            sub_node.destroy_node()
        print("✓ Camera frame published to local topic and received by subscriber")


if __name__ == '__main__':
    unittest.main()
