"""
Local ROS2 Camera Publisher (Task 2.5)

Publishes camera frames to local ROS2 topics only for the vessel this controller
controls. Frames from other vessels are not published. Requires rclpy (ROS2).
"""

import logging
import threading
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Optional ROS2 imports - only used when enable_ros2 is True (controller container)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    _HAS_ROS2 = True
except ImportError:
    _HAS_ROS2 = False
    Node = None
    CompressedImage = None


class LocalROS2CameraPublisher:
    """
    Publishes camera frames to local ROS2 topics for a single controlled vessel only.

    Only frames where vessel_id matches controlled_vessel_id are published.
    Used inside the controller container so client bag recording can include
    this vessel's camera topics. Data is not sent over rosbridge.
    """

    def __init__(
        self,
        controlled_vessel_id: int,
        namespace: str,
        vessel_name: str,
        node_name: str = "mavsim_sensor_bridge",
        camera_ids: Optional[List[int]] = None,
    ):
        """
        Args:
            controlled_vessel_id: Only publish frames for this vessel_id (int 0-255).
            namespace: ROS namespace (e.g. 'sim_abc123'), no leading/trailing slashes.
            vessel_name: Vessel name for topic (e.g. 'vessel_01' or 'matsya_01').
            node_name: Name for the rclpy node.
            camera_ids: Optional list of camera IDs from config; if provided, these
                topics are pre-created at start so they appear before first frame.
                If None, publishers are created on demand when frames arrive.
        """
        if not _HAS_ROS2:
            raise RuntimeError(
                "rclpy not available. Local ROS2 publishing requires ROS2 (e.g. in controller Docker container)."
            )
        self.controlled_vessel_id = controlled_vessel_id
        self.namespace = (namespace or "").strip().strip("/")
        self.vessel_name = vessel_name or "vessel_01"
        self.node_name = node_name
        self.camera_ids = list(camera_ids) if camera_ids else None

        self._node: Optional[Node] = None
        self._publishers: Dict[int, Any] = {}  # camera_id -> rclpy Publisher
        self._publishers_lock = threading.Lock()
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False
        self._log = logging.getLogger(f"{__name__}.LocalROS2CameraPublisher")

    def _topic_name(self, camera_id: int) -> str:
        """Build topic name: /{namespace}/{vessel_name}/camera_{id:02d}/image/compressed"""
        ns = self.namespace.strip("/")
        if ns:
            return f"/{ns}/{self.vessel_name}/camera_{camera_id:02d}/image/compressed"
        return f"/{self.vessel_name}/camera_{camera_id:02d}/image/compressed"

    def start(self) -> None:
        """Initialize rclpy node and start spin thread."""
        if not _HAS_ROS2 or self._running:
            return
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node(self.node_name)
            self._running = True
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
            # Pre-create publishers for camera IDs from config (if provided) so topics
            # appear in ros2 topic list before first frame; otherwise create on demand
            if self.camera_ids:
                for cam_id in self.camera_ids:
                    self._get_publisher(cam_id)
            self._log.info(
                "Local ROS2 camera publisher started (vessel_id=%s, namespace=%s, vessel=%s)",
                self.controlled_vessel_id,
                self.namespace or "(default)",
                self.vessel_name,
            )
        except Exception as e:
            self._log.error("Failed to start ROS2 publisher: %s", e)
            self._running = False
            raise

    def _spin_loop(self) -> None:
        """Run rclpy spin in background thread."""
        while self._running and self._node and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:
                if self._running:
                    self._log.debug("Spin once error: %s", e)

    def _get_publisher(self, camera_id: int):
        """Get or create publisher for camera_id (only for controlled vessel)."""
        with self._publishers_lock:
            if camera_id not in self._publishers:
                topic = self._topic_name(camera_id)
                self._publishers[camera_id] = self._node.create_publisher(
                    CompressedImage, topic, 10
                )
                self._log.info("Created ROS2 publisher: %s", topic)
            return self._publishers[camera_id]

    def publish_frame(
        self,
        vessel_id: int,
        camera_id: int,
        timestamp: float,
        jpeg_data: bytes,
    ) -> None:
        """
        Publish a camera frame to local ROS2 only if vessel_id matches controlled vessel.

        Frames from other vessels are ignored (not published).
        """
        if not _HAS_ROS2 or not self._node or not self._running:
            return
        if vessel_id != self.controlled_vessel_id:
            return
        try:
            pub = self._get_publisher(camera_id)
            sec = int(timestamp)
            nsec = int((timestamp - sec) * 1e9)
            msg = CompressedImage()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nsec
            msg.header.frame_id = ""
            msg.format = "jpeg"
            msg.data = list(jpeg_data)
            pub.publish(msg)
        except Exception as e:
            self._log.warning("Failed to publish camera frame: %s", e)

    def stop(self) -> None:
        """Shutdown node and spin thread."""
        self._running = False
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        if self._node:
            try:
                self._node.destroy_node()
            except Exception as e:
                self._log.debug("Destroy node: %s", e)
            self._node = None
        self._publishers.clear()
        # Do not call rclpy.shutdown() - other components (e.g. recording service) may use rclpy
        self._log.info("Local ROS2 camera publisher stopped")
