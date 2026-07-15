#!/usr/bin/env python3
"""
Base Controller Class for mavsim

This class handles all connection, handshake, recording, and status reporting
automatically. Client code only needs to implement control logic.

Author: mavsim Team
License: MIT
"""

import argparse
import asyncio
import logging
import os
import re
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, Optional

from python_controller import MavsimAPIClient, MavsimController, VesselState

try:
    from local_sensor_generator import LocalSensorGenerator
except ImportError:
    LocalSensorGenerator = None

logger = logging.getLogger(__name__)


SENSOR_BRIDGE_BASE_PORT = 7000
SENSOR_BRIDGE_VESSEL_STRIDE = 10


def _sensor_bridge_ports_for_vessel(vessel_id: int, base_port: int = SENSOR_BRIDGE_BASE_PORT):
    """Compute per-vessel camera/lidar bridge ports from vessel_id."""
    vid = max(0, min(255, int(vessel_id)))
    base = int(base_port) + (vid * SENSOR_BRIDGE_VESSEL_STRIDE)
    return base + 1, base + 2


class BaseController:
    """
    Base controller class that handles all infrastructure automatically.
    
    Client code should inherit from this class and implement:
    - control_loop(vessel_states: Dict[str, VesselState]) -> Dict[str, float]
    
    vessel_states is a dict keyed by ros_name containing the full state of
    every vessel in the simulation. The control_loop method should return a
    dictionary of actuator commands:
    {
        'cs_01': 10.0,   # Control surface angle in degrees
        'th_01': 1500.0  # Thruster RPM
    }
    """
    
    def __init__(self, controller_code: str, backend_url: str = 'http://localhost:5000',
                 frontend_url: str = 'http://localhost:5173',
                 vessel_name: Optional[str] = None,
                 camera_port: Optional[int] = None, lidar_port: Optional[int] = None,
                 sensor_base_port: int = SENSOR_BRIDGE_BASE_PORT,
                 token: Optional[Dict] = None,
                 rosbridge_port: int = 9090, visualizer_port: int = 8899,
                 teleop_http_port: int = 8900, teleop_ws_port: int = 8901):
        """
        Initialize base controller with automatic handshake.

        Args:
            controller_code: Controller code from simulation UI
            backend_url: Backend URL (default: http://localhost:5000)
            frontend_url: Where the MAVSim frontend is actually hosted, same
                treatment/default pattern as backend_url - override with the
                real address (e.g. http://<server-ip>:5173) whenever the
                bridge isn't on the same machine as the frontend. Used to
                launch the headless sensor observer
                (plans/plan_headless_observer.md).
            vessel_name: Vessel name (auto-assigned if None, ignored in multi-vessel token mode)
            camera_port: Camera sensor port override (default: auto per vessel)
            lidar_port: Lidar sensor port override (default: auto per vessel)
            sensor_base_port: Base port for computed per-vessel ports (default: 7000)
            token: Optional token dict from downloaded JSON (enables multi-vessel mode)
            rosbridge_port: Local port for the rosbridge websocket that exposes
                this container's ROS2 graph to the local topic visualizer
                (default: 9090)
            visualizer_port: Local port for the ROS2 topic visualizer's Flask
                app (default: 8899)
            teleop_http_port: Local port for the keyboard teleop page
                (plans/plan_teleop.md) (default: 8900)
            teleop_ws_port: Local port for the teleop key/telemetry WebSocket
                (default: 8901)
        """
        self.controller_code = controller_code
        self.frontend_url = frontend_url
        self.backend_url = backend_url
        self.vessel_name = vessel_name
        self.camera_port = camera_port
        self.lidar_port = lidar_port
        self.sensor_base_port = sensor_base_port
        self.token = token
        self.rosbridge_port = rosbridge_port
        self.visualizer_port = visualizer_port
        self.teleop_http_port = teleop_http_port
        self.teleop_ws_port = teleop_ws_port

        # Headless sensor observer subprocess (plans/plan_headless_observer.md) -
        # launched once, on the first successful handshake in this session; see
        # _connect_single_vessel() and close().
        self._observer_process: Optional[subprocess.Popen] = None

        # Local ROS2 topic visualizer: rosbridge websocket + its Flask static/
        # API server, both launched once per session the same way, so a
        # browser can inspect local ROS2 topics (time histories, camera,
        # point cloud, camera+lidar overlay) in every mode - including CLI/
        # token mode with no other web UI running at all.
        self._rosbridge_process: Optional[subprocess.Popen] = None
        self._visualizer_process: Optional[subprocess.Popen] = None

        # Keyboard teleop node (plans/plan_teleop.md): publishes
        # interfaces/Actuator commands on /<vessel>/actuator_cmd from
        # browser keypresses, launched once per session the same way.
        self._teleop_process: Optional[subprocess.Popen] = None

        # Multi-vessel mode: list of vessel ros_names from token
        self._token_vessels = token.get('vessels', []) if token else []
        self._multi_vessel = len(self._token_vessels) > 1
        
        # Per-vessel controllers (ros_name -> MavsimController) for multi-vessel mode
        self._controllers: Dict[str, MavsimController] = {}
        self._api_clients: Dict[str, MavsimAPIClient] = {}
        self._controlled_vessels: list = []  # ros_names successfully bound
        
        # Single-vessel backward compat (set after handshake)
        self._controller: Optional[MavsimController] = None
        self._api_client: Optional[MavsimAPIClient] = None
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._control_rate_hz = 10.0
        
        # Local sensor generators (per vessel) for client-side IMU/GPS/DVL/Encoder
        self._sensor_generators: Dict[str, 'LocalSensorGenerator'] = {}
        self._ros2_sensor_pubs: Dict[str, dict] = {}
        self._ros2_republishers: Dict[str, object] = {}
        self._ros2_node = None

        # Recording state
        self._recording = False
        self._recording_topics = []
        self._recording_namespace = None
        self._recording_service = None
        self._recording_failures = 0
        
        # Recording command polling
        self._polling_thread: Optional[threading.Thread] = None
        self._polling_active = False
        self._last_command = None
        
        mode = "multi-vessel" if self._multi_vessel else "single-vessel"
        logger.info(f"BaseController initialized (code: {controller_code}, mode: {mode})")
    
    def connect(self) -> bool:
        """
        Perform handshake and connect to simulation.
        
        In multi-vessel mode (token with multiple vessels), performs one
        handshake per vessel. Vessels that return 409 (already controlled)
        are skipped with a warning.

        In code mode with no vessel_name specified, the first handshake
        discovers all available vessels, then handshakes for each remaining
        one so that all vessels are controlled by default.
        
        Returns:
            True if at least one vessel connected successfully
        """
        try:
            if self._token_vessels:
                vessels_to_bind = list(self._token_vessels)
            elif self.vessel_name:
                vessels_to_bind = [self.vessel_name]
            else:
                # No token and no vessel_name: discover all vessels by doing
                # a first handshake (auto-assigns one), then bind the rest.
                vessels_to_bind = self._discover_and_bind_all()

            for vname in vessels_to_bind:
                self._connect_single_vessel(vname)

            if not self._controlled_vessels:
                logger.error("No vessels were successfully bound")
                return False

            if len(self._controlled_vessels) > 1:
                self._multi_vessel = True

            # In single-vessel mode, keep backward-compat references
            if not self._multi_vessel and len(self._controlled_vessels) == 1:
                primary = self._controlled_vessels[0]
                self._controller = self._controllers[primary]
                self._api_client = self._api_clients[primary]
                self.vessel_name = primary
            elif self._controlled_vessels:
                primary = self._controlled_vessels[0]
                self._controller = self._controllers[primary]
                self._api_client = self._api_clients[primary]
                if not self.vessel_name:
                    self.vessel_name = primary

            # Connect each controller's rosbridge and subscribe.
            # Skip server-side odometry subscription -- odometry is generated
            # locally from vessel_state to avoid duplicate rosbridge bandwidth.
            for vname in self._controlled_vessels:
                ctrl = self._controllers[vname]
                if not ctrl.connect(subscribe_odometry=False):
                    logger.error(f"Failed to connect rosbridge for vessel {vname}")
                    continue
                if not ctrl.subscribe_vessel_states():
                    logger.warning(f"Failed to subscribe vessel states for {vname}")
                if not ctrl.subscribe_vessel_state_ders():
                    logger.warning(f"Failed to subscribe vessel state ders for {vname}")

            # Set up local sensor generators for each controlled vessel
            self._setup_local_sensors()

            # Set up local ROS2 republishers for bag recording
            self._setup_local_republishers()

            # Start recording polling (uses first api_client) LAST, after every
            # local ROS2 publisher (republishers, sensor generators, camera/lidar
            # bridge topics) has been created. Recording's topic discovery takes
            # a one-shot snapshot of the ROS2 graph on the very first poll, so if
            # polling started earlier, that snapshot could race ahead of the
            # publishers being set up above and silently omit their topics from
            # the bag (see recording_service.py for the discovery mechanism).
            self._start_polling()

            # Register any pre-connection camera/lidar callbacks
            if hasattr(self, '_camera_callbacks'):
                for vessel_id, camera_id, func in self._camera_callbacks:
                    for ctrl in self._controllers.values():
                        ctrl.on_camera(vessel_id, camera_id)(func)

            if hasattr(self, '_lidar_callbacks'):
                for vessel_id, lidar_id, func in self._lidar_callbacks:
                    for ctrl in self._controllers.values():
                        ctrl.on_lidar(vessel_id, lidar_id=lidar_id)(func)

            logger.info(
                f"Connected to simulation: {len(self._controlled_vessels)} vessel(s) "
                f"({', '.join(self._controlled_vessels)})"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to connect: {e}", exc_info=True)
            return False

    def _discover_and_bind_all(self) -> list:
        """Discover available vessels and return the full list to bind.

        Performs an initial handshake with no vessel_name. If the backend
        auto-assigns (single vessel), we get the vessel list from the
        response and bind any remaining. If the backend rejects because
        multiple vessels are available, we parse the available names from
        the error response and return them.
        """
        # Try handshake with no vessel name
        ok = self._connect_single_vessel(None)
        if ok:
            # First vessel bound -- get the rest from the handshake vessels list
            first_ctrl = self._controllers[self._controlled_vessels[0]]
            remaining = [
                v.get('ros_name') for v in first_ctrl.vessels
                if v.get('ros_name') and not v.get('controlled')
                and v.get('ros_name') != self._controlled_vessels[0]
            ]
            if remaining:
                self._multi_vessel = True
                logger.info(
                    f"Auto-controlling all vessels: already bound "
                    f"{self._controlled_vessels[0]}, remaining: {remaining}"
                )
            return remaining  # first vessel already connected, just need these

        # Handshake failed -- likely 400 "VesselName required" with multiple
        # available vessels. Re-issue the handshake to parse vessel names
        # from the error response.
        try:
            import requests
            resp = requests.post(
                f"{self.backend_url}/api/control/handshake",
                json={'controllerCode': self.controller_code},
                timeout=10
            )
            if resp.status_code == 400:
                body = resp.json()
                msg = body.get('message', '')
                if 'Available:' in msg:
                    names_str = msg.split('Available:')[1].strip()
                    names = [n.strip() for n in names_str.split(',') if n.strip()]
                    if names:
                        self._multi_vessel = len(names) > 1
                        logger.info(f"Discovered {len(names)} vessels: {names}")
                        return names
        except Exception as e:
            logger.debug(f"Vessel discovery fallback failed: {e}")

        return []

    def _connect_single_vessel(self, vessel_name: Optional[str]) -> bool:
        """Handshake and set up one vessel. Returns True on success."""
        try:
            logger.info(f"Handshaking for vessel: {vessel_name or '(auto)'}")
            api_client, handshake_data = MavsimAPIClient.from_handshake(
                backend_url=self.backend_url,
                controller_code=self.controller_code,
                vessel_name=vessel_name
            )

            session_id = handshake_data['sessionId']
            api_token = handshake_data['apiToken']
            rosbridge_url = handshake_data['rosbridgeUrl']
            namespace = handshake_data['namespace']
            bound_name = handshake_data['vesselName']
            vessel_config = handshake_data.get('vesselConfig', {})
            vessels = handshake_data.get('vessels', [])
            use_quaternion = handshake_data.get('useQuaternion', False)

            vessel_id_int = self._extract_vessel_id(bound_name, vessels)

            logger.info(
                f"Handshake OK: session={session_id[:8]}, "
                f"vessel={bound_name}, vessel_id={vessel_id_int}"
            )

            ctrl = MavsimController(
                backend_url=self.backend_url,
                session_id=session_id,
                api_token=api_token,
                rosbridge_url=rosbridge_url,
                namespace=namespace,
                vessel_name=bound_name,
                vessel_config=vessel_config,
                vessel_id=vessel_id_int,
                vessels=vessels,
                use_quaternion=use_quaternion,
            )

            # Sensors are always enabled (plans/plan_headless_observer.md) - no more
            # opt-in --enable-sensors flag.
            camera_ids = handshake_data.get('cameraIds') or []
            lidar_ids = handshake_data.get('lidarIds') or []
            hs_cam = handshake_data.get('cameraPort')
            hs_lid = handshake_data.get('lidarPort')
            if hs_cam is None or hs_lid is None:
                hs_cam, hs_lid = _sensor_bridge_ports_for_vessel(
                    vessel_id_int, base_port=self.sensor_base_port
                )
            final_cam = self.camera_port if (self.camera_port is not None and not self._multi_vessel) else hs_cam
            final_lid = self.lidar_port if (self.lidar_port is not None and not self._multi_vessel) else hs_lid
            ctrl.enable_local_sensors(
                camera_port=int(final_cam),
                lidar_port=int(final_lid),
                enable_ros2=True,
                camera_ids=camera_ids if camera_ids else None,
                lidar_ids=lidar_ids if lidar_ids else None,
            )

            self._controllers[bound_name] = ctrl
            self._api_clients[bound_name] = api_client
            self._controlled_vessels.append(bound_name)

            # Launch the headless sensor observer once per session, on the first
            # successful handshake (multi-vessel sessions handshake once per vessel,
            # but one browser tab renders and streams all of them - see
            # getOrCreateSensorStreamManagerForVessel in SimulationPage.js).
            if self._observer_process is None:
                self._launch_observer(session_id, api_token, namespace)

            # Local ROS2 topic visualizer: rosbridge + its Flask server, also
            # launched once per session, in every mode.
            if self._rosbridge_process is None:
                self._reset_visualizer_state_file()
                self._launch_rosbridge()
            if self._visualizer_process is None:
                self._launch_visualizer_server()

            # Sensor extrinsics/intrinsics snapshot for the visualizer's camera+
            # lidar overlay - fetched once per vessel (each vessel needs its own
            # sensor list) and merged into the same shared state file.
            self._fetch_and_cache_sensor_config(session_id, api_token, bound_name)

            # Keyboard teleop (plans/plan_teleop.md): launched once per
            # session like rosbridge/visualizer above. Its per-vessel
            # thruster/control-surface geometry is already in-process on
            # `ctrl` (no HTTP round-trip needed, unlike the sensor-config
            # snapshot above), so it's written directly instead of fetched.
            if self._teleop_process is None:
                self._reset_teleop_state_file()
                self._launch_teleop()
            self._write_teleop_config(bound_name, ctrl)

            return True

        except Exception as e:
            err_str = str(e)
            if '409' in err_str or 'already controlled' in err_str.lower():
                logger.warning(f"Vessel {vessel_name} already controlled (409), skipping")
            else:
                logger.error(f"Handshake failed for vessel {vessel_name}: {e}", exc_info=True)
            return False

    @staticmethod
    def _extract_vessel_id(vessel_name: str, vessels: list) -> int:
        """Extract numeric vessel_id from handshake vessels list or name suffix."""
        for v in vessels:
            if v.get('ros_name') == vessel_name or v.get('name') == vessel_name:
                try:
                    vid = v.get('vessel_id')
                    if vid is not None and vid != '':
                        return int(vid) if isinstance(vid, int) else int(str(vid).strip())
                except (TypeError, ValueError):
                    pass
        m = re.search(r'_(\d+)$', vessel_name or '')
        return int(m.group(1)) if m else 0
    
    # ------------------------------------------------------------------
    # Local sensor generation
    # ------------------------------------------------------------------

    def _setup_local_sensors(self):
        """Initialize LocalSensorGenerator for each controlled vessel."""
        if LocalSensorGenerator is None:
            logger.info("LocalSensorGenerator not available, skipping client-side sensors")
            return

        for vname in self._controlled_vessels:
            ctrl = self._controllers.get(vname)
            if not ctrl:
                continue

            vessel_config = ctrl.vessel_config
            if not vessel_config:
                logger.debug(f"No vessel config for {vname}, skipping sensor generator")
                continue

            use_quat = ctrl.use_quaternion

            def make_callback(vessel_ros_name):
                def on_measurement(sensor_type, topic, measurement):
                    self._publish_sensor_ros2(sensor_type, topic, measurement)
                return on_measurement

            gen = LocalSensorGenerator(
                vessel_config=vessel_config,
                vessel_name=vname,
                use_quaternion=use_quat,
                on_measurement=make_callback(vname),
            )
            self._sensor_generators[vname] = gen

            if gen.sensor_count > 0:
                gen.start()
                logger.info(
                    f"Local sensor generator started for {vname}: "
                    f"{gen.sensor_count} sensor(s)"
                )

    def _setup_local_republishers(self):
        """Set up local ROS2 publishers for bag recording.

        rosbridge WebSocket data doesn't appear on the local ROS2 graph, so we
        republish vessel_state and vessel_state_der to local topics. odometry_sim
        is generated locally from vessel_state (not subscribed from server) to
        avoid duplicate rosbridge bandwidth.
        """
        self._init_ros2_sensor_node()
        if self._ros2_node is None:
            return

        try:
            from std_msgs.msg import Float64MultiArray
            from nav_msgs.msg import Odometry
        except ImportError:
            logger.warning("ROS2 message types not available, skipping republishers")
            return

        for vname in self._controlled_vessels:
            ctrl = self._controllers.get(vname)
            if not ctrl:
                continue

            prefix = f"/{vname}"

            state_topic = f"{prefix}/vessel_state"
            state_der_topic = f"{prefix}/vessel_state_der"
            odom_topic = f"{prefix}/odometry_sim"

            pub_state = self._ros2_node.create_publisher(Float64MultiArray, state_topic, 10)
            pub_state_der = self._ros2_node.create_publisher(Float64MultiArray, state_der_topic, 10)
            pub_odom = self._ros2_node.create_publisher(Odometry, odom_topic, 10)

            self._ros2_republishers[f"{vname}/vessel_state"] = pub_state
            self._ros2_republishers[f"{vname}/vessel_state_der"] = pub_state_der
            self._ros2_republishers[f"{vname}/odometry_sim"] = pub_odom

            logger.info(f"Local ROS2 publishers created for {vname}: "
                        f"{state_topic}, {state_der_topic}, {odom_topic}")

    def _republish_to_local_ros2(self):
        """Republish rosbridge data and generate odometry on local ROS2 topics."""
        if not self._ros2_republishers:
            return

        try:
            from std_msgs.msg import Float64MultiArray
        except ImportError:
            return

        for vname in self._controlled_vessels:
            ctrl = self._controllers.get(vname)
            if not ctrl:
                continue

            raw_state = ctrl.get_raw_state(vname)
            if raw_state is not None:
                pub = self._ros2_republishers.get(f"{vname}/vessel_state")
                if pub:
                    msg = Float64MultiArray()
                    msg.data = [float(x) for x in raw_state]
                    pub.publish(msg)

                # Generate odometry_sim locally from vessel_state instead of
                # subscribing to a separate server topic.
                pub_odom = self._ros2_republishers.get(f"{vname}/odometry_sim")
                if pub_odom:
                    self._publish_local_odometry(pub_odom, raw_state, ctrl.use_quaternion)

            raw_der = ctrl.get_raw_state_der(vname)
            if raw_der is not None:
                pub = self._ros2_republishers.get(f"{vname}/vessel_state_der")
                if pub:
                    msg = Float64MultiArray()
                    msg.data = [float(x) for x in raw_der]
                    pub.publish(msg)

    @staticmethod
    def _publish_local_odometry(pub, raw_state, use_quaternion: bool):
        """Build and publish an Odometry message from the raw vessel_state array.

        State layout (after timestamp strip):
        [u, v, w, p, q, r, x, y, z, phi/q0, theta/q1, psi/q2, (q3), ...]
        """
        try:
            from nav_msgs.msg import Odometry as OdometryMsg
            from geometry_msgs.msg import Point, Quaternion as QuatMsg, Vector3
            import numpy as np

            msg = OdometryMsg()
            msg.header.frame_id = 'NED'
            msg.child_frame_id = 'BODY'

            s = raw_state
            msg.pose.pose.position = Point(x=float(s[6]), y=float(s[7]), z=float(s[8]))

            if use_quaternion:
                msg.pose.pose.orientation = QuatMsg(
                    x=float(s[10]), y=float(s[11]), z=float(s[12]), w=float(s[9])
                )
            else:
                from local_sensor_generator import _eul_to_quat
                q = _eul_to_quat(np.array([s[9], s[10], s[11]]))
                msg.pose.pose.orientation = QuatMsg(
                    x=float(q[1]), y=float(q[2]), z=float(q[3]), w=float(q[0])
                )

            msg.twist.twist.linear = Vector3(x=float(s[0]), y=float(s[1]), z=float(s[2]))
            msg.twist.twist.angular = Vector3(x=float(s[3]), y=float(s[4]), z=float(s[5]))

            msg.pose.covariance = [0.0] * 36
            msg.twist.covariance = [0.0] * 36

            pub.publish(msg)
        except Exception as e:
            logger.debug(f"Error generating local odometry: {e}")

    def _update_sensor_states(self):
        """Feed latest raw state/state_der from rosbridge into sensor generators."""
        for vname, gen in self._sensor_generators.items():
            ctrl = self._controllers.get(vname)
            if not ctrl:
                continue
            state = ctrl.get_raw_state(vname)
            state_der = ctrl.get_raw_state_der(vname)
            if state is not None:
                gen.update_state(state, state_der)

    def _init_ros2_sensor_node(self):
        """Lazily initialize a local ROS2 node for publishing sensor topics."""
        if self._ros2_node is not None:
            return
        try:
            import rclpy
            from rclpy.node import Node
            if not rclpy.ok():
                rclpy.init()
            self._ros2_node = rclpy.create_node('local_sensor_publisher')
            logger.info("Created local ROS2 node for sensor publishing")
        except Exception as e:
            logger.warning(f"Failed to create ROS2 sensor node: {e}")

    def _publish_sensor_ros2(self, sensor_type: str, topic: str, measurement: dict):
        """Publish a sensor measurement to a local ROS2 topic."""
        self._init_ros2_sensor_node()
        if self._ros2_node is None:
            return

        try:
            if topic not in self._ros2_sensor_pubs:
                msg_type = self._get_sensor_msg_type(sensor_type)
                if msg_type is None:
                    return
                pub = self._ros2_node.create_publisher(msg_type, topic, 10)
                self._ros2_sensor_pubs[topic] = {'pub': pub, 'type': sensor_type, 'msg_type': msg_type}

            pub_info = self._ros2_sensor_pubs[topic]
            msg = self._build_sensor_msg(pub_info['type'], measurement)
            if msg is not None:
                pub_info['pub'].publish(msg)
        except Exception as e:
            logger.debug(f"ROS2 sensor publish error for {topic}: {e}")

    @staticmethod
    def _get_sensor_msg_type(sensor_type: str):
        """Return the ROS2 message type for a given sensor type."""
        try:
            stype = sensor_type.lower()
            if stype == 'imu':
                from sensor_msgs.msg import Imu
                return Imu
            elif stype == 'gps':
                from sensor_msgs.msg import NavSatFix
                return NavSatFix
            elif stype == 'dvl':
                from geometry_msgs.msg import TwistWithCovarianceStamped
                return TwistWithCovarianceStamped
            elif stype == 'encoder':
                from std_msgs.msg import Float64MultiArray
                return Float64MultiArray
        except ImportError:
            pass
        return None

    @staticmethod
    def _build_sensor_msg(sensor_type: str, measurement: dict):
        """Build a ROS2 message from a sensor measurement dict."""
        import numpy as np
        stype = sensor_type.lower()
        try:
            if stype == 'imu':
                from sensor_msgs.msg import Imu
                from geometry_msgs.msg import Quaternion, Vector3
                msg = Imu()
                q = measurement['orientation']
                msg.orientation = Quaternion(w=float(q[0]), x=float(q[1]),
                                             y=float(q[2]), z=float(q[3]))
                av = measurement['angular_velocity']
                msg.angular_velocity = Vector3(x=float(av[0]), y=float(av[1]), z=float(av[2]))
                la = measurement['linear_acceleration']
                msg.linear_acceleration = Vector3(x=float(la[0]), y=float(la[1]), z=float(la[2]))
                msg.orientation_covariance = [float(x) for x in measurement['orientation_covariance']]
                msg.angular_velocity_covariance = [float(x) for x in measurement['angular_velocity_covariance']]
                msg.linear_acceleration_covariance = [float(x) for x in measurement['linear_acceleration_covariance']]
                return msg

            elif stype == 'gps':
                from sensor_msgs.msg import NavSatFix
                msg = NavSatFix()
                msg.latitude = float(measurement['latitude'])
                msg.longitude = float(measurement['longitude'])
                msg.altitude = float(measurement['altitude'])
                cov = measurement['position_covariance']
                msg.position_covariance = [float(x) for x in cov]
                msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
                return msg

            elif stype == 'dvl':
                from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3
                msg = TwistWithCovarianceStamped()
                vel = measurement['velocity']
                msg.twist.twist.linear = Vector3(
                    x=float(vel[0]), y=float(vel[1]), z=float(vel[2])
                )
                cov_3x3 = np.array(measurement['covariance']).reshape(3, 3)
                cov_6x6 = np.zeros((6, 6))
                cov_6x6[:3, :3] = cov_3x3
                msg.twist.covariance = [float(x) for x in cov_6x6.flatten()]
                return msg

            elif stype == 'encoder':
                from std_msgs.msg import Float64MultiArray
                msg = Float64MultiArray()
                msg.data = [float(v) for v in measurement.get('actuator_values', [])]
                return msg

        except Exception as e:
            logger.debug(f"Failed to build {stype} message: {e}")
        return None

    def start_recording(self, topics: Optional[list] = None, namespace: Optional[str] = None):
        """
        Start recording ROS2 topics to bag file.
        
        Args:
            topics: List of topics to record (None = record all vessel topics)
            namespace: Topic discovery prefix. On the local container, vessel
                       names are used (no simulation namespace).
        """
        if self._recording:
            logger.warning("Recording already in progress")
            return

        if not self._api_client:
            logger.error("Not connected - cannot start recording")
            return

        # Don't attempt recording until rosbridge is connected; topics
        # won't be visible in the local ROS2 graph until then.
        if self._controller and not self._controller.subscriber.is_connected():
            logger.debug("Rosbridge not connected yet — deferring recording start")
            return
        
        # For local topic discovery, use vessel name(s) as prefix instead of
        # the simulation namespace since local topics omit the namespace.
        if namespace:
            self._recording_namespace = namespace
        elif self._controlled_vessels:
            self._recording_namespace = f"/{self._controlled_vessels[0]}"
        elif self.vessel_name:
            self._recording_namespace = f"/{self.vessel_name}"
        elif self._controller:
            self._recording_namespace = self._controller.namespace
        
        if not self._recording_namespace:
            logger.error("No namespace/vessel prefix available for recording")
            return
        
        # For multi-vessel auto-discovery (topics=None), record every
        # controlled vessel's namespace via regex instead of pre-discovering
        # a static topic list -- RecordingService keeps matching new topics
        # (e.g. a camera publisher that only appears once the browser sends
        # its first frame) for the life of the recording, rather than
        # freezing the topic list at this instant.
        namespaces = None
        if topics is None and len(self._controlled_vessels) > 1:
            namespaces = [f"/{v}" for v in self._controlled_vessels]

        self._recording_topics = topics

        try:
            from recording_service import RecordingService
            self._recording_service = RecordingService()

            success = self._recording_service.start_recording(
                namespace=self._recording_namespace,
                topics=topics,
                namespaces=namespaces,
            )
            
            if success:
                self._recording = True
                self._recording_failures = 0
                self._report_recording_status('recording', topics or [])
                logger.info(f"Recording started: prefix={self._recording_namespace}, topics={topics or 'all'}")
            else:
                self._recording_failures += 1
                if self._recording_failures <= 3:
                    logger.warning(
                        f"Topic discovery returned no topics (attempt {self._recording_failures}/3) "
                        "— will retry on next poll"
                    )
                elif self._recording_failures == 4:
                    logger.error("Recording failed after multiple attempts — giving up")
                    self._report_recording_status('error', topics or [],
                                                  error="No topics discovered after retries")
                
        except ImportError:
            logger.warning("RecordingService not available (ROS2 not installed?) - recording disabled")
            self._report_recording_status('recording', topics or [])
            self._recording = True
        except Exception as e:
            error_msg = f"Failed to start recording: {e}"
            logger.error(error_msg, exc_info=True)
            self._report_recording_status('error', topics or [], error=error_msg)
    
    def stop_recording(self):
        """Stop recording and upload the bag to the cloud backend."""
        if not self._recording:
            logger.warning("No recording in progress")
            return
        
        try:
            bag_path = None
            if self._recording_service:
                bag_path = self._recording_service.stop_recording()
                if bag_path:
                    logger.info(f"Bag file saved: {bag_path}")
                self._recording_service = None
            
            self._recording = False
            self._report_recording_status('stopped', self._recording_topics)
            logger.info("Recording stopped")

            # Upload to cloud backend in a background thread so it doesn't
            # block the polling loop or controller shutdown.
            if bag_path and self._api_client:
                upload_thread = threading.Thread(
                    target=self._upload_bag_async,
                    args=(bag_path,),
                    daemon=True,
                )
                upload_thread.start()

        except Exception as e:
            logger.error(f"Failed to stop recording: {e}")
            self._report_recording_status('error', self._recording_topics, error=str(e))

    def _upload_bag_async(self, bag_path: str):
        """Background upload of a bag file to the cloud backend."""
        try:
            from recording_service import upload_bag
            ok = upload_bag(
                bag_path=bag_path,
                backend_url=self.backend_url,
                session_id=self._api_client.session_id,
                api_token=self._api_client.api_token,
            )
            if ok:
                logger.info("Bag uploaded to cloud backend successfully")
            else:
                logger.warning("Bag upload to cloud backend failed (non-fatal)")
        except Exception as e:
            logger.warning(f"Bag upload error (non-fatal): {e}")
    
    def poll_recording_command(self) -> Optional[Dict]:
        """
        Poll backend for recording command.
        
        Returns:
            Dict with 'command', 'namespace', 'topics' or None if error
        """
        if not self._api_client:
            return None
        
        try:
            import requests
            url = f"{self.backend_url}/api/simulation/recording/command"
            params = {
                'session_id': self._api_client.session_id,
                'api_token': self._api_client.api_token
            }
            
            response = requests.get(url, params=params, timeout=5.0)
            response.raise_for_status()
            
            data = response.json()
            self._last_command = data
            return data
            
        except Exception as e:
            logger.debug(f"Failed to poll recording command: {e}")
            return None
    
    def _poll_recording_loop(self):
        """Background thread that polls for recording commands."""
        MAX_RECORDING_RETRIES = 10
        while self._polling_active:
            try:
                command_data = self.poll_recording_command()
                
                if command_data:
                    command = command_data.get('command')
                    namespace = command_data.get('namespace')
                    topics = command_data.get('topics')
                    
                    if command == 'start' and not self._recording:
                        if self._recording_failures < MAX_RECORDING_RETRIES:
                            self.start_recording(topics=topics, namespace=namespace)
                    elif command == 'stop' and self._recording:
                        logger.info("Received stop recording command")
                        self.stop_recording()
                
            except Exception as e:
                logger.error(f"Error in polling loop: {e}", exc_info=True)
            
            time.sleep(2.0)
    
    def _start_polling(self):
        """Start background polling thread."""
        if self._polling_active:
            return
        
        self._polling_active = True
        self._polling_thread = threading.Thread(
            target=self._poll_recording_loop,
            daemon=True
        )
        self._polling_thread.start()
        logger.info("Started recording command polling")
        
        # Poll once immediately
        self.poll_recording_command()
    
    def _stop_polling(self):
        """Stop background polling thread."""
        if not self._polling_active:
            return
        
        self._polling_active = False
        if self._polling_thread and self._polling_thread.is_alive():
            self._polling_thread.join(timeout=3.0)
        logger.info("Stopped recording command polling")
    
    def _report_recording_status(self, status: str, topics: list, error: Optional[str] = None):
        """Report recording status to backend."""
        if not self._api_client:
            return
        
        # Task 2.4: Report status to backend
        try:
            self._api_client.report_recording_status(
                status=status,
                topics=topics if topics else None,
                error=error
            )
        except Exception as e:
            # Log but don't fail - status reporting is best-effort
            logger.warning(f"Failed to report recording status: {e}")
    
    def start_control_loop(self, rate_hz: float = 10.0):
        """
        Start automatic control loop that calls client's control_loop method.
        
        Args:
            rate_hz: Control loop frequency in Hz (default: 10.0)
        """
        if self._running:
            logger.warning("Control loop already running")
            return
        
        if not self._controller:
            logger.error("Not connected - cannot start control loop")
            return
        
        self._control_rate_hz = rate_hz
        self._running = True
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True
        )
        self._control_thread.start()
        logger.info(f"Control loop started at {rate_hz} Hz")
    
    def _control_loop(self):
        """Internal control loop that calls client's control_loop method.
        
        In multi-vessel mode, the user's control_loop() may return:
        - A flat dict {'cs_01': 5.0, ...} -> sent to the primary vessel
        - A nested dict {'matsya_02': {'cs_01': 5.0}, 'matsya_03': {...}} -> per-vessel dispatch
        """
        period = 1.0 / self._control_rate_hz
        
        while self._running:
            try:
                # Feed latest state into local sensor generators
                if self._sensor_generators:
                    self._update_sensor_states()

                # Republish server data to local ROS2 for bag recording
                if self._ros2_republishers:
                    self._republish_to_local_ros2()

                # Aggregate vessel states from all controllers
                vessel_states = {}
                for vname, ctrl in self._controllers.items():
                    vs = ctrl.get_vessel_states()
                    if vs:
                        vessel_states.update(vs)
                # Fallback for single-vessel mode
                if not vessel_states and self._controller:
                    vessel_states = self._controller.get_vessel_states() or {}

                if not vessel_states:
                    time.sleep(period)
                    continue
                
                result = self.control_loop(vessel_states)
                
                if result:
                    self._dispatch_commands(result)
                
            except Exception as e:
                logger.error(f"Control loop error: {e}", exc_info=True)
            
            time.sleep(period)

    def _dispatch_commands(self, result: dict):
        """Dispatch control_loop return value to the appropriate vessel controllers."""
        if not result:
            return

        # Detect whether result is per-vessel (nested) or flat (single-vessel)
        first_val = next(iter(result.values()), None)
        is_nested = isinstance(first_val, dict)

        if is_nested:
            for vname, cmds in result.items():
                ctrl = self._controllers.get(vname)
                if ctrl and cmds:
                    names = list(cmds.keys())
                    values = [cmds[n] for n in names]
                    ctrl.send_command(names, values)
        else:
            # Flat dict -> send to the primary controller
            ctrl = self._controller
            if ctrl and result:
                names = list(result.keys())
                values = [result[n] for n in names]
                ctrl.send_command(names, values)
    
    def control_loop(self, vessel_states: Dict[str, VesselState]) -> Dict[str, float]:
        """
        Client's control loop implementation.
        
        Override this method in your controller class.
        
        Args:
            vessel_states: Dict mapping ros_name -> VesselState for every
                           vessel in the simulation. Use self.vessel_name to
                           look up your own vessel's state.
            
        Returns:
            Dictionary of actuator commands:
            {
                'cs_01': 10.0,   # Control surface angle in degrees
                'th_01': 1500.0  # Thruster RPM
            }
        """
        # Default: do nothing
        return {}
    
    def on_camera(self, vessel_id: int, camera_id: int):
        """
        Decorator to register camera frame callback.
        
        Usage:
            @controller.on_camera(vessel_id=1, camera_id=1)
            def handle_frame(vessel_id, camera_id, timestamp, jpeg_data):
                # Process frame
                pass
        """
        def decorator(func):
            # Store callback - will register when controller is connected
            if not hasattr(self, '_camera_callbacks'):
                self._camera_callbacks = []
            self._camera_callbacks.append((vessel_id, camera_id, func))
            
            # If already connected, register immediately
            if self._controller:
                self._controller.on_camera(vessel_id, camera_id)(func)
            
            return func
        return decorator

    def on_lidar(self, vessel_id: int, lidar_id: int = 0):
        """
        Decorator to register lidar scan callback.

        Usage:
            @controller.on_lidar(vessel_id=1, lidar_id=0)
            def handle_scan(vessel_id, lidar_id, points, timestamp):
                # points: numpy (N, 4) float32 — x, y, z, intensity
                pass
        """
        def decorator(func):
            if not hasattr(self, '_lidar_callbacks'):
                self._lidar_callbacks = []
            self._lidar_callbacks.append((vessel_id, lidar_id, func))

            if self._controller:
                self._controller.on_lidar(vessel_id, lidar_id=lidar_id)(func)

            return func
        return decorator

    def get_vessel_id_map(self) -> Dict[str, int]:
        """Return a mapping from vessel ros_name to numeric vessel_id.

        Useful for correlating camera/lidar callbacks (keyed by int vessel_id)
        with vessel names used in control_loop.
        """
        result = {}
        for vname, ctrl in self._controllers.items():
            result[vname] = ctrl.vessel_id
        return result
    
    def get_control_surface_names(self, vessel_name: Optional[str] = None) -> list:
        """
        Get list of control surface actuator names for a vessel.

        Args:
            vessel_name: ROS name of the vessel (e.g. 'matsya_02'). If None,
                uses the primary vessel (backward compatible with single-vessel).

        Returns:
            List of actuator name strings, e.g. ['cs_02'] for a vessel with one
            control surface whose actuator_id is 2.
        """
        ctrl = self._controllers.get(vessel_name) if vessel_name else self._controller
        if not ctrl:
            return []
        cs_list = self._extract_control_surfaces(ctrl)
        return [f'cs_{cs.get("actuator_id", 0):02d}' for cs in cs_list]

    def get_thruster_names(self, vessel_name: Optional[str] = None) -> list:
        """
        Get list of thruster actuator names for a vessel.

        Args:
            vessel_name: ROS name of the vessel (e.g. 'matsya_02'). If None,
                uses the primary vessel (backward compatible with single-vessel).

        Returns:
            List of actuator name strings, e.g. ['th_01'] for a vessel with one
            thruster whose actuator_id is 1.
        """
        ctrl = self._controllers.get(vessel_name) if vessel_name else self._controller
        if not ctrl:
            return []
        th_list = self._extract_thrusters(ctrl)
        return [f'th_{th.get("actuator_id", 0):02d}' for th in th_list]

    @staticmethod
    def _extract_control_surfaces(ctrl) -> list:
        """Extract control surface list from controller (handles nested config)."""
        raw = ctrl.control_surfaces
        if isinstance(raw, list):
            return raw
        if isinstance(raw, dict):
            return raw.get('control_surfaces', []) or []
        return []

    @staticmethod
    def _extract_thrusters(ctrl) -> list:
        """Extract thruster list from controller (handles nested config)."""
        raw = ctrl.thrusters
        if isinstance(raw, list):
            return raw
        if isinstance(raw, dict):
            return raw.get('thrusters', []) or []
        return []
    
    def get_vessel_states(self) -> Dict[str, VesselState]:
        """Return a snapshot dict of all vessel states keyed by ros_name."""
        states = {}
        for ctrl in self._controllers.values():
            vs = ctrl.get_vessel_states()
            if vs:
                states.update(vs)
        if not states and self._controller:
            return self._controller.get_vessel_states() or {}
        return states

    def get_vessel_state(self, ros_name: str) -> Optional[VesselState]:
        """Return the latest state for a single vessel, or None."""
        for ctrl in self._controllers.values():
            st = ctrl.get_vessel_state(ros_name)
            if st is not None:
                return st
        if self._controller:
            return self._controller.get_vessel_state(ros_name)
        return None
    
    def send_command(self, actuator_commands: Dict[str, float], vessel_name: Optional[str] = None) -> bool:
        """
        Send actuator commands directly.
        
        Args:
            actuator_commands: Dictionary of actuator commands
            vessel_name: Target vessel ros_name (default: primary vessel)
            
        Returns:
            True if successful
        """
        ctrl = None
        if vessel_name and vessel_name in self._controllers:
            ctrl = self._controllers[vessel_name]
        elif self._controller:
            ctrl = self._controller
        
        if not ctrl:
            return False
        
        actuator_names = list(actuator_commands.keys())
        actuator_values = [actuator_commands[name] for name in actuator_names]
        return ctrl.send_command(actuator_names, actuator_values)

    def _zero_all_actuators(self):
        """Send zero commands for all actuators on all controlled vessels."""
        for vname, ctrl in self._controllers.items():
            try:
                cs_list = self._extract_control_surfaces(ctrl)
                th_list = self._extract_thrusters(ctrl)
                names = (
                    [f'cs_{cs.get("actuator_id", 0):02d}' for cs in cs_list]
                    + [f'th_{th.get("actuator_id", 0):02d}' for th in th_list]
                )
                if names:
                    values = [0.0] * len(names)
                    ctrl.send_command(names, values)
                    logger.info(f"Zeroed {len(names)} actuators for {vname}")
            except Exception as e:
                logger.debug(f"Failed to zero actuators for {vname}: {e}")
    
    def _launch_observer(self, session_id: str, api_token: str, namespace: str):
        """
        Launch the headless sensor observer subprocess (plans/plan_headless_observer.md).

        Runs in the same container as this controller - not a separate Docker
        container - so it inherits this process's stdout (captured into
        bridge_webapp.py's /api/logs in web mode, printed directly to
        `docker logs` otherwise) and is cleaned up as a normal child process
        in close(), with no cross-container signaling needed.
        """
        try:
            observer_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "observer.py")
            self._observer_process = subprocess.Popen([
                sys.executable, observer_script,
                "--session-id", session_id,
                "--api-token", api_token,
                "--namespace", namespace or "",
                "--frontend-url", self.frontend_url,
            ], start_new_session=True)
            logger.info(f"Launched headless sensor observer (pid={self._observer_process.pid})")
        except Exception as e:
            # Best-effort: a failed observer launch must not break the actual
            # control connection.
            logger.warning(f"Failed to launch headless sensor observer: {e}")
            self._observer_process = None

    def _reset_visualizer_state_file(self):
        """
        Clear the shared sensor_config.json state file at the start of a
        fresh session, before any vessel writes into it.

        _fetch_and_cache_sensor_config() merges each vessel's entry into
        this file (existing[vessel_name] = {...}) and never removes one -
        by design, since a single session can have several vessels handshake
        at different times and each write must not clobber its siblings.
        But the file itself lives at a fixed, session-independent path
        (/tmp/mavsim_bridge_state/sensor_config.json), so without this reset
        a vessel from a previous, already-stopped session (e.g. a container
        that was `docker rm -f`'d, or crashed, before close() ever ran) would
        keep showing up forever in every subsequent session's visualizer
        vessel dropdown - the dropdown is populated purely from this file's
        keys (visualizer_server.py's loadSensorConfig()), with no live-topic
        cross-check. Best effort: only ever called once, right before this
        session's own rosbridge/visualizer subprocesses start, so it can't
        race with or clobber this same session's later per-vessel writes.
        """
        try:
            state_file = os.path.join("/tmp/mavsim_bridge_state", "sensor_config.json")
            if os.path.exists(state_file):
                os.remove(state_file)
        except OSError as e:
            logger.debug(f"Failed to reset visualizer state file: {e}")

    def _launch_rosbridge(self):
        """
        Launch the rosbridge websocket server, exposing this container's local
        ROS2 graph to a browser (the local ROS2 topic visualizer). This is a
        different rosbridge instance from the simulation's own remote one -
        it only ever sees this container's local topics.

        Uses the rosbridge_websocket_launch.xml launch file (not a bare
        `ros2 run rosbridge_server rosbridge_websocket`) because it also
        brings up rosapi_node, which the visualizer needs for live topic
        discovery (/rosapi/topics) - camera/lidar topic IDs assigned at
        publish time don't always match the config's sensor_id (a pre-
        existing sensor_bridge quirk), so the visualizer can't just guess
        topic names from the config; it asks the ROS graph directly.
        """
        try:
            # start_new_session=True (setsid) makes this process the leader of
            # a brand-new process group, which every node `ros2 launch` itself
            # spawns (rosbridge_websocket, rosapi, rosapi_params) inherits -
            # required so close() can kill the whole group at once instead of
            # just this one PID (see close()'s os.killpg usage).
            self._rosbridge_process = subprocess.Popen([
                "ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml",
                f"port:={self.rosbridge_port}",
            ], start_new_session=True)
            logger.info(f"Launched rosbridge websocket (+ rosapi) on port {self.rosbridge_port} "
                        f"(pid={self._rosbridge_process.pid})")
        except Exception as e:
            logger.warning(f"Failed to launch rosbridge websocket: {e}")
            self._rosbridge_process = None

    def _launch_visualizer_server(self):
        """Launch the local ROS2 topic visualizer's Flask app (static page + sensor-config API)."""
        try:
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "visualizer_server.py")
            self._visualizer_process = subprocess.Popen([
                sys.executable, script,
                "--port", str(self.visualizer_port),
                "--rosbridge-port", str(self.rosbridge_port),
            ], start_new_session=True)
            logger.info(f"Launched ROS2 topic visualizer on port {self.visualizer_port} "
                        f"(pid={self._visualizer_process.pid})")
        except Exception as e:
            logger.warning(f"Failed to launch ROS2 topic visualizer: {e}")
            self._visualizer_process = None

    def _reset_teleop_state_file(self):
        """
        Clear the shared teleop_config.json state file at the start of a
        fresh session, before any vessel writes into it - same rationale and
        pattern as _reset_visualizer_state_file() above (a vessel from a
        previous, already-stopped session must not leak into this session's
        teleop vessel list).
        """
        try:
            state_file = os.path.join("/tmp/mavsim_bridge_state", "teleop_config.json")
            if os.path.exists(state_file):
                os.remove(state_file)
        except OSError as e:
            logger.debug(f"Failed to reset teleop state file: {e}")

    def _launch_teleop(self):
        """
        Launch the keyboard teleop node (plans/plan_teleop.md): an rclpy
        process that publishes interfaces/Actuator commands on
        /<vessel>/actuator_cmd from browser keypresses, and serves its own
        page + WebSocket. Launched once per session, the same way as
        rosbridge/the visualizer above.
        """
        try:
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "teleop_node.py")
            self._teleop_process = subprocess.Popen([
                sys.executable, script,
                "--http-port", str(self.teleop_http_port),
                "--ws-port", str(self.teleop_ws_port),
            ], start_new_session=True)
            logger.info(f"Launched keyboard teleop on port {self.teleop_http_port} "
                        f"(ws {self.teleop_ws_port}, pid={self._teleop_process.pid})")
        except Exception as e:
            logger.warning(f"Failed to launch keyboard teleop: {e}")
            self._teleop_process = None

    def _write_teleop_config(self, vessel_name: str, ctrl: 'MavsimController'):
        """
        Write this vessel's thruster/control-surface geometry to the shared
        teleop_config.json state file the teleop node reads at startup to
        build its per-vessel ThrustAllocator. Merge-write pattern like
        _fetch_and_cache_sensor_config() above, but simpler - no HTTP fetch
        needed, since ctrl.thrusters/ctrl.control_surfaces are already
        in-process by the time this is called.
        """
        try:
            import json

            state_dir = "/tmp/mavsim_bridge_state"
            os.makedirs(state_dir, exist_ok=True)
            state_file = os.path.join(state_dir, "teleop_config.json")

            existing = {}
            if os.path.exists(state_file):
                try:
                    with open(state_file, "r") as f:
                        existing = json.load(f)
                except (json.JSONDecodeError, OSError):
                    existing = {}

            existing[vessel_name] = {
                "thrusters": self._extract_thrusters(ctrl),
                "control_surfaces": self._extract_control_surfaces(ctrl),
            }
            with open(state_file, "w") as f:
                json.dump(existing, f)

            logger.info(f"Wrote teleop config for {vessel_name}")
        except Exception as e:
            logger.warning(f"Failed to write teleop config for {vessel_name}: {e}")

    def _fetch_and_cache_sensor_config(self, session_id: str, api_token: str, vessel_name: str):
        """
        Fetch this vessel's sensor extrinsics/intrinsics (mounting location/
        orientation, camera fov/resolution) via the same token-authenticated
        endpoint the headless observer's frontend session uses, and cache them
        to a shared state file the visualizer's Flask app reads - the same
        shared-/tmp-directory pattern already used for camera preview frames
        (CAMERA_FRAME_DIR in bridge_controller.py / bridge_webapp.py). Best
        effort: the visualizer's time-history/camera/point-cloud/overlay views
        simply show nothing for this vessel until this succeeds.
        """
        try:
            import json
            import requests

            state_dir = "/tmp/mavsim_bridge_state"
            os.makedirs(state_dir, exist_ok=True)
            state_file = os.path.join(state_dir, "sensor_config.json")

            response = requests.get(
                f"{self.backend_url}/api/simulation/observer/status",
                params={"session_id": session_id, "api_token": api_token},
                timeout=5.0,
            )
            response.raise_for_status()
            full_config = response.json().get("fullConfig", {})

            # Agents don't carry their own ROS name - it's constructed the same
            # way python_controller.py builds full_vessel_name: f"{name}_{vessel_id}"
            # (e.g. name="sookshma", vessel_id="01" -> "sookshma_01"). Fall back to
            # a bare name match, then to the only agent, for configs with unusual
            # vessel_id formatting.
            agents = full_config.get("agents", [])
            sensors = []
            matched = next(
                (a for a in agents if f"{a.get('name')}_{a.get('vessel_id')}" == vessel_name),
                next((a for a in agents if a.get("name") == vessel_name), None)
                or (agents[0] if len(agents) == 1 else None),
            )
            if matched is not None:
                sensors = matched.get("sensors") or []

            # Camera sensors carry fov/resolution nested under "camera_config"
            # (the top-level fov/resolution keys are always present but null
            # for cameras) rather than as top-level fields - flatten so the
            # visualizer's JS only ever has to read sensor.fov / sensor.resolution.
            for sensor in sensors:
                cam_cfg = sensor.get("camera_config")
                if cam_cfg:
                    if sensor.get("fov") is None:
                        sensor["fov"] = cam_cfg.get("fov")
                    if sensor.get("resolution") is None:
                        sensor["resolution"] = cam_cfg.get("resolution")

            existing = {}
            if os.path.exists(state_file):
                try:
                    with open(state_file, "r") as f:
                        existing = json.load(f)
                except (json.JSONDecodeError, OSError):
                    existing = {}
            existing[vessel_name] = {"sensors": sensors}
            with open(state_file, "w") as f:
                json.dump(existing, f)

            logger.info(f"Cached sensor config for {vessel_name} ({len(sensors)} sensors) "
                        f"for the ROS2 topic visualizer")
        except Exception as e:
            logger.warning(f"Failed to fetch/cache sensor config for {vessel_name}: {e}")

    def close(self):
        """Close connections and cleanup."""
        logger.info("Closing controller...")

        self._running = False

        # Stop the observer/rosbridge/visualizer subprocesses, if running.
        # Guarded so close() being called more than once (both the
        # SIGINT/SIGTERM handler and run()'s finally block can call it) is a
        # safe no-op the second time.
        #
        # Signal all of them first, THEN wait - not terminate-then-wait one
        # at a time. Sequentially waiting up to 5s per subprocess (up to 15s
        # total for 3) exceeds Docker's default 10s stop grace period, so a
        # plain `docker stop`/container removal would SIGKILL the container
        # before all three finished shutting down, leaving the survivors
        # (confirmed empirically: orphaned `ros2 launch rosbridge_server`
        # processes surviving container removal and squatting on the
        # rosbridge port for days) as host-level orphans outside this
        # container's process tree.
        #
        # Signal/kill the whole process GROUP (os.killpg), not just the
        # direct child PID (proc.terminate()/proc.kill()). `ros2 launch`
        # doesn't run rosbridge itself - it spawns rosbridge_websocket,
        # rosapi and rosapi_params as its OWN children. If it's still mid-
        # cascade on its own graceful shutdown when our 5s timeout expires,
        # a plain proc.kill() SIGKILLs only the `ros2 launch` process itself
        # (unblockable, instant) with no chance to forward anything further -
        # its children get reparented to init and keep running forever as
        # orphans. All three subprocesses are launched with
        # start_new_session=True specifically so they (and everything they
        # spawn) share one killable process group instead.
        to_stop = [
            ("_observer_process", "headless sensor observer"),
            ("_rosbridge_process", "rosbridge websocket"),
            ("_visualizer_process", "ROS2 topic visualizer"),
            ("_teleop_process", "keyboard teleop node"),
        ]
        procs = [(attr, label, getattr(self, attr)) for attr, label in to_stop]
        for attr, label, proc in procs:
            if proc is not None:
                logger.info(f"Stopping {label}...")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError) as e:
                    logger.debug(f"Error signaling {label} process group: {e}")
        for attr, label, proc in procs:
            if proc is not None:
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except (ProcessLookupError, PermissionError) as e:
                        logger.debug(f"Error killing {label} process group: {e}")
                    try:
                        proc.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        logger.warning(f"{label} process group did not exit even after SIGKILL")
                except Exception as e:
                    logger.debug(f"Error stopping {label}: {e}")
                setattr(self, attr, None)

        # Zero all actuators before disconnecting so the vessel doesn't
        # keep running with the last commanded thrust/rudder.
        self._zero_all_actuators()
        
        # Stop sensor generators
        for vname, gen in self._sensor_generators.items():
            try:
                gen.stop()
            except Exception as e:
                logger.debug(f"Error stopping sensor generator for {vname}: {e}")
        self._sensor_generators.clear()
        
        # Clear republishers and destroy local ROS2 node
        self._ros2_republishers.clear()
        self._ros2_sensor_pubs.clear()
        if self._ros2_node is not None:
            try:
                self._ros2_node.destroy_node()
            except Exception:
                pass
            self._ros2_node = None
        
        # Stop polling
        self._stop_polling()
        
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        
        if self._recording:
            self.stop_recording()
        
        # Close all vessel controllers
        for vname, ctrl in self._controllers.items():
            try:
                ctrl.close()
            except Exception as e:
                logger.warning(f"Error closing controller for {vname}: {e}")
        self._controllers.clear()
        
        # Fallback for single-vessel mode if not in _controllers
        if self._controller and self._controller not in self._controllers.values():
            self._controller.close()
        
        logger.info("Controller closed")
    
    def run(self, rate_hz: float = 10.0):
        """
        Run controller (connect, start control loop, wait for shutdown).
        
        Args:
            rate_hz: Control loop frequency in Hz
        """
        # Setup signal handlers
        def signal_handler(sig, frame):
            logger.info("Received shutdown signal")
            self.close()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Connect
        if not self.connect():
            logger.error("Failed to connect - exiting")
            sys.exit(1)
        
        # Start control loop
        self.start_control_loop(rate_hz=rate_hz)
        
        # Wait for shutdown
        try:
            while self._running:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()


def create_controller_from_args() -> BaseController:
    """
    Create controller from command-line arguments.
    
    Returns:
        BaseController instance
    """
    parser = argparse.ArgumentParser(description='mavsim Base Controller')
    parser.add_argument('--code', required=True, help='Controller code from simulation UI')
    parser.add_argument('--frontend-url', default='http://localhost:5173',
                       help='Where the MAVSim frontend is hosted (default: http://localhost:5173) - '
                            'override with the real address whenever the bridge isn\'t on the same '
                            'machine as the frontend')
    parser.add_argument('--backend-url', default='http://localhost:5000',
                       help='Backend URL (default: http://localhost:5000)')
    parser.add_argument('--vessel-name', help='Vessel name (auto-assigned if not specified)')
    parser.add_argument('--camera-port', type=int, default=None,
                       help='Camera sensor port override (default: auto per vessel)')
    parser.add_argument('--lidar-port', type=int, default=None,
                       help='Lidar sensor port override (default: auto per vessel)')
    parser.add_argument('--sensor-base-port', type=int, default=SENSOR_BRIDGE_BASE_PORT,
                       help='Base port for auto sensor bridge ports (default: 7000)')
    parser.add_argument('--rate', type=float, default=10.0,
                       help='Control loop rate in Hz (default: 10.0)')
    parser.add_argument('--rosbridge-port', type=int, default=9090,
                       help='Local rosbridge websocket port for the ROS2 topic visualizer (default: 9090)')
    parser.add_argument('--visualizer-port', type=int, default=8899,
                       help='Local ROS2 topic visualizer port (default: 8899)')
    parser.add_argument('--teleop-http-port', type=int, default=8900,
                       help='Local keyboard teleop page port (default: 8900)')
    parser.add_argument('--teleop-ws-port', type=int, default=8901,
                       help='Local keyboard teleop key/telemetry WebSocket port (default: 8901)')

    args = parser.parse_args()

    # This will be overridden by client code
    # Client code should import this and create their own controller
    controller = BaseController(
        controller_code=args.code,
        frontend_url=args.frontend_url,
        backend_url=args.backend_url,
        vessel_name=args.vessel_name,
        camera_port=args.camera_port,
        lidar_port=args.lidar_port,
        sensor_base_port=args.sensor_base_port,
        rosbridge_port=args.rosbridge_port,
        visualizer_port=args.visualizer_port,
        teleop_http_port=args.teleop_http_port,
        teleop_ws_port=args.teleop_ws_port,
    )

    return controller
