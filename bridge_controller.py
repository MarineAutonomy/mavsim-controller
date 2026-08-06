#!/usr/bin/env python3
"""
mavsim Bridge Controller
========================

This file is NOT meant to be edited. It turns the mavsim controller container
into a ROS2 bridge so you can write your own controller in any language.

What it does
------------
1. Handshakes with the backend and connects to the simulation (handled by the
   BaseController framework that ships inside the Docker image).
2. Republishes telemetry for every vessel you own onto LOCAL ROS2 topics:
       /<vessel>/vessel_state        std_msgs/Float64MultiArray
       /<vessel>/vessel_state_der    std_msgs/Float64MultiArray
       /<vessel>/odometry_sim        nav_msgs/Odometry
   plus camera/lidar sensor topics (always enabled, streamed via a headless
   observer browser - see plans/plan_headless_observer.md).
3. Subscribes to a LOCAL actuator command topic for every vessel you own:
       /<vessel>/actuator_cmd        interfaces/Actuator
   The latest message received is stored (and overwritten by newer ones).
4. On every control-loop tick (the --rate timer) it forwards the latest stored
   command for each owned vessel to the simulator via the REST control API.

Your own controller code (any language) just needs to:
  - subscribe to the telemetry topics above, and
  - publish interfaces/Actuator messages on /<vessel>/actuator_cmd.

Safety / ownership
------------------
The bridge ONLY exposes an actuator_cmd subscription for vessels it actually
owns (bound via the controller code or token). Vessels controlled from other
machines are never given a local command topic, so you cannot accidentally
command them.

Configuration (set by start.sh / start.bat via environment variables)
---------------------------------------------------------------------
  MAVSIM_OBSERVE_OTHERS   "1" to also publish READ-ONLY odometry + actuator
                          state for vessels you do NOT own (default: off).
  MAVSIM_CMD_TIMEOUT      seconds; stop forwarding a command if no fresh one
                          has arrived within this window (default: 1.0;
                          <= 0 disables the timeout and always resends the
                          last command).
  CAMERA_FRAME_DIR        where camera JPEGs are written for the web viewer
                          (default: /tmp/mavsim_camera_frames).
"""

import logging
import os
import threading
import time
from pathlib import Path

from base_controller import BaseController

logger = logging.getLogger("bridge_controller")

# Where camera frames are written so the web viewer / camera_viewer.html can
# display them (only used when sensors are enabled).
FRAME_DIR = Path(os.environ.get("CAMERA_FRAME_DIR", "/tmp/mavsim_camera_frames"))

# Vessel state layout (after the leading timestamp has been stripped by the
# rosbridge handler):
#   [u, v, w, p, q, r, x, y, z, <attitude>, <actuators...>]
# attitude is 3 values (euler) or 4 (quaternion), so actuators start here:
_ATTITUDE_EULER = 3
_ATTITUDE_QUAT = 4
_KINEMATIC_LEN = 9  # u,v,w,p,q,r,x,y,z


def _env_flag(name: str, default: bool = False) -> bool:
    val = os.environ.get(name)
    if val is None:
        return default
    return val.strip().lower() in ("1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


class MyController(BaseController):
    """Bridge controller: exposes the simulation as local ROS2 topics.

    Named ``MyController`` so the container's run_controller.py discovers it
    automatically (it loads a BaseController subclass from
    /app/user_code/my_controller.py).
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._observe_others = _env_flag("MAVSIM_OBSERVE_OTHERS", False)
        self._cmd_timeout = _env_float("MAVSIM_CMD_TIMEOUT", 1.0)

        # Latest command per owned vessel: ros_name -> {names, values, t}
        self._latest_cmd = {}
        self._cmd_lock = threading.Lock()

        # ROS2 plumbing for the command subscriptions (serviced by a dedicated
        # spin thread). Telemetry/observe publishers live on the BaseController
        # republisher node and are published from the control-loop thread.
        self._bridge_node = None
        self._cmd_subs = {}
        self._observe_pubs = {}
        self._spin_executor = None
        self._spin_thread = None

        # Write camera frames to disk for the web viewer. Sensors are always
        # enabled now (plans/plan_headless_observer.md) - no more opt-in flag.
        try:
            FRAME_DIR.mkdir(parents=True, exist_ok=True)
            for vid in range(10):
                for cid in range(10):
                    self.on_camera(vessel_id=vid, camera_id=cid)(self._on_camera_frame)
        except Exception as exc:  # pragma: no cover - best effort
            logger.debug("Could not set up camera frame writing: %s", exc)

        logger.info(
            "Bridge controller initialized (observe_others=%s, cmd_timeout=%.2fs)",
            self._observe_others,
            self._cmd_timeout,
        )

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        ok = super().connect()
        if ok:
            try:
                self._setup_bridge()
            except Exception as exc:
                logger.error("Failed to set up ROS2 bridge interface: %s", exc, exc_info=True)
        return ok

    def _setup_bridge(self):
        """Create command subscriptions (owned vessels only) and a spin thread."""
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        Actuator = self._import_actuator()
        if Actuator is None:
            logger.error(
                "interfaces/Actuator is not available in this image; the bridge "
                "cannot expose actuator_cmd topics."
            )
            return

        if not rclpy.ok():
            rclpy.init()

        self._bridge_node = rclpy.create_node("mavsim_bridge")

        # Subscribe to /<vessel>/actuator_cmd for OWNED vessels only.
        for vname in self._controlled_vessels:
            topic = f"/{vname}/actuator_cmd"
            sub = self._bridge_node.create_subscription(
                Actuator, topic, self._make_cmd_callback(vname), 10
            )
            self._cmd_subs[vname] = sub
            logger.info("Bridge listening for commands on %s (owned vessel)", topic)

        if self._observe_others:
            self._setup_observe_publishers()

        # Spin the bridge node so subscription callbacks fire.
        self._spin_executor = SingleThreadedExecutor()
        self._spin_executor.add_node(self._bridge_node)
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        logger.info("Bridge spin thread started")

    def _spin_loop(self):
        try:
            self._spin_executor.spin()
        except Exception as exc:  # pragma: no cover - shutdown races
            logger.debug("Bridge spin loop ended: %s", exc)

    # ------------------------------------------------------------------
    # Command intake (user -> bridge)
    # ------------------------------------------------------------------

    def _make_cmd_callback(self, vname: str):
        def _callback(msg):
            # Defensive: never accept commands for a vessel we do not own.
            if vname not in self._controlled_vessels:
                logger.warning("Dropping actuator_cmd for non-owned vessel %s", vname)
                return

            names = [str(n) for n in msg.actuator_names]
            values = [float(v) for v in msg.actuator_values]

            if not names:
                logger.warning(
                    "actuator_cmd for %s has no actuator_names; ignoring "
                    "(names are required so commands map to the right actuators)",
                    vname,
                )
                return
            if len(names) != len(values):
                logger.warning(
                    "actuator_cmd for %s: actuator_names (%d) and actuator_values "
                    "(%d) length mismatch; ignoring",
                    vname,
                    len(names),
                    len(values),
                )
                return

            with self._cmd_lock:
                self._latest_cmd[vname] = {
                    "names": names,
                    "values": values,
                    "t": time.monotonic(),
                }

        return _callback

    # ------------------------------------------------------------------
    # Command forwarding (bridge -> simulator via REST)
    # ------------------------------------------------------------------

    def control_loop(self, vessel_states):
        """Return the latest stored command per owned vessel.

        The BaseController control loop calls this on every --rate tick and
        forwards the returned commands to the simulator over REST. We also use
        this tick to publish read-only telemetry for observed vessels.
        """
        if self._observe_others and self._observe_pubs:
            try:
                self._publish_observed()
            except Exception as exc:  # pragma: no cover - best effort
                logger.debug("observe-others publish error: %s", exc)

        now = time.monotonic()
        commands = {}
        with self._cmd_lock:
            for vname in self._controlled_vessels:
                entry = self._latest_cmd.get(vname)
                if not entry:
                    continue
                if self._cmd_timeout > 0 and (now - entry["t"]) > self._cmd_timeout:
                    # Stale: stop forwarding so the simulator's watchdog can
                    # safely zero the actuators.
                    continue
                commands[vname] = dict(zip(entry["names"], entry["values"]))
        # Nested dict {vessel: {actuator: value}} -> per-vessel REST dispatch.
        return commands

    # ------------------------------------------------------------------
    # observe-others: read-only telemetry for vessels we do not own
    # ------------------------------------------------------------------

    def _setup_observe_publishers(self):
        if self._ros2_node is None:
            self._init_ros2_sensor_node()
        if self._ros2_node is None:
            logger.warning("No local ROS2 node available; observe-others disabled")
            return

        from nav_msgs.msg import Odometry

        Actuator = self._import_actuator()
        owned = set(self._controlled_vessels)
        vessels = self._controller.vessels if self._controller else []

        for vessel in vessels:
            vname = vessel.get("ros_name") if isinstance(vessel, dict) else None
            if not vname or vname in owned:
                continue
            pubs = {
                "odom": self._ros2_node.create_publisher(
                    Odometry, f"/{vname}/odometry_sim", 10
                )
            }
            if Actuator is not None:
                pubs["act"] = self._ros2_node.create_publisher(
                    Actuator, f"/{vname}/actuator_state", 10
                )
            self._observe_pubs[vname] = pubs
            logger.info(
                "observe-others: publishing read-only /%s/odometry_sim and "
                "/%s/actuator_state",
                vname,
                vname,
            )

    def _publish_observed(self):
        if self._controller is None:
            return
        use_quat = self._controller.use_quaternion
        for vname, pubs in self._observe_pubs.items():
            raw = self._controller.get_raw_state(vname)
            if raw is None:
                continue
            try:
                self._publish_local_odometry(pubs["odom"], raw, use_quat)
            except Exception as exc:  # pragma: no cover
                logger.debug("observed odometry error for %s: %s", vname, exc)
            if "act" in pubs:
                self._publish_observed_actuators(pubs["act"], raw, use_quat)

    def _publish_observed_actuators(self, pub, raw, use_quat: bool):
        """Publish the simulator-reported actuator values for an observed vessel.

        These are the actuator states sliced from vessel_state (post-application),
        NOT the raw command another agent issued. Names are unknown for vessels
        we do not own, so actuator_names is left empty.
        """
        Actuator = self._import_actuator()
        if Actuator is None:
            return
        start = _KINEMATIC_LEN + (_ATTITUDE_QUAT if use_quat else _ATTITUDE_EULER)
        if len(raw) <= start:
            return
        values = [float(x) for x in raw[start:]]
        msg = Actuator()
        msg.actuator_values = values
        msg.actuator_names = []
        msg.covariance = [0.0] * len(values)
        pub.publish(msg)

    # ------------------------------------------------------------------
    # Camera frame writing (for the web viewer / camera_viewer.html)
    # ------------------------------------------------------------------

    def _on_camera_frame(self, vessel_id, camera_id, timestamp, jpeg_data):
        try:
            frame_path = FRAME_DIR / f"{vessel_id}_{camera_id}.jpg"
            tmp_path = frame_path.with_suffix(".tmp")
            tmp_path.write_bytes(jpeg_data)
            tmp_path.rename(frame_path)
        except OSError:
            pass

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self):
        try:
            if self._spin_executor is not None:
                self._spin_executor.shutdown()
        except Exception:
            pass
        try:
            if self._bridge_node is not None:
                self._bridge_node.destroy_node()
                self._bridge_node = None
        except Exception:
            pass
        super().close()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _import_actuator():
        try:
            from interfaces.msg import Actuator
            return Actuator
        except ImportError:
            return None
