#!/usr/bin/env python3
"""
Example Python Controller for mavsim Simulation Platform

This script demonstrates how to:
1. Send actuator commands to a running simulation via REST API
2. Subscribe to vessel odometry via WebSocket (rosbridge)
3. Implement a simple proportional controller for heading control
4. Access vessel configuration automatically provided by backend during handshake

Usage:
    # Handshake mode (recommended - automatically gets config from backend)
    python python_controller.py --backend-url http://localhost:5000 \
                               --code ABC123
    
    # Direct mode (legacy - requires explicit credentials, no config)
    python python_controller.py --backend-url http://localhost:5000 \
                               --session-id YOUR_SESSION_ID \
                               --api-token YOUR_API_TOKEN \
                               --rosbridge-url ws://localhost:9090

Requirements:
    pip install requests roslibpy

Note:
    The vessel configuration (including actuator IDs, geometry, etc.) is automatically
    fetched by the backend from S3 and provided in the handshake response. You do NOT
    need to use boto3 or access S3 directly - the backend handles all S3 access.

Author: mavsim Team
License: MIT
"""

import argparse
import asyncio
import logging
import math
import re
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import requests

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class Vector3:
    """3D vector for position and velocity."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    """Quaternion for orientation."""
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def to_euler(self) -> Tuple[float, float, float]:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        
        Returns:
            Tuple of (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


@dataclass
class Odometry:
    """Vessel odometry data."""
    position: Vector3 = field(default_factory=Vector3)
    orientation: Quaternion = field(default_factory=Quaternion)
    linear_velocity: Vector3 = field(default_factory=Vector3)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    timestamp: float = 0.0
    
    @classmethod
    def from_ros_message(cls, msg: dict) -> 'Odometry':
        """
        Create Odometry from ROS nav_msgs/Odometry message.
        
        Args:
            msg: ROS message dictionary from roslib
            
        Returns:
            Odometry object
        """
        odom = cls()
        
        # Extract position
        pose = msg.get('pose', {}).get('pose', {})
        pos = pose.get('position', {})
        odom.position = Vector3(
            x=pos.get('x', 0.0),
            y=pos.get('y', 0.0),
            z=pos.get('z', 0.0)
        )
        
        # Extract orientation
        orient = pose.get('orientation', {})
        odom.orientation = Quaternion(
            w=orient.get('w', 1.0),
            x=orient.get('x', 0.0),
            y=orient.get('y', 0.0),
            z=orient.get('z', 0.0)
        )
        
        # Extract linear velocity
        twist = msg.get('twist', {}).get('twist', {})
        linear = twist.get('linear', {})
        odom.linear_velocity = Vector3(
            x=linear.get('x', 0.0),
            y=linear.get('y', 0.0),
            z=linear.get('z', 0.0)
        )
        
        # Extract angular velocity
        angular = twist.get('angular', {})
        odom.angular_velocity = Vector3(
            x=angular.get('x', 0.0),
            y=angular.get('y', 0.0),
            z=angular.get('z', 0.0)
        )
        
        # Extract timestamp
        header = msg.get('header', {})
        stamp = header.get('stamp', {})
        sec = stamp.get('sec', stamp.get('secs', 0))
        nsec = stamp.get('nanosec', stamp.get('nsecs', 0))
        odom.timestamp = sec + nsec * 1e-9
        
        return odom


@dataclass
class VesselState:
    """Full vessel state parsed from the simulator's Float64MultiArray topic.

    The state vector layout is:
        [time, u, v, w, p, q, r, x, y, z, {attitude}, actuators_sorted_by_id...]
    where {attitude} is [phi, theta, psi] (euler) or [q0, q1, q2, q3] (quaternion).
    """
    timestamp: float = 0.0
    position: Vector3 = field(default_factory=Vector3)
    orientation: Quaternion = field(default_factory=Quaternion)
    euler_angles: Vector3 = field(default_factory=Vector3)
    linear_velocity: Vector3 = field(default_factory=Vector3)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    actuator_states: List[float] = field(default_factory=list)

    @staticmethod
    def detect_quaternion(data: list) -> bool:
        """Heuristic: if data[10:14] form a unit quaternion, assume quaternion mode."""
        if len(data) < 14:
            return False
        norm_sq = sum(v * v for v in data[10:14])
        return abs(norm_sq - 1.0) < 0.05

    @classmethod
    def from_float_array(cls, data: list, use_quaternion: bool = False) -> 'VesselState':
        """Parse a Float64MultiArray.data list into a VesselState.

        Args:
            data: The flat list of floats from the ROS message.
            use_quaternion: Whether the attitude block is 4-element quaternion
                            (True) or 3-element euler (False, default).
        """
        vs = cls()
        if len(data) < 10:
            return vs

        vs.timestamp = data[0]
        vs.linear_velocity = Vector3(x=data[1], y=data[2], z=data[3])
        vs.angular_velocity = Vector3(x=data[4], y=data[5], z=data[6])
        vs.position = Vector3(x=data[7], y=data[8], z=data[9])

        if use_quaternion:
            if len(data) < 14:
                return vs
            q0, q1, q2, q3 = data[10], data[11], data[12], data[13]
            vs.orientation = Quaternion(w=q0, x=q1, y=q2, z=q3)
            roll, pitch, yaw = vs.orientation.to_euler()
            vs.euler_angles = Vector3(x=roll, y=pitch, z=yaw)
            actuator_start = 14
        else:
            if len(data) < 13:
                return vs
            phi, theta, psi = data[10], data[11], data[12]
            vs.euler_angles = Vector3(x=phi, y=theta, z=psi)
            # Convert euler to quaternion so orientation is always populated
            cr, sr = math.cos(phi / 2), math.sin(phi / 2)
            cp, sp = math.cos(theta / 2), math.sin(theta / 2)
            cy, sy = math.cos(psi / 2), math.sin(psi / 2)
            vs.orientation = Quaternion(
                w=cr * cp * cy + sr * sp * sy,
                x=sr * cp * cy - cr * sp * sy,
                y=cr * sp * cy + sr * cp * sy,
                z=cr * cp * sy - sr * sp * cy,
            )
            actuator_start = 13

        vs.actuator_states = list(data[actuator_start:])
        return vs


# =============================================================================
# REST API Client
# =============================================================================

class MavsimAPIClient:
    """
    REST API client for mavsim control endpoint.
    
    This client sends actuator commands to running simulations
    using the per-session API token for authentication.
    
    Supports two connection modes:
    1. Direct mode: Provide session_id, api_token, etc. directly
    2. Handshake mode: Use controller code to discover session details
    """
    
    def __init__(
        self,
        backend_url: str,
        session_id: str = None,
        api_token: str = None,
        vessel_name: str = 'vessel_01',
        timeout: float = 10.0
    ):
        """
        Initialize API client.
        
        Args:
            backend_url: Backend URL (e.g., http://localhost:5000)
            session_id: Session UUID (optional if using handshake)
            api_token: Per-session API token (optional if using handshake)
            vessel_name: Vessel name (default: vessel_01)
            timeout: Request timeout in seconds
        """
        self.backend_url = backend_url.rstrip('/')
        self.session_id = session_id
        self.api_token = api_token
        self.vessel_name = vessel_name
        self.timeout = timeout
        
        self._control_url = f"{self.backend_url}/api/control"
        self._handshake_url = f"{self.backend_url}/api/control/handshake"
        logger.info(f"API client initialized: {self._control_url}")
    
    @classmethod
    def from_handshake(
        cls,
        backend_url: str,
        controller_code: str,
        vessel_name: str = None,
        controller_id: str = None,
        timeout: float = 60.0
    ) -> Tuple['MavsimAPIClient', Dict]:
        """
        Create API client using handshake with controller code.
        
        This is the recommended way to connect when you have a controller code
        from the simulation UI.
        
        Args:
            backend_url: Backend URL (e.g., http://localhost:5000)
            controller_code: Short controller code from UI
            vessel_name: Vessel name (optional, auto-assigned for single-vessel sessions)
            controller_id: Optional controller identifier for logging
            timeout: Request timeout in seconds
            
        Returns:
            Tuple of (MavsimAPIClient instance, handshake response dict)
            
        Raises:
            requests.RequestException: If handshake fails
        """
        handshake_url = f"{backend_url.rstrip('/')}/api/control/handshake"
        
        payload = {
            'controllerCode': controller_code
        }
        if vessel_name:
            payload['vesselName'] = vessel_name
        if controller_id:
            payload['controllerId'] = controller_id
        
        logger.info(f"Performing handshake with controller code: {controller_code}")
        response = requests.post(
            handshake_url,
            json=payload,
            headers={'Content-Type': 'application/json'},
            timeout=timeout
        )
        if response.status_code != 200:
            try:
                error_body = response.json()
                error_msg = error_body.get('message') or error_body.get('error') or response.text
            except Exception:
                error_msg = response.text
            logger.error(f"Handshake failed ({response.status_code}): {error_msg}")
        response.raise_for_status()
        
        handshake_data = response.json()
        session_id = handshake_data.get('sessionId')
        api_token = handshake_data.get('apiToken')
        assigned_vessel = handshake_data.get('vesselName')
        status = handshake_data.get('status')
        
        if not session_id or not api_token:
            raise ValueError(
                f"Handshake response missing required fields: "
                f"sessionId={session_id}, apiToken={'***' if api_token else None}"
            )
        
        logger.info(
            f"Handshake successful: sessionId={session_id}, "
            f"vesselName={assigned_vessel}, status={status}"
        )
        
        client = cls(
            backend_url=backend_url,
            session_id=session_id,
            api_token=api_token,
            vessel_name=assigned_vessel or vessel_name or 'vessel_01',
            timeout=timeout
        )
        
        return client, handshake_data
    
    def send_command(
        self,
        actuator_names: List[str],
        actuator_values: List[float]
    ) -> bool:
        """
        Send actuator command to simulation.
        
        Args:
            actuator_names: List of actuator names (e.g., ['cs_01', 'th_01'])
                           Format: 'cs_XX' for control surfaces, 'th_XX' for thrusters
                           where XX is zero-padded actuator_id (e.g., 'cs_01', 'th_02')
            actuator_values: List of actuator values (e.g., [10.0, 1500.0])
                           Control surfaces: angle in degrees (-180 to 180)
                           Thrusters: RPM (0 to 10000)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if len(actuator_names) != len(actuator_values):
            logger.error("Actuator names and values must have same length")
            return False
        
        payload = {
            'sessionId': self.session_id,
            'apiToken': self.api_token,
            'vesselName': self.vessel_name,  # This should be full_vessel_name (e.g., "matsya_00")
            'actuatorNames': actuator_names,
            'actuatorValues': actuator_values
        }
        
        logger.debug(f"Sending control command: vesselName={self.vessel_name}, actuators={actuator_names}")
        
        try:
            response = requests.post(
                self._control_url,
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                return True
            else:
                error_data = response.json()
                logger.warning(
                    f"Control command failed: {response.status_code} - "
                    f"{error_data.get('error', 'Unknown error')}"
                )
                return False
                
        except requests.exceptions.Timeout:
            logger.error("Control command timed out")
            return False
        except requests.exceptions.RequestException as e:
            logger.error(f"Control command failed: {e}")
            return False
    
    def report_recording_status(
        self,
        status: str,
        topics: Optional[List[str]] = None,
        error: Optional[str] = None
    ) -> bool:
        """
        Report recording status to backend.
        
        Args:
            status: Recording status ("recording", "stopped", "error")
            topics: List of topics being recorded (optional)
            error: Error message if status is "error" (optional)
            
        Returns:
            True if status reported successfully
        """
        if not self.session_id or not self.api_token:
            logger.warning("Cannot report recording status: not authenticated")
            return False
        
        url = f"{self.backend_url}/api/simulation/recording/status"
        
        payload = {
            'sessionId': self.session_id,
            'apiToken': self.api_token,
            'status': status
        }
        
        if error:
            payload['error'] = error
        
        logger.debug(f"Reporting recording status: {status}")
        
        try:
            response = requests.post(
                url,
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                return True
            else:
                error_data = response.json()
                logger.warning(
                    f"Recording status report failed: {response.status_code} - "
                    f"{error_data.get('error', 'Unknown error')}"
                )
                return False
                
        except requests.exceptions.Timeout:
            logger.error("Recording status report timed out")
            return False
        except requests.exceptions.RequestException as e:
            logger.error(f"Recording status report failed: {e}")
            return False
    
    def send_thruster_command(self, actuator_id: int, rpm: float) -> bool:
        """
        Send single thruster command.
        
        Args:
            actuator_id: Actuator ID from vessel configuration (must be unique across all actuators)
                        This is the actuator_id field from the thruster config, NOT a sequential index.
                        Example: If thruster has actuator_id=2 in config, use actuator_id=2 here.
                        The resulting name will be 'th_02' (zero-padded to 2 digits).
            rpm: Thruster RPM (0-10000)
            
        Returns:
            True if successful
            
        Note:
            - Actuator IDs must be unique across ALL actuators (both control surfaces and thrusters)
            - You cannot have both a control surface and thruster with the same actuator_id
            - The name format is 'th_XX' where XX is the zero-padded actuator_id (e.g., 'th_01', 'th_02')
            - Actuator IDs are automatically provided in vessel_config from handshake response
        """
        # Use zero-padded format to match ROS expectations: 'th_XX' where XX is actuator_id
        return self.send_command([f'th_{actuator_id:02d}'], [rpm])
    
    def send_control_surface_command(self, actuator_id: int, angle: float) -> bool:
        """
        Send single control surface command.
        
        Args:
            actuator_id: Actuator ID from vessel configuration (must be unique across all actuators)
                        This is the actuator_id field from the control surface config, NOT a sequential index.
                        Example: If control surface has actuator_id=1 in config, use actuator_id=1 here.
                        The resulting name will be 'cs_01' (zero-padded to 2 digits).
            angle: Deflection angle in degrees (-180 to 180)
            
        Returns:
            True if successful
            
        Note:
            - Actuator IDs must be unique across ALL actuators (both control surfaces and thrusters)
            - You cannot have both a control surface and thruster with the same actuator_id
            - The name format is 'cs_XX' where XX is the zero-padded actuator_id (e.g., 'cs_01', 'cs_02')
            - Actuator IDs are automatically provided in vessel_config from handshake response
        """
        # Use zero-padded format to match ROS expectations: 'cs_XX' where XX is actuator_id
        return self.send_command([f'cs_{actuator_id:02d}'], [angle])


# =============================================================================
# Roslib Subscriber
# =============================================================================

_sni_patch_applied = False


def _ensure_roslibpy_sends_sni():
    """
    Make roslibpy's wss:// connections send an SNI server_name extension.

    autobahn's connectWS() falls back to twisted's legacy
    ssl.ClientContextFactory() when no contextFactory is passed, and roslibpy
    never passes one. That legacy context sends no SNI, which is fatal against
    the hosted deployment: CloudFront is configured sni-only
    (infra/terraform/frontend.tf), so it answers a handshake with no
    server_name with a TLS alert 40 rather than a certificate. The symptom is
    silent - connect() below just retries until its 90s deadline and gives up,
    and the bridge comes up with no ROS2 topics at all.

    Patching the module-level connectWS symbol (rather than the factory) is
    what makes this take effect: roslibpy.Ros.__init__ calls self.connect()
    during construction, which queues the *bound* factory._connect through
    reactor.callFromThread before any caller gets the object back. Overriding
    factory._connect afterwards is therefore too late and silently does
    nothing, while _connect's own `connectWS(self)` is a module-global lookup
    resolved at call time - i.e. after this patch is in place.

    optionsForClientTLS also verifies the certificate chain against the
    system trust store, which ClientContextFactory did not do at all.
    """
    global _sni_patch_applied
    if _sni_patch_applied:
        return
    try:
        import roslibpy.comm.comm_autobahn as comm_autobahn
        from twisted.internet import ssl as twisted_ssl
    except ImportError:
        # Non-autobahn roslibpy backend (or roslibpy absent) - connect() below
        # reports the missing dependency on its own.
        return

    original_connect_ws = comm_autobahn.connectWS

    def connect_ws_with_sni(factory, contextFactory=None, *args, **kwargs):
        if contextFactory is None and getattr(factory, 'isSecure', False):
            contextFactory = twisted_ssl.optionsForClientTLS(factory.host)
        return original_connect_ws(factory, contextFactory, *args, **kwargs)

    comm_autobahn.connectWS = connect_ws_with_sni
    _sni_patch_applied = True


class RoslibSubscriber:
    """
    WebSocket subscriber for ROS topics via rosbridge.
    
    This subscriber connects to rosbridge and subscribes to
    vessel odometry topics using the roslibpy library.
    """
    
    def __init__(
        self,
        rosbridge_url: str,
        namespace: str = '',
        vessel_name: str = 'vessel_01'
    ):
        """
        Initialize roslib subscriber.
        
        Args:
            rosbridge_url: Rosbridge WebSocket URL (e.g., ws://localhost:9090)
            namespace: ROS namespace (e.g., 'sim_abc123')
            vessel_name: Vessel name (e.g., 'vessel_01')
        """
        self.rosbridge_url = rosbridge_url
        # Normalize namespace: remove all leading and trailing slashes (match frontend behavior)
        # Frontend uses: namespace.replace(/^\/+|\/+$/g, '')
        # Python equivalent: re.sub(r'^/+|/+$', '', namespace)
        self.namespace = re.sub(r'^/+|/+$', '', namespace) if namespace else ''
        self.vessel_name = vessel_name
        
        self._ros = None
        self._odometry_topic = None
        self._latest_odometry: Optional[Odometry] = None
        self._odometry_lock = threading.Lock()
        self._callbacks: List[Callable[[Odometry], None]] = []
        self._connected = False

        # Multi-vessel state storage
        self._vessel_states: Dict[str, VesselState] = {}
        self._vessel_state_lock = threading.Lock()
        self._vessel_state_topics: list = []
        self._use_quaternion: Optional[bool] = None

        # Raw state arrays for local sensor generation
        self._raw_vessel_states: Dict[str, np.ndarray] = {}
        self._raw_vessel_state_ders: Dict[str, np.ndarray] = {}
        # Elapsed sim time (leading element of the vessel_state message, stripped
        # from _raw_vessel_states above) -- needed by time-dependent client-side
        # sensors such as the wave probe, which don't derive their measurement
        # from vessel motion alone.
        self._raw_vessel_state_times: Dict[str, float] = {}
        self._vessel_state_der_topics: list = []
        self._state_der_callbacks: List[Callable] = []
        
        logger.info(f"Roslib subscriber initialized: {self.rosbridge_url}")
    
    def connect(self) -> bool:
        """
        Connect to rosbridge WebSocket.
        
        Returns:
            True if connected successfully
        """
        try:
            import roslibpy
        except ImportError:
            logger.error("roslibpy not installed. Install with: pip install roslibpy")
            return False

        # Must happen before roslibpy.Ros() below - see the function's docstring
        # on why constructing Ros already commits to a connection attempt.
        _ensure_roslibpy_sends_sni()

        # In external-controller "wait_mode=all", rosbridge may come up only after
        # the final controller handshakes. Keep retrying across that startup window.
        timeout = 90.0
        deadline = time.time() + timeout
        attempt = 0
        host, port, has_path = self._parse_url()
        while time.time() < deadline:
            attempt += 1
            try:
                self._ros = roslibpy.Ros(host=host, port=port)
                # roslibpy only builds ws://host:port URLs.  When connecting
                # through the WS proxy the URL includes a path and query
                # string (e.g. /ws/session-<id>?token=<tok>).  We must call
                # setSessionParameters so Autobahn re-parses *all* URL
                # components — especially `resource` (the HTTP upgrade path).
                # Simply setting factory.url alone leaves resource as "/".
                if has_path:
                    full_url = self.rosbridge_url
                    if not full_url.startswith(('ws://', 'wss://')):
                        full_url = f'ws://{full_url}'
                    self._ros.factory.setSessionParameters(url=full_url)
                self._ros.run()

                # Give connection state a brief moment to settle after run().
                settle_deadline = min(deadline, time.time() + 5.0)
                while not self._ros.is_connected and time.time() < settle_deadline:
                    time.sleep(0.2)

                if self._ros.is_connected:
                    self._connected = True
                    logger.info("Connected to rosbridge")
                    return True
            except Exception as e:
                remaining = max(0, int(deadline - time.time()))
                logger.info(
                    "Rosbridge not ready yet (attempt %s, %ss remaining): %s",
                    attempt,
                    remaining,
                    e,
                )
            finally:
                # Ensure failed attempts don't leave stale clients around.
                if self._ros and not self._ros.is_connected:
                    try:
                        self._ros.close()
                    except Exception:
                        pass
                    self._ros = None

            time.sleep(2.0)

        logger.error("Rosbridge connection timed out")
        return False
    
    def _parse_url(self):
        """Parse rosbridge URL into (host, port, has_path) using urllib."""
        from urllib.parse import urlparse
        url = self.rosbridge_url
        if not url.startswith(('ws://', 'wss://')):
            url = f'ws://{url}'
        parsed = urlparse(url)
        host = parsed.hostname or 'localhost'
        # A hosted wss:// URL carries no explicit port and is served on 443.
        # setSessionParameters() in connect() re-derives the port anyway
        # whenever has_path is set, but only then - defaulting every portless
        # URL to 9090 would otherwise build a factory pointing at a port
        # nothing listens on. ws:// keeps the 9090 default: that is rosbridge's
        # own port, and every local/direct URL in this codebase is ws://.
        if parsed.port:
            port = parsed.port
        elif parsed.scheme == 'wss':
            port = 443
        else:
            port = 9090
        has_path = bool(parsed.path and parsed.path not in ('', '/'))
        return host, port, has_path
    
    def subscribe_odometry(
        self,
        callback: Optional[Callable[[Odometry], None]] = None
    ) -> bool:
        """
        Subscribe to vessel odometry topic.
        
        Args:
            callback: Optional callback function called on each odometry message
            
        Returns:
            True if subscribed successfully
        """
        if not self._connected or self._ros is None:
            logger.error("Not connected to rosbridge")
            return False
        
        try:
            import roslibpy
            
            # Build topic name (ensure no double slashes)
            # Normalize namespace one more time to be safe (match frontend normalization)
            clean_namespace = re.sub(r'^/+|/+$', '', self.namespace) if self.namespace else ''
            clean_vessel_name = re.sub(r'^/+|/+$', '', self.vessel_name) if self.vessel_name else ''
            
            if clean_namespace:
                topic_name = f"/{clean_namespace}/{clean_vessel_name}/odometry_sim"
            else:
                topic_name = f"/{clean_vessel_name}/odometry_sim"
            
            logger.info(f"Subscribing to odometry topic: {topic_name}")
            logger.debug(f"  namespace (raw): '{self.namespace}' -> (cleaned): '{clean_namespace}'")
            logger.debug(f"  vessel_name (raw): '{self.vessel_name}' -> (cleaned): '{clean_vessel_name}'")
            
            self._odometry_topic = roslibpy.Topic(
                self._ros,
                topic_name,
                'nav_msgs/Odometry'
            )
            
            if callback:
                self._callbacks.append(callback)
            
            self._odometry_topic.subscribe(self._on_odometry_message)
            
            logger.info(f"Subscribed to odometry topic: {topic_name}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to subscribe to odometry: {e}")
            return False
    
    def _on_odometry_message(self, message: dict):
        """Handle incoming odometry message."""
        try:
            odometry = Odometry.from_ros_message(message)
            
            with self._odometry_lock:
                self._latest_odometry = odometry
            
            # Call registered callbacks
            for callback in self._callbacks:
                try:
                    callback(odometry)
                except Exception as e:
                    logger.warning(f"Odometry callback error: {e}")
                    
        except Exception as e:
            logger.warning(f"Error parsing odometry message: {e}")
    
    def get_odometry(self) -> Optional[Odometry]:
        """
        Get latest odometry data.
        
        Returns:
            Latest Odometry or None if not available
        """
        with self._odometry_lock:
            return self._latest_odometry
    
    def add_callback(self, callback: Callable[[Odometry], None]):
        """Add odometry callback."""
        self._callbacks.append(callback)

    # -----------------------------------------------------------------
    # Multi-vessel state subscription
    # -----------------------------------------------------------------

    def subscribe_vessel_states(
        self,
        vessel_ros_names: List[str],
        use_quaternion: Optional[bool] = None,
    ) -> bool:
        """Subscribe to vessel_state topics for all vessels in the session.

        Args:
            vessel_ros_names: List of ros_name strings (e.g. ['matsya_01', 'matsya_02']).
            use_quaternion: Explicit flag from config. If None, auto-detect on first message.

        Returns:
            True if at least one subscription succeeded.
        """
        if not self._connected or self._ros is None:
            logger.error("Not connected to rosbridge")
            return False

        self._use_quaternion = use_quaternion

        try:
            import roslibpy
        except ImportError:
            logger.error("roslibpy not installed. Install with: pip install roslibpy")
            return False

        clean_ns = re.sub(r'^/+|/+$', '', self.namespace) if self.namespace else ''
        ok_count = 0

        for ros_name in vessel_ros_names:
            clean_name = re.sub(r'^/+|/+$', '', ros_name) if ros_name else ''
            if not clean_name:
                continue
            if clean_ns:
                topic = f"/{clean_ns}/{clean_name}/vessel_state"
            else:
                topic = f"/{clean_name}/vessel_state"

            try:
                t = roslibpy.Topic(self._ros, topic, 'std_msgs/Float64MultiArray')
                # Capture ros_name in closure
                t.subscribe(lambda msg, _rn=clean_name: self._on_vessel_state(msg, _rn))
                self._vessel_state_topics.append(t)
                ok_count += 1
                logger.info(f"Subscribed to vessel state: {topic}")
            except Exception as e:
                logger.warning(f"Failed to subscribe to {topic}: {e}")

        return ok_count > 0

    def _on_vessel_state(self, message: dict, ros_name: str):
        """Handle incoming vessel_state Float64MultiArray message."""
        try:
            data = message.get('data', [])
            if not data:
                return

            # Auto-detect quaternion mode on first message if not set
            if self._use_quaternion is None:
                self._use_quaternion = VesselState.detect_quaternion(data)
                logger.info(
                    f"Auto-detected attitude mode: {'quaternion' if self._use_quaternion else 'euler'}"
                )

            vs = VesselState.from_float_array(data, use_quaternion=self._use_quaternion)
            with self._vessel_state_lock:
                self._vessel_states[ros_name] = vs
                # Store raw state array (strip timestamp) for sensor generation
                self._raw_vessel_states[ros_name] = np.array(data[1:])
                self._raw_vessel_state_times[ros_name] = float(data[0])
        except Exception as e:
            logger.warning(f"Error parsing vessel_state for {ros_name}: {e}")

    def subscribe_vessel_state_ders(
        self,
        vessel_ros_names: List[str],
    ) -> bool:
        """Subscribe to vessel_state_der topics for client-side sensor generation.

        Args:
            vessel_ros_names: List of ros_name strings.

        Returns:
            True if at least one subscription succeeded.
        """
        if not self._connected or self._ros is None:
            logger.error("Not connected to rosbridge")
            return False

        try:
            import roslibpy
        except ImportError:
            logger.error("roslibpy not installed")
            return False

        clean_ns = re.sub(r'^/+|/+$', '', self.namespace) if self.namespace else ''
        ok_count = 0

        for ros_name in vessel_ros_names:
            clean_name = re.sub(r'^/+|/+$', '', ros_name) if ros_name else ''
            if not clean_name:
                continue
            if clean_ns:
                topic = f"/{clean_ns}/{clean_name}/vessel_state_der"
            else:
                topic = f"/{clean_name}/vessel_state_der"

            try:
                t = roslibpy.Topic(self._ros, topic, 'std_msgs/Float64MultiArray')
                t.subscribe(lambda msg, _rn=clean_name: self._on_vessel_state_der(msg, _rn))
                self._vessel_state_der_topics.append(t)
                ok_count += 1
                logger.info(f"Subscribed to vessel state der: {topic}")
            except Exception as e:
                logger.warning(f"Failed to subscribe to {topic}: {e}")

        return ok_count > 0

    def _on_vessel_state_der(self, message: dict, ros_name: str):
        """Handle incoming vessel_state_der Float64MultiArray message."""
        try:
            data = message.get('data', [])
            if not data:
                return
            with self._vessel_state_lock:
                self._raw_vessel_state_ders[ros_name] = np.array(data[1:])
            for cb in self._state_der_callbacks:
                try:
                    cb(ros_name, data)
                except Exception:
                    pass
        except Exception as e:
            logger.warning(f"Error parsing vessel_state_der for {ros_name}: {e}")

    def add_state_der_callback(self, callback: Callable):
        """Register a callback(ros_name, data_list) for vessel_state_der updates."""
        self._state_der_callbacks.append(callback)

    def get_raw_state(self, ros_name: str) -> Optional[np.ndarray]:
        """Return the latest raw state array (no timestamp) for a vessel."""
        with self._vessel_state_lock:
            return self._raw_vessel_states.get(ros_name)

    def get_raw_state_der(self, ros_name: str) -> Optional[np.ndarray]:
        """Return the latest raw state derivative array (no timestamp) for a vessel."""
        with self._vessel_state_lock:
            return self._raw_vessel_state_ders.get(ros_name)

    def get_raw_state_time(self, ros_name: str) -> Optional[float]:
        """Return the elapsed sim time (s) of the latest vessel_state message for a vessel."""
        with self._vessel_state_lock:
            return self._raw_vessel_state_times.get(ros_name)

    def get_vessel_states(self) -> Dict[str, VesselState]:
        """Return a snapshot dict of all vessel states keyed by ros_name."""
        with self._vessel_state_lock:
            return dict(self._vessel_states)

    def get_vessel_state(self, ros_name: str) -> Optional[VesselState]:
        """Return the latest state for a single vessel, or None."""
        with self._vessel_state_lock:
            return self._vessel_states.get(ros_name)

    def is_connected(self) -> bool:
        """Check if connected to rosbridge."""
        return self._connected and self._ros is not None and self._ros.is_connected
    
    def close(self):
        """Close rosbridge connection."""
        if self._odometry_topic:
            try:
                self._odometry_topic.unsubscribe()
            except Exception:
                pass
            self._odometry_topic = None

        for t in self._vessel_state_topics:
            try:
                t.unsubscribe()
            except Exception:
                pass
        self._vessel_state_topics.clear()
        
        if self._ros:
            try:
                self._ros.terminate()
            except Exception:
                pass
            self._ros = None
        
        self._connected = False
        logger.info("Roslib subscriber closed")


# =============================================================================
# Proportional Controller
# =============================================================================

class ProportionalController:
    """
    Simple proportional controller for vessel heading and position control.
    
    This controller demonstrates basic closed-loop control:
    - P control for heading (yaw angle)
    - P control for forward velocity based on distance to target
    
    Users can extend this with PID, waypoint following, etc.
    """
    
    def __init__(
        self,
        kp_heading: float = 2.0,
        kp_velocity: float = 0.5,
        max_rudder_angle: float = 30.0,
        max_thruster_rpm: float = 3000.0,
        target_tolerance: float = 5.0
    ):
        """
        Initialize proportional controller.
        
        Args:
            kp_heading: Proportional gain for heading control
            kp_velocity: Proportional gain for velocity control
            max_rudder_angle: Maximum rudder deflection (degrees)
            max_thruster_rpm: Maximum thruster RPM
            target_tolerance: Distance tolerance for "arrived" (meters)
        """
        self.kp_heading = kp_heading
        self.kp_velocity = kp_velocity
        self.max_rudder_angle = max_rudder_angle
        self.max_thruster_rpm = max_thruster_rpm
        self.target_tolerance = target_tolerance
        
        # Target position
        self.target_x: float = 0.0
        self.target_y: float = 0.0
        self.target_heading: Optional[float] = None  # If None, head towards target
        
        # Current state (accepts Odometry or VesselState — both have position/orientation)
        self._latest_state = None
        self._arrived = False
        
        logger.info(
            f"P-controller initialized: Kp_heading={kp_heading}, "
            f"Kp_velocity={kp_velocity}"
        )
    
    def set_target(
        self,
        x: float,
        y: float,
        heading: Optional[float] = None
    ):
        """
        Set target position and optional heading.
        
        Args:
            x: Target X position (meters)
            y: Target Y position (meters)
            heading: Target heading in radians (None = point towards target)
        """
        self.target_x = x
        self.target_y = y
        self.target_heading = heading
        self._arrived = False
        
        logger.info(f"Target set: ({x:.1f}, {y:.1f}), heading={heading}")
    
    def update(self, state):
        """Update controller with latest vessel state (Odometry or VesselState)."""
        self._latest_state = state
    
    def compute_control(self) -> Tuple[float, float]:
        """
        Compute control signals based on current state and target.
        
        Returns:
            Tuple of (rudder_angle_degrees, thruster_rpm)
        """
        if self._latest_state is None:
            logger.warning("No state available, returning zero commands")
            return 0.0, 0.0
        
        st = self._latest_state
        
        # Get current position and heading
        current_x = st.position.x
        current_y = st.position.y
        _, _, current_yaw = st.orientation.to_euler()
        
        # Calculate distance to target
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Check if arrived
        if distance < self.target_tolerance:
            if not self._arrived:
                logger.info(f"Arrived at target (distance: {distance:.2f}m)")
                self._arrived = True
            return 0.0, 500.0  # Station keeping thrust
        
        self._arrived = False
        
        # Calculate desired heading (bearing to target)
        if self.target_heading is not None:
            desired_heading = self.target_heading
        else:
            desired_heading = math.atan2(dy, dx)
        
        # Calculate heading error (normalize to [-pi, pi])
        heading_error = desired_heading - current_yaw
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # P control for rudder
        rudder_angle = self.kp_heading * math.degrees(heading_error)
        rudder_angle = max(-self.max_rudder_angle, min(self.max_rudder_angle, rudder_angle))
        
        # P control for thruster based on distance
        # Reduce speed when close to target or when turning sharply
        heading_factor = 1.0 - abs(heading_error) / math.pi
        distance_factor = min(1.0, distance / 50.0)  # Scale by 50m
        
        thruster_rpm = self.kp_velocity * self.max_thruster_rpm * heading_factor * distance_factor
        thruster_rpm = max(500.0, min(self.max_thruster_rpm, thruster_rpm))
        
        logger.debug(
            f"Control: dist={distance:.1f}m, heading_err={math.degrees(heading_error):.1f}°, "
            f"rudder={rudder_angle:.1f}°, thrust={thruster_rpm:.0f}rpm"
        )
        
        return rudder_angle, thruster_rpm
    
    def has_arrived(self) -> bool:
        """Check if vessel has arrived at target."""
        return self._arrived


# =============================================================================
# Main Controller Class
# =============================================================================

class MavsimController:
    """
    Main controller class combining API client, roslib subscriber, and controller.
    
    This class provides a unified interface for controlling simulated vessels:
    - Sends commands via REST API
    - Receives feedback via roslib (WebSocket)
    - Implements closed-loop control
    """
    
    def __init__(
        self,
        backend_url: str,
        session_id: str,
        api_token: str,
        rosbridge_url: str,
        namespace: str = '',
        vessel_name: str = 'vessel_01',
        vessel_config: Dict = None,
        vessel_id: Optional[int] = None,
        vessels: Optional[List[Dict]] = None,
        use_quaternion: bool = False,
    ):
        """
        Initialize mavsim controller.
        
        Args:
            backend_url: Backend URL (e.g., http://localhost:5000)
            session_id: Session UUID
            api_token: Per-session API token
            rosbridge_url: Rosbridge WebSocket URL (e.g., ws://localhost:9090)
            namespace: ROS namespace (e.g., 'sim_abc123')
            vessel_name: Vessel name (default: vessel_01)
            vessel_config: Full vessel configuration dict from handshake response.
            vessel_id: Numeric vessel ID for this controller's vessel.
            vessels: List of all vessels in session (each with 'ros_name', 'name', etc.)
            use_quaternion: Whether the sim uses quaternion attitude representation.
        """
        self.backend_url = backend_url
        self.session_id = session_id
        self.api_token = api_token
        self.rosbridge_url = rosbridge_url
        self.namespace = namespace
        self.vessel_name = vessel_name
        self.vessel_config = vessel_config or {}
        self.vessel_id = vessel_id if vessel_id is not None else 0
        self.vessels = vessels or []
        self.use_quaternion = use_quaternion
        
        # Extract actuator information from vessel config
        self.control_surfaces = []
        self.thrusters = []
        
        if vessel_config:
            self.control_surfaces = vessel_config.get('control_surfaces') or []
            self.thrusters = vessel_config.get('thrusters') or []
        
        if not self.control_surfaces and not self.thrusters:
            logger.warning(
                f"No actuators found in vessel config. "
                f"Controller may not work correctly."
            )
        
        # Initialize components
        self.api_client = MavsimAPIClient(
            backend_url=backend_url,
            session_id=session_id,
            api_token=api_token,
            vessel_name=vessel_name
        )
        
        self.subscriber = RoslibSubscriber(
            rosbridge_url=rosbridge_url,
            namespace=namespace,
            vessel_name=vessel_name
        )
        
        self.controller = ProportionalController()
        
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        
        # Sensor bridge for local sensor data (Task 2.7)
        self._sensor_bridge = None
        self._bridge_task: Optional[asyncio.Task] = None
        self._bridge_loop: Optional[asyncio.AbstractEventLoop] = None
        self._bridge_thread: Optional[threading.Thread] = None
        
        logger.info(
            f"MavsimController initialized for vessel: {vessel_name} "
            f"({len(self.control_surfaces)} control surfaces, {len(self.thrusters)} thrusters)"
        )
        
        if self.control_surfaces:
            cs_info = ', '.join([f"cs_{cs.get('actuator_id', '?'):02d}" for cs in self.control_surfaces])
            logger.info(f"  Control surfaces: {cs_info}")
        
        if self.thrusters:
            th_info = ', '.join([f"th_{th.get('actuator_id', '?'):02d}" for th in self.thrusters])
            logger.info(f"  Thrusters: {th_info}")
    
    def enable_local_sensors(
        self,
        camera_port: int = 8765,
        lidar_port: int = 8766,
        enable_ros2: bool = True,
        camera_ids: Optional[List[int]] = None,
        lidar_ids: Optional[List[int]] = None,
    ):
        """
        Enable local sensor bridge for receiving browser sensor data.
        
        This method initializes the sensor bridge that will receive sensor data
        (e.g., camera frames) from the browser via WebSocket. The bridge will
        be started automatically when connect() is called.
        
        When enable_ros2 is True (default), this vessel's camera frames are
        also published to local ROS2 topics so client bag recording can include
        them (Task 2.5). Only this controller's vessel sensors are published.
        
        Args:
            camera_port: Port for camera sensor server (default: 8765)
            lidar_port: Port for lidar sensor server (default: 8766)
            enable_ros2: If True, publish this vessel's camera frames to local ROS2 (default: True)
            camera_ids: List of camera IDs from config (e.g. from handshake cameraIds).
                        If provided, ROS2 topics for these IDs are pre-created so all cameras
                        appear as separate topics (e.g. camera_01, camera_02).
            lidar_ids: List of lidar IDs from config (e.g. from handshake lidarIds).
                       If provided, ROS2 topics for these IDs are pre-created so all lidars
                       appear as separate topics (e.g. lidar_01, lidar_02).
        """
        try:
            from mavsim_sensor_bridge import SensorBridge, BridgeConfig
            
            config = BridgeConfig(
                camera_port=camera_port,
                lidar_port=lidar_port,
                camera_enabled=True,
                lidar_enabled=True,
            )
            self._sensor_bridge = SensorBridge(config=config)
            if enable_ros2:
                self._sensor_bridge.enable_ros2(
                    controlled_vessel_id=self.vessel_id,
                    namespace=self.namespace,
                    vessel_name=self.vessel_name,
                    camera_ids=camera_ids,
                    lidar_ids=lidar_ids,
                )
            logger.info(
                f"Local sensor bridge enabled (camera_port={camera_port}, lidar_port={lidar_port}, enable_ros2={enable_ros2}, "
                f"camera_ids={camera_ids}, lidar_ids={lidar_ids})"
            )
        except ImportError as e:
            logger.error(f"Failed to import sensor bridge: {e}")
            logger.error("Install with: pip install mavsim-sensor-bridge")
            raise
    
    def on_camera(self, vessel_id: int, camera_id: int):
        """
        Decorator to register camera frame callback.
        
        The decorated function will be called whenever a camera frame is received
        for the specified vessel and camera.
        
        Args:
            vessel_id: Vessel identifier (0-255)
            camera_id: Camera identifier (0-255)
        
        Returns:
            Decorator function
        
        Example:
            @controller.on_camera(vessel_id=1, camera_id=1)
            def handle_frame(vessel_id, camera_id, timestamp, jpeg_data):
                # Process camera frame
                pass
        """
        def decorator(func: Callable):
            if self._sensor_bridge:
                self._sensor_bridge.on_camera(vessel_id, camera_id, func)
                logger.info(f"Registered camera callback for vessel_id={vessel_id}, camera_id={camera_id}")
            else:
                logger.warning(
                    "Sensor bridge not enabled. Call enable_local_sensors() first. "
                    "Callback will not be registered."
                )
            return func
        return decorator

    def on_lidar(self, vessel_id: int, lidar_id: int = 0):
        """
        Decorator to register lidar scan callback.

        The decorated function will be called whenever a lidar scan is received
        for the specified vessel and lidar.

        Args:
            vessel_id: Vessel identifier (0-255)
            lidar_id: Lidar sensor identifier (0-255), default 0

        Example:
            @controller.on_lidar(vessel_id=1, lidar_id=0)
            def handle_scan(vessel_id, lidar_id, points, timestamp):
                # points: numpy (N, 4) float32 — x, y, z, intensity
                pass
        """
        def decorator(func: Callable):
            if self._sensor_bridge:
                self._sensor_bridge.on_lidar(vessel_id, func, lidar_id=lidar_id)
                logger.info(f"Registered lidar callback for vessel_id={vessel_id}, lidar_id={lidar_id}")
            else:
                logger.warning(
                    "Sensor bridge not enabled. Call enable_local_sensors() first. "
                    "Callback will not be registered."
                )
            return func
        return decorator
    
    def connect(self, subscribe_odometry: bool = True) -> bool:
        """
        Connect to rosbridge and optionally subscribe to odometry.
        
        If local sensors are enabled, also starts the sensor bridge.

        Args:
            subscribe_odometry: Whether to subscribe to the server's odometry_sim
                topic. Set to False when using BaseController, which generates
                odometry locally from vessel_state to avoid duplicate bandwidth.
        
        Returns:
            True if connected successfully
        """
        if not self.subscriber.connect():
            return False
        
        if subscribe_odometry:
            if not self.subscriber.subscribe_odometry(self.controller.update):
                return False
        
        # Start sensor bridge if enabled (Task 2.7)
        if self._sensor_bridge:
            try:
                # Run bridge in a background thread with its own event loop
                def run_bridge():
                    """Run bridge in background thread."""
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    self._bridge_loop = loop
                    try:
                        loop.run_until_complete(self._sensor_bridge.start())
                    except Exception as e:
                        logger.error(f"Bridge error: {e}")
                    finally:
                        loop.close()
                
                self._bridge_thread = threading.Thread(target=run_bridge, daemon=True)
                self._bridge_thread.start()
                logger.info("Sensor bridge started in background thread")
            except Exception as e:
                logger.error(f"Failed to start sensor bridge: {e}")
                # Don't fail the connection if bridge fails to start
                # The controller can still work without sensor bridge
        
        return True
    
    def send_command(
        self,
        actuator_names: List[str],
        actuator_values: List[float]
    ) -> bool:
        """
        Send actuator command.
        
        Args:
            actuator_names: List of actuator names (e.g., ['cs_01', 'th_01'])
                           Format: 'cs_XX' for control surfaces, 'th_XX' for thrusters
                           where XX is zero-padded actuator_id
            actuator_values: List of actuator values
                           Control surfaces: angle in degrees (-180 to 180)
                           Thrusters: RPM (0 to 10000)
            
        Returns:
            True if successful
        """
        return self.api_client.send_command(actuator_names, actuator_values)
    
    def get_odometry(self) -> Optional[Odometry]:
        """Get latest odometry."""
        return self.subscriber.get_odometry()

    def subscribe_vessel_states(self) -> bool:
        """Subscribe to vessel_state topics for every vessel in the session."""
        ros_names = [v.get('ros_name', '') for v in self.vessels if v.get('ros_name')]
        if not ros_names:
            ros_names = [self.vessel_name]
        return self.subscriber.subscribe_vessel_states(
            ros_names, use_quaternion=self.use_quaternion
        )

    def subscribe_vessel_state_ders(self) -> bool:
        """Subscribe to vessel_state_der topics for client-side sensor generation."""
        ros_names = [v.get('ros_name', '') for v in self.vessels if v.get('ros_name')]
        if not ros_names:
            ros_names = [self.vessel_name]
        return self.subscriber.subscribe_vessel_state_ders(ros_names)

    def get_vessel_states(self) -> Dict[str, VesselState]:
        """Return a snapshot dict of all vessel states keyed by ros_name."""
        return self.subscriber.get_vessel_states()

    def get_vessel_state(self, ros_name: str) -> Optional[VesselState]:
        """Return the latest state for a single vessel, or None."""
        return self.subscriber.get_vessel_state(ros_name)

    def get_raw_state(self, ros_name: str) -> Optional[np.ndarray]:
        """Return latest raw state array (no timestamp) for sensor generation."""
        return self.subscriber.get_raw_state(ros_name)

    def get_raw_state_der(self, ros_name: str) -> Optional[np.ndarray]:
        """Return latest raw state derivative array (no timestamp)."""
        return self.subscriber.get_raw_state_der(ros_name)

    def get_raw_state_time(self, ros_name: str) -> Optional[float]:
        """Return the elapsed sim time (s) of the latest vessel_state message."""
        return self.subscriber.get_raw_state_time(ros_name)

    def set_target(self, x: float, y: float, heading: Optional[float] = None):
        """
        Set target position for proportional controller.
        
        Args:
            x: Target X position
            y: Target Y position
            heading: Target heading (radians, None = auto)
        """
        self.controller.set_target(x, y, heading)
    
    def start_control_loop(self, rate_hz: float = 10.0):
        """
        Start automatic control loop.
        
        Args:
            rate_hz: Control loop frequency in Hz
        """
        if self._running:
            logger.warning("Control loop already running")
            return
        
        self._running = True
        self._control_thread = threading.Thread(
            target=self._control_loop,
            args=(rate_hz,),
            daemon=True
        )
        self._control_thread.start()
        logger.info(f"Control loop started at {rate_hz} Hz")
    
    def _control_loop(self, rate_hz: float):
        """Internal control loop."""
        period = 1.0 / rate_hz
        
        while self._running:
            try:
                # Compute control signals
                rudder_angle, thruster_rpm = self.controller.compute_control()
                
                # Build actuator commands from vessel config
                # This example uses the first control surface and first thruster
                # For more complex control, override this method to use all actuators
                actuator_names = []
                actuator_values = []
                
                # Add first control surface (rudder)
                if len(self.control_surfaces) > 0:
                    first_cs = self.control_surfaces[0]
                    cs_actuator_id = first_cs.get('actuator_id')
                    if cs_actuator_id is not None:
                        cs_name = f'cs_{cs_actuator_id:02d}'
                        actuator_names.append(cs_name)
                        actuator_values.append(rudder_angle)
                
                # Add first thruster (main propulsion)
                if len(self.thrusters) > 0:
                    first_th = self.thrusters[0]
                    th_actuator_id = first_th.get('actuator_id')
                    if th_actuator_id is not None:
                        th_name = f'th_{th_actuator_id:02d}'
                        actuator_names.append(th_name)
                        actuator_values.append(thruster_rpm)
                
                if not actuator_names:
                    logger.error("No actuators configured - cannot send commands")
                    continue
                
                self.api_client.send_command(actuator_names, actuator_values)
                
            except Exception as e:
                logger.error(f"Control loop error: {e}")
            
            time.sleep(period)
    
    def send_actuator_commands(self, control_surface_commands: Dict[int, float] = None, 
                               thruster_commands: Dict[int, float] = None) -> bool:
        """
        Send commands to multiple actuators.
        
        Args:
            control_surface_commands: Dict mapping actuator_id to angle (degrees)
                                     Example: {2: 15.0, 3: -10.0} for rudder and elevator
            thruster_commands: Dict mapping actuator_id to RPM
                              Example: {1: 2000.0, 4: 500.0} for main and bow thruster
        
        Returns:
            True if successful
        
        Example:
            # Control rudder (actuator_id=2) and main thruster (actuator_id=1)
            controller.send_actuator_commands(
                control_surface_commands={2: 15.0},  # 15 degrees rudder
                thruster_commands={1: 2000.0}        # 2000 RPM main thruster
            )
        """
        actuator_names = []
        actuator_values = []
        
        if control_surface_commands:
            for actuator_id, angle in control_surface_commands.items():
                actuator_names.append(f'cs_{actuator_id:02d}')
                actuator_values.append(angle)
        
        if thruster_commands:
            for actuator_id, rpm in thruster_commands.items():
                actuator_names.append(f'th_{actuator_id:02d}')
                actuator_values.append(rpm)
        
        if not actuator_names:
            logger.warning("No actuator commands provided")
            return False
        
        return self.api_client.send_command(actuator_names, actuator_values)
    
    def stop_control_loop(self):
        """Stop automatic control loop."""
        self._running = False
        if self._control_thread:
            self._control_thread.join(timeout=2.0)
            self._control_thread = None
        logger.info("Control loop stopped")
    
    def has_arrived(self) -> bool:
        """Check if vessel has arrived at target."""
        return self.controller.has_arrived()
    
    def get_control_surfaces(self) -> List[Dict]:
        """
        Get list of all control surfaces with their actuator_ids and names.
        
        Returns:
            List of control surface dicts, each with 'actuator_id' and 'name' fields
        """
        return self.control_surfaces.copy()
    
    def get_thrusters(self) -> List[Dict]:
        """
        Get list of all thrusters with their actuator_ids and names.
        
        Returns:
            List of thruster dicts, each with 'actuator_id' and 'name' fields
        """
        return self.thrusters.copy()
    
    def get_vessel_config(self) -> Dict:
        """
        Get the full vessel configuration dict.
        
        Returns:
            Complete vessel configuration including geometry, inertia, actuators, etc.
        """
        return self.vessel_config.copy() if self.vessel_config else {}
    
    def close(self):
        """Close all connections and stop control loop."""
        self.stop_control_loop()
        
        # Stop sensor bridge if running (Task 2.7)
        if self._sensor_bridge and self._bridge_loop:
            try:
                # Stop bridge by calling stop() in the bridge's event loop
                async def stop_bridge():
                    await self._sensor_bridge.stop()
                
                # Schedule stop in the bridge's event loop
                if self._bridge_loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(stop_bridge(), self._bridge_loop)
                    # Wait up to 2 seconds for stop to complete
                    future.result(timeout=2.0)
                else:
                    # If loop is not running, just wait for thread to finish
                    pass
                
                # Wait for bridge thread to finish
                if self._bridge_thread and self._bridge_thread.is_alive():
                    self._bridge_thread.join(timeout=2.0)
                
                logger.info("Sensor bridge stopped")
            except Exception as e:
                logger.warning(f"Error stopping sensor bridge: {e}")
        
        self.subscriber.close()
        logger.info("MavsimController closed")


# =============================================================================
# Main Script
# =============================================================================

def main():
    """Main entry point for example controller."""
    parser = argparse.ArgumentParser(
        description='Example Python Controller for mavsim Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:

Handshake mode (recommended - use controller code from UI):
    python python_controller.py \\
        --backend-url http://localhost:5000 \\
        --code ABC123 \\
        --target-x 100 --target-y 50

Direct mode (legacy - requires explicit credentials):
    python python_controller.py \\
        --backend-url http://localhost:5000 \\
        --session-id 550e8400-e29b-41d4-a716-446655440000 \\
        --api-token abc123xyz789 \\
        --rosbridge-url ws://localhost:9090 \\
        --namespace sim_abc123 \\
        --target-x 100 --target-y 50

How to get controller code:
    1. Start simulation from web UI with "External controllers present" enabled
    2. Copy the controller code displayed on the simulation page
    3. Use --code option with the controller code
"""
    )
    
    parser.add_argument(
        '--backend-url', '-b',
        required=True,
        help='Backend URL (e.g., http://localhost:5000)'
    )
    
    # Connection mode: either handshake (new) or direct (legacy)
    connection_group = parser.add_mutually_exclusive_group(required=True)
    connection_group.add_argument(
        '--code', '-c',
        help='Controller code from simulation UI (handshake mode)'
    )
    connection_group.add_argument(
        '--session-id', '-s',
        help='Session UUID (direct mode, requires --api-token and --rosbridge-url)'
    )
    
    parser.add_argument(
        '--api-token', '-t',
        help='API token (required for direct mode)'
    )
    parser.add_argument(
        '--rosbridge-url', '-r',
        help='Rosbridge WebSocket URL (required for direct mode, e.g., ws://localhost:9090)'
    )
    parser.add_argument(
        '--namespace', '-n',
        default='',
        help='ROS namespace (optional for direct mode, e.g., sim_abc123)'
    )
    parser.add_argument(
        '--vessel-name', '-v',
        help='Vessel name (optional, auto-assigned in handshake mode)'
    )
    parser.add_argument(
        '--controller-id',
        help='Controller identifier for logging (optional)'
    )
    parser.add_argument(
        '--target-x',
        type=float,
        default=100.0,
        help='Target X position (meters)'
    )
    parser.add_argument(
        '--target-y',
        type=float,
        default=0.0,
        help='Target Y position (meters)'
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=10.0,
        help='Control loop rate in Hz (default: 10)'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug logging'
    )
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Determine connection mode
    if args.code:
        # Handshake mode: use controller code
        logger.info("Using handshake mode with controller code")
        try:
            api_client, handshake_data = MavsimAPIClient.from_handshake(
                backend_url=args.backend_url,
                controller_code=args.code,
                vessel_name=args.vessel_name,
                controller_id=args.controller_id
            )
            
            # Extract connection details from handshake response
            session_id = handshake_data.get('sessionId')
            api_token = handshake_data.get('apiToken')
            rosbridge_url = handshake_data.get('rosbridgeUrl')
            namespace = handshake_data.get('namespace', '')
            vessel_name = handshake_data.get('vesselName') or args.vessel_name or 'vessel_01'
            status = handshake_data.get('status')
            vessels = handshake_data.get('vessels', [])
            vessel_config = handshake_data.get('vesselConfig')  # Full vessel config from S3
            
            # Find vessel_id for the chosen vessel
            vessel_id = '00'  # Default
            for v in vessels:
                if v.get('name') == vessel_name:
                    vessel_id = str(v.get('vessel_id', '0')).zfill(2)  # Pad to 2 digits
                    break
            
            # Construct full vessel topic name: {vessel_name}_{vessel_id:02d}
            # This matches ROS2 topic format: /{namespace}/{vessel_name}_{vessel_id:02d}/odometry_sim
            full_vessel_name = f"{vessel_name}_{vessel_id}"
            
            logger.info(f"Handshake successful: sessionId={session_id}, status={status}")
            logger.info(f"  namespace (from handshake): '{namespace}'")
            logger.info(f"  vessel_name: '{vessel_name}'")
            logger.info(f"  vessel_id: '{vessel_id}'")
            logger.info(f"  full_vessel_name (for topic): '{full_vessel_name}'")
            logger.info(f"  rosbridge_url: '{rosbridge_url}'")
            
            if not rosbridge_url:
                logger.error("Handshake response missing rosbridgeUrl")
                sys.exit(1)
            
            # If status is 'waiting_for_controller', the simulation should transition to 'running'
            # Give it a moment to start processes
            if status == 'waiting_for_controller':
                logger.info("Session is waiting for controller - processes should start shortly...")
                time.sleep(2.0)  # Give simulation time to spawn processes
            elif status != 'running':
                logger.warning(f"Session status is '{status}' (expected 'running') - simulation may not be ready")
            
        except requests.exceptions.HTTPError as e:
            logger.error(f"Handshake failed: HTTP {e.response.status_code}")
            if e.response.status_code == 404:
                logger.error("Controller code is invalid or expired")
            elif e.response.status_code == 400:
                error_data = e.response.json()
                logger.error(f"Handshake error: {error_data.get('message', 'Unknown error')}")
            sys.exit(1)
        except Exception as e:
            logger.error(f"Handshake failed: {e}")
            sys.exit(1)
    else:
        # Direct mode: use explicit credentials
        logger.info("Using direct mode with explicit credentials")
        if not args.session_id or not args.api_token or not args.rosbridge_url:
            logger.error("Direct mode requires --session-id, --api-token, and --rosbridge-url")
            sys.exit(1)
        
        api_client = MavsimAPIClient(
            backend_url=args.backend_url,
            session_id=args.session_id,
            api_token=args.api_token,
            vessel_name=args.vessel_name or 'vessel_01'
        )
        
        session_id = args.session_id
        api_token = args.api_token
        rosbridge_url = args.rosbridge_url
        namespace = args.namespace
        vessel_name = args.vessel_name or 'vessel_01'
        # For direct mode, assume vessel_id is 00 (or extract from vessel_name if it includes _XX)
        if '_' in vessel_name and vessel_name.split('_')[-1].isdigit():
            # Vessel name already includes ID (e.g., "matsya_00")
            full_vessel_name = vessel_name
        else:
            # Default to vessel_id 00
            full_vessel_name = f"{vessel_name}_00"
    
    # Create controller (use full_vessel_name for topic subscription AND control commands)
    logger.info(f"Creating controller with full_vessel_name: '{full_vessel_name}'")
    
    # For direct mode, vessel config is not available (legacy mode)
    # Users should use handshake mode to get full vessel config from S3
    if not args.code:
        logger.warning(
            "Direct mode does not provide vessel configuration from S3. "
            "Consider using handshake mode (--code) to automatically get full vessel config."
        )
    elif vessel_config:
        cs_count = len(vessel_config.get('control_surfaces', {}).get('control_surfaces', []))
        th_count = len(vessel_config.get('thrusters', {}).get('thrusters', []))
        logger.info(f"Vessel config loaded: {cs_count} control surfaces, {th_count} thrusters")
    else:
        logger.warning("No vessel configuration in handshake response - controller may not work correctly")
    
    controller = MavsimController(
        backend_url=args.backend_url,
        session_id=session_id,
        api_token=api_token,
        rosbridge_url=rosbridge_url,
        namespace=namespace,
        vessel_name=full_vessel_name,  # Use full name with vessel_id for both odometry subscription AND control commands
        vessel_config=vessel_config if args.code else None  # Only available in handshake mode
    )
    
    # Setup signal handler for clean shutdown
    def signal_handler(sig, frame):
        logger.info("Shutting down...")
        controller.close()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Connect to rosbridge
    logger.info("Connecting to rosbridge...")
    if not controller.connect():
        logger.error("Failed to connect to rosbridge")
        sys.exit(1)
    
    # Wait for first odometry
    logger.info("Waiting for odometry (this may take a few seconds while simulation initializes)...")
    timeout = 30.0  # Increased timeout to allow simulation to fully start
    start_time = time.time()
    while controller.get_odometry() is None:
        elapsed = time.time() - start_time
        if elapsed > timeout:
            # Get the actual topic name being subscribed to
            actual_topic = controller.subscriber._odometry_topic.name if hasattr(controller.subscriber, '_odometry_topic') and controller.subscriber._odometry_topic else 'unknown'
            logger.error(
                f"Timeout waiting for odometry after {timeout}s. "
                "Possible issues:\n"
                "  1. Simulation processes may not have started\n"
                "  2. Rosbridge may not be connected properly\n"
                "  3. Simulation may not be publishing odometry\n"
                f"  4. Topic name may be incorrect: {actual_topic}"
            )
            # Check rosbridge connection
            if not controller.subscriber.is_connected():
                logger.error("Rosbridge is not connected!")
            else:
                logger.info("Rosbridge is connected, but no odometry received")
            controller.close()
            sys.exit(1)
        if int(elapsed) % 5 == 0 and elapsed > 0:  # Log every 5 seconds
            logger.info(f"Still waiting for odometry... ({int(elapsed)}s elapsed)")
        time.sleep(0.1)
    
    initial_odom = controller.get_odometry()
    logger.info(
        f"Initial position: ({initial_odom.position.x:.1f}, "
        f"{initial_odom.position.y:.1f})"
    )
    
    # Set target and start control
    controller.set_target(args.target_x, args.target_y)
    controller.start_control_loop(rate_hz=args.rate)
    
    logger.info(f"Navigating to ({args.target_x}, {args.target_y})...")
    
    # Monitor progress
    try:
        while not controller.has_arrived():
            odom = controller.get_odometry()
            if odom:
                _, _, yaw = odom.orientation.to_euler()
                logger.info(
                    f"Position: ({odom.position.x:.1f}, {odom.position.y:.1f}), "
                    f"Heading: {math.degrees(yaw):.1f}°"
                )
            time.sleep(2.0)
        
        logger.info("Target reached!")
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    
    finally:
        controller.close()


if __name__ == '__main__':
    main()
