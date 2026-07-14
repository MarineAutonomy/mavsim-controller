#!/usr/bin/env python3
"""
Local Sensor Generator for mavsim Controller Docker

Generates IMU, GPS, DVL, and Encoder sensor measurements locally from
vessel_state and vessel_state_der data received via rosbridge. This moves
non-perception sensor generation from the server to the controller Docker,
consistent with the client-side rendering architecture.

The sensor classes here mirror the math from ros2_ws/src/mavsim/mavsim/module_sensors.py
but operate on raw numpy state arrays instead of accessing vessel_node references.

State vector layout (without leading timestamp):
    [u, v, w, p, q, r, x, y, z, {phi, theta, psi} or {q0, q1, q2, q3}, actuators...]

State derivative vector layout (without leading timestamp):
    [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, ...]

Author: mavsim Team
License: MIT
"""

import logging
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Kinematics helpers (subset of module_kinematics.py)
# ---------------------------------------------------------------------------


def _ssa(ang: float) -> float:
    """Smallest signed angle (radians)."""
    return (ang + np.pi) % (2 * np.pi) - np.pi


def _eul_to_rotm(eul: np.ndarray) -> np.ndarray:
    """Euler angles [phi, theta, psi] -> 3x3 rotation matrix (ZYX order)."""
    phi, theta, psi = eul[0], eul[1], eul[2]
    c1, s1 = np.cos(phi), np.sin(phi)
    c2, s2 = np.cos(theta), np.sin(theta)
    c3, s3 = np.cos(psi), np.sin(psi)
    return np.array([
        [c2 * c3, -c1 * s3 + s1 * s2 * c3, s1 * s3 + c1 * s2 * c3],
        [c2 * s3, c1 * c3 + s1 * s2 * s3, -s1 * c3 + c1 * s2 * s3],
        [-s2, s1 * c2, c1 * c2],
    ])


def _eul_to_quat(eul: np.ndarray) -> np.ndarray:
    """Euler angles [phi, theta, psi] -> quaternion [qw, qx, qy, qz] (ZYX order)."""
    phi, theta, psi = eul[0], eul[1], eul[2]
    cr, sr = np.cos(phi / 2), np.sin(phi / 2)
    cp, sp = np.cos(theta / 2), np.sin(theta / 2)
    cy, sy = np.cos(psi / 2), np.sin(psi / 2)
    quat = np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    ])
    return quat / np.linalg.norm(quat)


def _quat_to_eul(quat: np.ndarray) -> np.ndarray:
    """Quaternion [qw, qx, qy, qz] -> Euler angles [phi, theta, psi] (ZYX order)."""
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    theta = np.arcsin(np.clip(2 * (qy * qw - qx * qz), -1.0, 1.0))
    phi = np.arctan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2))
    psi = np.arctan2(2 * (qx * qy + qz * qw), 1 - 2 * (qy ** 2 + qz ** 2))
    return np.array([phi, theta, psi])


def _quat_to_rotm(quat: np.ndarray) -> np.ndarray:
    """Quaternion [qw, qx, qy, qz] -> 3x3 rotation matrix."""
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    return np.array([
        [1 - 2 * qy ** 2 - 2 * qz ** 2, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw],
        [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx ** 2 - 2 * qz ** 2, 2 * qy * qz - 2 * qx * qw],
        [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx ** 2 - 2 * qy ** 2],
    ])


def _rotm_to_quat(rotm: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> quaternion [qw, qx, qy, qz]."""
    trace = np.trace(rotm)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        w = 0.25 * S
        x = (rotm[2, 1] - rotm[1, 2]) / S
        y = (rotm[0, 2] - rotm[2, 0]) / S
        z = (rotm[1, 0] - rotm[0, 1]) / S
    elif rotm[0, 0] > rotm[1, 1] and rotm[0, 0] > rotm[2, 2]:
        S = np.sqrt(1.0 + rotm[0, 0] - rotm[1, 1] - rotm[2, 2]) * 2
        w = (rotm[2, 1] - rotm[1, 2]) / S
        x = 0.25 * S
        y = (rotm[0, 1] + rotm[1, 0]) / S
        z = (rotm[0, 2] + rotm[2, 0]) / S
    elif rotm[1, 1] > rotm[2, 2]:
        S = np.sqrt(1.0 + rotm[1, 1] - rotm[0, 0] - rotm[2, 2]) * 2
        w = (rotm[0, 2] - rotm[2, 0]) / S
        x = (rotm[0, 1] + rotm[1, 0]) / S
        y = 0.25 * S
        z = (rotm[1, 2] + rotm[2, 1]) / S
    else:
        S = np.sqrt(1.0 + rotm[2, 2] - rotm[0, 0] - rotm[1, 1]) * 2
        w = (rotm[1, 0] - rotm[0, 1]) / S
        x = (rotm[0, 2] + rotm[2, 0]) / S
        y = (rotm[1, 2] + rotm[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z])


def _ned_to_llh(ned: np.ndarray, llh0: np.ndarray) -> np.ndarray:
    """NED position -> latitude/longitude/height given a GPS datum."""
    xn, yn, zn = ned[0], ned[1], ned[2]
    mu0 = llh0[0] * np.pi / 180
    l0 = llh0[1] * np.pi / 180
    h0 = llh0[2]

    re = 6378137.0
    rp = 6356752.314245
    e = np.sqrt(1 - (rp / re) ** 2)

    RN = re / np.sqrt(1 - (e * np.sin(mu0)) ** 2)
    RM = re * (1 - e ** 2) / (1 - (e * np.sin(mu0)) ** 2)

    dmu = xn * np.arctan2(1, RN)
    dl = yn * np.arctan2(1, RM * np.cos(mu0))

    mu = _ssa(mu0 + dmu) * 180 / np.pi
    lon = _ssa(l0 + dl) * 180 / np.pi
    h = h0 - zn

    return np.array([mu, lon, h])


# ---------------------------------------------------------------------------
# Standalone sensor classes (no vessel_node dependency)
# ---------------------------------------------------------------------------


class LocalIMUSensor:
    """IMU sensor that operates on raw state/state_der arrays."""

    def __init__(self, sensor_config: dict, gravity: float = 9.80665):
        self.sensor_type = 'IMU'
        self.rate = sensor_config.get('publish_rate', 100)
        self.location = np.array(sensor_config.get('sensor_location', [0, 0, 0]), dtype=float)
        self.orientation = np.array(sensor_config.get('sensor_orientation', [0, 0, 0]), dtype=float) * np.pi / 180.0
        self.gravity = gravity

        self.eul_rms = np.array([1, 1, 1]) * 1e-2
        self.eul_cov = np.diag(self.eul_rms ** 2)
        self.ang_vel_rms = np.array([1, 1, 1]) * 1e-2
        self.ang_vel_cov = np.diag(self.ang_vel_rms ** 2)
        self.lin_acc_rms = np.array([1, 1, 1]) * 1.5e-1
        self.lin_acc_cov = np.diag(self.lin_acc_rms ** 2)

        custom_cov = sensor_config.get('custom_covariance') or sensor_config.get('covariance') or {}
        if custom_cov:
            if 'orientation_covariance' in custom_cov:
                self.eul_cov = np.array(custom_cov['orientation_covariance']).reshape(3, 3)
            if 'angular_velocity_covariance' in custom_cov:
                self.ang_vel_cov = np.array(custom_cov['angular_velocity_covariance']).reshape(3, 3)
            if 'linear_acceleration_covariance' in custom_cov:
                self.lin_acc_cov = np.array(custom_cov['linear_acceleration_covariance']).reshape(3, 3)

    def get_measurement(self, state: np.ndarray, state_der: np.ndarray,
                        use_quaternion: bool = False) -> dict:
        if not use_quaternion:
            eul = state[9:12]
            quat = _eul_to_quat(eul)
            orientation_quat = _eul_to_quat(self.orientation)
        else:
            quat = state[9:13]
            orientation_quat = _eul_to_quat(self.orientation)

        omg_bcs = state[3:6]
        v_bcs = state[0:3]
        alpha = state_der[3:6]

        q_sensor = _rotm_to_quat(_quat_to_rotm(quat) @ _quat_to_rotm(orientation_quat))
        q_sensor = _eul_to_quat(
            _quat_to_eul(q_sensor) + np.random.multivariate_normal(np.zeros(3), self.eul_cov)
        )

        acc_bcs = state_der[0:3] + np.cross(omg_bcs, v_bcs)
        acc_s_bcs = acc_bcs + np.cross(alpha, self.location) + np.cross(omg_bcs, np.cross(omg_bcs, self.location))
        acc_sensor = _quat_to_rotm(orientation_quat).T @ acc_s_bcs
        acc_sensor = acc_sensor + _quat_to_rotm(q_sensor).T @ np.array([0, 0, -self.gravity])
        acc_sensor = acc_sensor + np.random.multivariate_normal(np.zeros(3), self.lin_acc_cov)

        omg_sensor = _quat_to_rotm(orientation_quat).T @ omg_bcs
        omg_sensor = omg_sensor + np.random.multivariate_normal(np.zeros(3), self.ang_vel_cov)

        return {
            'orientation': q_sensor,
            'angular_velocity': omg_sensor,
            'linear_acceleration': acc_sensor,
            'orientation_covariance': self.eul_cov.flatten(),
            'angular_velocity_covariance': self.ang_vel_cov.flatten(),
            'linear_acceleration_covariance': self.lin_acc_cov.flatten(),
        }


class LocalGPSSensor:
    """GPS sensor that operates on raw state arrays."""

    def __init__(self, sensor_config: dict, gps_datum: list):
        self.sensor_type = 'GPS'
        self.rate = sensor_config.get('publish_rate', 50)
        self.location = np.array(sensor_config.get('sensor_location', [0, 0, 0]), dtype=float)
        self.gps_datum = np.array(gps_datum, dtype=float)

        self.gps_rms = np.array([3, 3, 3], dtype=float)
        self.gps_cov = np.diag(self.gps_rms ** 2)

        custom_cov = sensor_config.get('custom_covariance') or sensor_config.get('covariance') or {}
        if custom_cov and 'position_covariance' in custom_cov:
            self.gps_cov = np.array(custom_cov['position_covariance']).reshape(3, 3)

    def get_measurement(self, state: np.ndarray, state_der: np.ndarray,
                        use_quaternion: bool = False) -> dict:
        if not use_quaternion:
            orientation = _eul_to_quat(state[9:12])
        else:
            orientation = state[9:13]

        ned = state[6:9] + _quat_to_rotm(orientation) @ self.location
        ned = ned + np.random.multivariate_normal(np.zeros(3), self.gps_cov)
        llh = _ned_to_llh(ned, self.gps_datum)

        return {
            'latitude': llh[0],
            'longitude': llh[1],
            'altitude': llh[2],
            'position_covariance': self.gps_cov.flatten(),
        }


class LocalDVLSensor:
    """DVL sensor that operates on raw state arrays."""

    def __init__(self, sensor_config: dict):
        self.sensor_type = 'DVL'
        self.rate = sensor_config.get('publish_rate', 10)
        self.location = np.array(sensor_config.get('sensor_location', [0, 0, 0]), dtype=float)

        self.vel_rms = np.array([0.05, 0.05, 0.05])
        self.vel_cov = np.diag(self.vel_rms ** 2)

        custom_cov = sensor_config.get('custom_covariance') or sensor_config.get('covariance') or {}
        if custom_cov and 'linear_velocity_covariance' in custom_cov:
            self.vel_cov = np.array(custom_cov['linear_velocity_covariance']).reshape(3, 3)

    def get_measurement(self, state: np.ndarray, state_der: np.ndarray,
                        use_quaternion: bool = False) -> dict:
        v_body = state[0:3]
        v_body_noisy = v_body + np.random.multivariate_normal(np.zeros(3), self.vel_cov)
        return {
            'velocity': v_body_noisy,
            'covariance': self.vel_cov.flatten(),
        }


class LocalEncoderSensor:
    """Encoder sensor that operates on raw state arrays."""

    def __init__(self, sensor_config: dict, n_control_surfaces: int, n_thrusters: int,
                 use_quaternion: bool = False):
        self.sensor_type = 'encoder'
        self.rate = sensor_config.get('publish_rate', 100)

        attitude_size = 4 if use_quaternion else 3
        base_idx = 9 + attitude_size
        total_actuators = n_control_surfaces + n_thrusters

        self.actuators = []
        if 'actuator_ids' in sensor_config:
            actuator_indices = sensor_config['actuator_ids']
            if not isinstance(actuator_indices, list):
                actuator_indices = [actuator_indices]
            actuator_indices = [a - 1 for a in actuator_indices]
            for act_idx in actuator_indices:
                if act_idx < 0 or act_idx >= total_actuators:
                    continue
                is_cs = act_idx < n_control_surfaces
                self.actuators.append({
                    'state_index': base_idx + act_idx,
                    'unit_conversion': 180.0 / np.pi if is_cs else 60.0,
                    'noise_rms': 1e-1,
                    'actuator_name': f'actuator_{act_idx}',
                    'encoder_cov': 1e-2,
                })
        elif 'actuator_type' in sensor_config and 'actuator_id' in sensor_config:
            act_type = sensor_config['actuator_type']
            act_id = sensor_config['actuator_id']
            if act_type in ('Rudder', 'Elevator', 'Aileron') and 1 <= act_id <= n_control_surfaces:
                self.actuators.append({
                    'state_index': base_idx + act_id - 1,
                    'unit_conversion': 180.0 / np.pi,
                    'noise_rms': 1e-1,
                    'actuator_name': f'actuator_{act_id - 1}',
                    'encoder_cov': 1e-2,
                })
            elif act_type == 'Thruster' and 1 <= act_id <= n_thrusters:
                self.actuators.append({
                    'state_index': base_idx + n_control_surfaces + act_id - 1,
                    'unit_conversion': 60.0,
                    'noise_rms': 1e-1,
                    'actuator_name': f'actuator_{n_control_surfaces + act_id - 1}',
                    'encoder_cov': 1e-2,
                })

    def get_measurement(self, state: np.ndarray, state_der: np.ndarray,
                        use_quaternion: bool = False) -> dict:
        actuator_values = []
        actuator_names = []
        covariances = []
        for act in self.actuators:
            if act['state_index'] < len(state):
                raw = state[act['state_index']]
                converted = raw * act['unit_conversion']
                noisy = converted + np.random.normal(0, act['noise_rms'])
                actuator_values.append(noisy)
                actuator_names.append(act['actuator_name'])
                covariances.append(act['encoder_cov'])
        return {
            'actuator_values': actuator_values,
            'actuator_names': actuator_names,
            'covariance': covariances,
        }


# ---------------------------------------------------------------------------
# Sensor generator orchestrator
# ---------------------------------------------------------------------------

_SENSOR_FACTORY = {
    'imu': lambda cfg, **kw: LocalIMUSensor(cfg, gravity=kw.get('gravity', 9.80665)),
    'gps': lambda cfg, **kw: LocalGPSSensor(cfg, gps_datum=kw.get('gps_datum', [0, 0, 0])),
    'dvl': lambda cfg, **kw: LocalDVLSensor(cfg),
    'encoder': lambda cfg, **kw: LocalEncoderSensor(
        cfg,
        n_control_surfaces=kw.get('n_control_surfaces', 0),
        n_thrusters=kw.get('n_thrusters', 0),
        use_quaternion=kw.get('use_quaternion', False),
    ),
}


def _extract_sensor_list(vessel_config: dict) -> list:
    """Navigate the nested sensors config structure to get the flat sensor list."""
    sensors = vessel_config.get('sensors')
    if sensors is None:
        return []
    if isinstance(sensors, list):
        return sensors
    if isinstance(sensors, dict):
        inner = sensors.get('sensors')
        if isinstance(inner, list):
            return inner
    return []


class LocalSensorGenerator:
    """
    Generates sensor measurements locally from vessel state/state_der data.

    Usage:
        gen = LocalSensorGenerator(vessel_config, use_quaternion=False)
        gen.start()
        # On each state update:
        gen.update_state(state_array, state_der_array)
        # Get latest measurements:
        gen.get_latest_measurements()  # -> dict of sensor_topic -> measurement
        gen.stop()
    """

    def __init__(self, vessel_config: dict, vessel_name: str = '',
                 use_quaternion: bool = False,
                 on_measurement: Optional[Callable] = None):
        """
        Args:
            vessel_config: Full agent config dict from handshake vesselConfig
            vessel_name: ROS vessel name for topic generation
            use_quaternion: Whether the state uses quaternion (4) or euler (3) attitude
            on_measurement: Optional callback(sensor_type, topic, measurement) called
                           each time a sensor generates a new measurement
        """
        self.vessel_name = vessel_name
        self.use_quaternion = use_quaternion
        self.on_measurement = on_measurement

        gravity = vessel_config.get('gravity', 9.80665)
        gps_datum = vessel_config.get('gps_datum', [0, 0, 0])

        cs_list = vessel_config.get('control_surfaces')
        if isinstance(cs_list, dict):
            cs_list = cs_list.get('control_surfaces', [])
        n_cs = len(cs_list) if cs_list else 0

        thr_list = vessel_config.get('thrusters')
        if isinstance(thr_list, dict):
            thr_list = thr_list.get('thrusters', [])
        n_thr = len(thr_list) if thr_list else 0

        factory_kwargs = dict(
            gravity=gravity,
            gps_datum=gps_datum,
            n_control_surfaces=n_cs,
            n_thrusters=n_thr,
            use_quaternion=use_quaternion,
        )

        self._sensors: List[Tuple[str, object, float]] = []  # (topic, sensor, period)
        sensor_configs = _extract_sensor_list(vessel_config)

        for i, scfg in enumerate(sensor_configs):
            stype = (scfg.get('sensor_type') or '').lower()
            if stype in ('camera', 'lidar'):
                continue
            if stype not in _SENSOR_FACTORY:
                logger.debug(f"Skipping unsupported sensor type: {stype}")
                continue

            sensor = _SENSOR_FACTORY[stype](scfg, **factory_kwargs)

            sensor_name = scfg.get('name', stype)
            sensor_id = scfg.get('sensor_id', i + 1)
            sid = str(sensor_id).zfill(2)
            topic = scfg.get('sensor_topic', f'/{vessel_name}/{sensor_name}_{sid}/data')

            period = 1.0 / max(1, sensor.rate)
            self._sensors.append((topic, sensor, period))
            logger.info(f"Local sensor: {stype} -> {topic} @ {sensor.rate} Hz")

        # Shared state buffers (updated atomically via lock)
        self._lock = threading.Lock()
        self._state: Optional[np.ndarray] = None
        self._state_der: Optional[np.ndarray] = None
        self._state_update_time: float = 0.0
        self._latest: Dict[str, dict] = {}
        self._timers: List[threading.Timer] = []
        self._running = False

    @property
    def sensor_count(self) -> int:
        return len(self._sensors)

    @property
    def sensor_topics(self) -> List[str]:
        return [topic for topic, _, _ in self._sensors]

    def update_state(self, state: np.ndarray, state_der: Optional[np.ndarray] = None):
        """Update the shared state arrays from incoming rosbridge data."""
        with self._lock:
            self._state = state.copy()
            if state_der is not None:
                self._state_der = state_der.copy()
            self._state_update_time = time.monotonic()

    def get_latest_measurements(self) -> Dict[str, dict]:
        """Return a copy of the latest sensor measurements keyed by topic."""
        with self._lock:
            return dict(self._latest)

    def start(self):
        """Start periodic sensor generation threads."""
        if self._running:
            return
        self._running = True
        for topic, sensor, period in self._sensors:
            self._start_sensor_timer(topic, sensor, period)
        if self._sensors:
            logger.info(f"LocalSensorGenerator started: {len(self._sensors)} sensor(s)")

    def stop(self):
        """Stop all sensor generation threads."""
        self._running = False
        for t in self._timers:
            t.cancel()
        self._timers.clear()
        logger.info("LocalSensorGenerator stopped")

    def _start_sensor_timer(self, topic: str, sensor, period: float):
        """Schedule a repeating timer for one sensor."""
        def tick():
            if not self._running:
                return
            self._generate_one(topic, sensor)
            if self._running:
                t = threading.Timer(period, tick)
                t.daemon = True
                t.start()
                self._timers.append(t)

        t = threading.Timer(period, tick)
        t.daemon = True
        t.start()
        self._timers.append(t)

    def _interpolate_state(self, state: np.ndarray, state_der: np.ndarray,
                           dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Dead-reckon the state forward by *dt* seconds using state_der.

        Uses the velocities already in the state and accelerations in state_der
        to linearly extrapolate so that sensor measurements taken between
        simulation steps see smoothly varying data rather than a stale snapshot.

        Returns (interpolated_state, state_der)  -- state_der is unchanged.
        """
        if dt <= 0.0 or dt > 1.0:
            return state, state_der

        s = state.copy()

        # Velocity: v(t) = v(t0) + a * dt
        s[0:6] = state[0:6] + state_der[0:6] * dt

        # Position: x(t) = x(t0) + v * dt + 0.5 * a * dt^2
        # Need NED velocity from body velocity for position update.
        # Use rotation matrix at t0 to transform body velocity -> NED.
        if self.use_quaternion:
            R = _quat_to_rotm(state[9:13])
        else:
            R = _eul_to_rotm(state[9:12])

        v_body = state[0:3]
        v_ned = R @ v_body
        # Approximate NED acceleration from body-frame acceleration
        a_body = state_der[0:3]
        a_ned = R @ a_body

        s[6:9] = state[6:9] + v_ned * dt + 0.5 * a_ned * dt * dt

        # Attitude: euler += omega * dt  (small-angle approximation,
        # good enough for the sub-timestep intervals we're bridging)
        omega = state[3:6]
        if self.use_quaternion:
            eul = _quat_to_eul(state[9:13])
            eul = eul + omega * dt
            s[9:13] = _eul_to_quat(eul)
        else:
            s[9:12] = state[9:12] + omega * dt

        # Actuator states (beyond attitude) are kept as-is
        return s, state_der

    def _generate_one(self, topic: str, sensor):
        """Generate a single sensor measurement, interpolating state if needed."""
        with self._lock:
            state = self._state
            state_der = self._state_der
            update_t = self._state_update_time

        if state is None:
            return
        if state_der is None:
            state_der = np.zeros_like(state)

        # Dead-reckon forward from last update to "now"
        dt = time.monotonic() - update_t
        interp_state, interp_der = self._interpolate_state(state, state_der, dt)

        try:
            measurement = sensor.get_measurement(
                interp_state, interp_der, use_quaternion=self.use_quaternion
            )
            with self._lock:
                self._latest[topic] = measurement
            if self.on_measurement:
                self.on_measurement(sensor.sensor_type, topic, measurement)
        except Exception as e:
            logger.debug(f"Sensor generation error for {topic}: {e}")

    def generate_once(self, state: np.ndarray,
                      state_der: Optional[np.ndarray] = None) -> Dict[str, dict]:
        """One-shot: generate all sensor measurements for the given state."""
        if state_der is None:
            state_der = np.zeros_like(state)
        results = {}
        for topic, sensor, _ in self._sensors:
            try:
                m = sensor.get_measurement(state, state_der, use_quaternion=self.use_quaternion)
                results[topic] = m
            except Exception as e:
                logger.debug(f"Sensor error for {topic}: {e}")
        return results
