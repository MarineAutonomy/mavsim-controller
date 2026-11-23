#!/usr/bin/env python3
"""
Bridge fixes from a mission test run (2026-11-23):

  - the accelerometer projects gravity with the true attitude, not the noisy
    reported one (which leaked ~g * attitude noise into accel x/y);
  - UWB, depth and compass sensors are generated (the configurator offered
    them, but neither the simulator nor the bridge produced them);
  - every sensor message with a header, and odometry_sim, is stamped with
    wall-clock time (they were unstamped, so an EKF could not order them);
  - vessel_state/odometry_sim are republished, and the sensor generators fed,
    on every vessel_state message rather than once per control tick (~5 Hz);
  - the control loop runs at a fixed rate (its REST round trip no longer
    stretches the period);
  - recording discovers topics under the vessel names, not the backend's
    simulation namespace, which local topics do not have ("No topics
    discovered after retries");
  - the bridge's rosbridge port comes from MAVSIM_ROSBRIDGE_PORT;
  - handshakes carry a controllerId that is stable across restarts.

No ROS needed: the ROS message modules are replaced by small stand-ins.

    python3 -m pytest core/tests/unit/test_mission_fixes.py
"""

import sys
import threading
import time
import types
from unittest.mock import Mock, patch

import numpy as np
import pytest

import base_controller
from base_controller import BaseController
from local_sensor_generator import (LocalCompassSensor, LocalDepthSensor, LocalIMUSensor,
                                    LocalSensorGenerator, LocalUWBSensor, _eul_to_quat)

G = 9.80665


def state(pos=(0, 0, 0), eul=(0, 0, 0), vel=(0, 0, 0, 0, 0, 0)):
    """Euler-mode state: [u v w p q r x y z phi theta psi]."""
    return np.array([*vel, *pos, *eul], dtype=float)


ZERO_DER = np.zeros(12)


# ---------------------------------------------------------------------------
# Sensor models
# ---------------------------------------------------------------------------

class TestIMUGravity:
    def imu(self, eul_rms):
        return LocalIMUSensor({'custom_covariance': {
            'orientation_covariance': list(np.diag([eul_rms ** 2] * 3).flatten()),
            'angular_velocity_covariance': [0.0] * 9,
            'linear_acceleration_covariance': [0.0] * 9,
        }})

    def test_attitude_noise_does_not_reach_the_accelerometer(self):
        np.random.seed(0)
        imu = self.imu(eul_rms=0.1)
        for _ in range(50):
            m = imu.get_measurement(state(), ZERO_DER)
            np.testing.assert_allclose(m['linear_acceleration'], [0, 0, -G], atol=1e-12)

    def test_gravity_follows_the_true_attitude(self):
        imu = self.imu(eul_rms=0.0)
        roll = 0.3
        m = imu.get_measurement(state(eul=(roll, 0, 0)), ZERO_DER)
        np.testing.assert_allclose(m['linear_acceleration'],
                                   [0, -G * np.sin(roll), -G * np.cos(roll)], atol=1e-12)

    def test_reported_orientation_still_has_noise(self):
        np.random.seed(1)
        imu = self.imu(eul_rms=0.1)
        m = imu.get_measurement(state(), ZERO_DER)
        assert not np.allclose(m['orientation'], [1, 0, 0, 0])


class TestNewSensors:
    def test_uwb_is_the_sensor_position_in_ned(self):
        uwb = LocalUWBSensor({'sensor_location': [1.0, 0.0, 0.0],
                              'covariance': {'position_covariance': [0.0] * 9}})
        m = uwb.get_measurement(state(pos=(10, 20, 0), eul=(0, 0, np.pi / 2)), ZERO_DER)
        np.testing.assert_allclose(m['position'], [10, 21, 0], atol=1e-12)
        cov = np.reshape(m['covariance'], (6, 6))
        np.testing.assert_allclose(np.diag(cov), [0, 0, 0, -1, -1, -1])

    def test_uwb_defaults_to_the_simulators_1m(self):
        cov = np.reshape(LocalUWBSensor({}).covariance, (6, 6))
        np.testing.assert_allclose(np.diag(cov), [1, 1, 1, -1, -1, -1])

    def test_depth_is_the_sensor_z_positive_down(self):
        depth = LocalDepthSensor({'sensor_location': [0.0, 0.0, 0.5],
                                  'covariance': {'depth_covariance': [0.0]}})
        m = depth.get_measurement(state(pos=(3, 4, 2)), ZERO_DER)
        assert m['depth'] == pytest.approx(2.5)
        np.testing.assert_allclose(m['position'], [0, 0, 2.5])
        cov = np.reshape(m['covariance'], (6, 6))
        np.testing.assert_allclose(np.diag(cov), [-1, -1, 0, -1, -1, -1])

    def test_depth_variance_from_the_configurator(self):
        depth = LocalDepthSensor({'covariance': {'depth_covariance': [0.04]}})
        assert depth.variance == pytest.approx(0.04)
        assert np.reshape(depth.covariance, (6, 6))[2, 2] == pytest.approx(0.04)

    def test_compass_heading_includes_mounting_and_wraps(self):
        compass = LocalCompassSensor({'sensor_orientation': [0, 0, 90],
                                      'covariance': {'heading_covariance': [0.0]}})
        m = compass.get_measurement(state(eul=(0, 0, 3.0)), ZERO_DER)
        expected = (3.0 + np.pi / 2 + np.pi) % (2 * np.pi) - np.pi     # wrapped to [-pi, pi)
        assert m['heading'] == pytest.approx(expected)
        np.testing.assert_allclose(m['orientation'], _eul_to_quat(np.array([0, 0, expected])))
        cov = np.reshape(m['covariance'], (6, 6))
        np.testing.assert_allclose(np.diag(cov), [-1, -1, -1, -1, -1, 0])

    def test_compass_ignores_roll_and_pitch(self):
        compass = LocalCompassSensor({'covariance': {'heading_covariance': [0.0]}})
        m = compass.get_measurement(state(eul=(0.4, -0.3, 1.2)), ZERO_DER)
        assert m['heading'] == pytest.approx(1.2)

    def test_generator_builds_them_from_the_vessel_config(self):
        config = {'sensors': {'sensors': [
            {'sensor_type': 'UWB', 'sensor_id': 1, 'name': 'uwb', 'publish_rate': 10},
            {'sensor_type': 'depth', 'sensor_id': 2, 'name': 'depth', 'publish_rate': 10},
            {'sensor_type': 'compass', 'sensor_id': 3, 'name': 'compass', 'publish_rate': 10},
        ]}}
        gen = LocalSensorGenerator(config, vessel_name='sookshma_01')
        assert gen.sensor_topics == ['/sookshma_01/uwb_01/data', '/sookshma_01/depth_02/data',
                                     '/sookshma_01/compass_03/data']


# ---------------------------------------------------------------------------
# ROS message stand-ins
# ---------------------------------------------------------------------------

class Auto:
    """A message whose nested fields spring into existence on first access."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        value = Auto()
        setattr(self, name, value)
        return value


class NavSatStatus(Auto):
    STATUS_FIX = 0
    SERVICE_GPS = 1


class Time(Auto):
    pass


@pytest.fixture
def fake_ros(monkeypatch):
    names = {
        'std_msgs.msg': ['Float64MultiArray', 'UInt32'],
        'sensor_msgs.msg': ['Imu', 'NavSatFix'],
        'geometry_msgs.msg': ['Quaternion', 'Vector3', 'Point', 'TwistWithCovarianceStamped',
                              'PoseWithCovarianceStamped'],
        'nav_msgs.msg': ['Odometry'],
        'interfaces.msg': ['WaveProbe'],
        'builtin_interfaces.msg': [],
    }
    for mod, classes in names.items():
        pkg = mod.split('.')[0]
        package = types.ModuleType(pkg)
        module = types.ModuleType(mod)
        for c in classes:
            setattr(module, c, type(c, (Auto,), {}))
        package.msg = module
        monkeypatch.setitem(sys.modules, pkg, package)
        monkeypatch.setitem(sys.modules, mod, module)
    sys.modules['sensor_msgs.msg'].NavSatStatus = NavSatStatus
    sys.modules['builtin_interfaces.msg'].Time = Time


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


MEASUREMENTS = {
    'imu': {'orientation': [1, 0, 0, 0], 'angular_velocity': [0, 0, 0],
            'linear_acceleration': [0, 0, -G], 'orientation_covariance': [0] * 9,
            'angular_velocity_covariance': [0] * 9, 'linear_acceleration_covariance': [0] * 9},
    'gps': {'latitude': 12.9, 'longitude': 80.2, 'altitude': 0.0, 'position_covariance': [0] * 9},
    'dvl': {'velocity': [1, 0, 0], 'covariance': [0] * 9},
    'waveprobe': {'elevation': 0.1, 'x': 0.0, 'y': 0.0},
}


class TestStamps:
    @pytest.mark.parametrize('stype', ['imu', 'gps', 'dvl', 'waveprobe'])
    def test_sensor_messages_are_stamped_with_wall_clock_time(self, fake_ros, stype):
        before = time.time()
        msg = BaseController._build_sensor_msg(stype, MEASUREMENTS[stype], node=None,
                                               frame_id='sookshma_01/x_01')
        assert before - 0.01 <= stamp_seconds(msg.header.stamp) <= time.time() + 0.01

    @pytest.mark.parametrize('stype', ['imu', 'gps', 'dvl'])
    def test_body_mounted_sensors_carry_their_frame(self, fake_ros, stype):
        msg = BaseController._build_sensor_msg(stype, MEASUREMENTS[stype], frame_id='sookshma_01/imu_01')
        assert msg.header.frame_id == 'sookshma_01/imu_01'

    @pytest.mark.parametrize('cls,cfg', [(LocalUWBSensor, {}), (LocalDepthSensor, {}),
                                         (LocalCompassSensor, {})])
    def test_pose_sensors_are_stamped_poses_in_ned(self, fake_ros, cls, cfg):
        sensor = cls(cfg)
        m = sensor.get_measurement(state(pos=(1, 2, 3)), ZERO_DER)
        msg = BaseController._build_sensor_msg(sensor.sensor_type, m)
        assert msg.header.frame_id == 'NED'
        assert stamp_seconds(msg.header.stamp) == pytest.approx(time.time(), abs=1.0)
        assert len(msg.pose.covariance) == 36

    def test_node_clock_is_used_when_there_is_a_node(self, fake_ros):
        node = Mock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = 'node-stamp'
        msg = BaseController._build_sensor_msg('imu', MEASUREMENTS['imu'], node=node)
        assert msg.header.stamp == 'node-stamp'

    def test_frame_id_from_topic(self):
        assert base_controller._sensor_frame_id('/sookshma_01/imu_01/data') == 'sookshma_01/imu_01'
        assert base_controller._sensor_frame_id('/custom/topic') == 'custom/topic'

    def test_odometry_is_stamped(self, fake_ros):
        pub = Mock()
        BaseController._publish_local_odometry(pub, state(pos=(1, 2, 3)), False, stamp='t0')
        msg = pub.publish.call_args.args[0]
        assert msg.header.stamp == 't0'
        assert msg.header.frame_id == 'NED'


# ---------------------------------------------------------------------------
# State fan-out and loop timing
# ---------------------------------------------------------------------------

class FakeCtrl:
    def __init__(self):
        self.state_cbs, self.der_cbs = [], []
        self.use_quaternion = False

    def add_state_callback(self, cb):
        self.state_cbs.append(cb)

    def add_state_der_callback(self, cb):
        self.der_cbs.append(cb)

    def get_raw_state_der(self, name):
        return np.zeros(12)


class TestStateFanout:
    def wired(self):
        c = BaseController(controller_code='TEST123')
        ctrl = FakeCtrl()
        c._controllers = {'sookshma_01': ctrl}
        c._controlled_vessels = ['sookshma_01']
        c._ros2_republishers = {k: Mock() for k in
                                ('sookshma_01/vessel_state', 'sookshma_01/vessel_state_der',
                                 'sookshma_01/odometry_sim')}
        gen = Mock()
        c._sensor_generators = {'sookshma_01': gen}
        c._wire_state_fanout()
        return c, ctrl, gen

    def test_every_state_message_is_republished_and_feeds_the_sensors(self, fake_ros):
        c, ctrl, gen = self.wired()
        for i in range(5):
            for cb in ctrl.state_cbs:
                cb('sookshma_01', [0.1 * i, *state(pos=(i, 0, 0))])
        assert c._ros2_republishers['sookshma_01/vessel_state'].publish.call_count == 5
        assert c._ros2_republishers['sookshma_01/odometry_sim'].publish.call_count == 5
        assert gen.update_state.call_count == 5
        args, kwargs = gen.update_state.call_args
        assert args[0][6] == 4 and kwargs['sim_time'] == pytest.approx(0.4)
        odom = c._ros2_republishers['sookshma_01/odometry_sim'].publish.call_args.args[0]
        assert stamp_seconds(odom.header.stamp) == pytest.approx(time.time(), abs=1.0)

    def test_other_vessels_states_are_ignored(self, fake_ros):
        c, ctrl, gen = self.wired()
        for cb in ctrl.state_cbs:
            cb('other_02', [0.0, *state()])
        gen.update_state.assert_not_called()
        c._ros2_republishers['sookshma_01/vessel_state'].publish.assert_not_called()

    def test_state_der_is_republished_without_its_time(self, fake_ros):
        c, ctrl, _ = self.wired()
        for cb in ctrl.der_cbs:
            cb('sookshma_01', [1.5, 0.1, 0.2])
        msg = c._ros2_republishers['sookshma_01/vessel_state_der'].publish.call_args.args[0]
        assert msg.data == [0.1, 0.2]

    def test_subscriber_calls_state_callbacks(self):
        from python_controller import RoslibSubscriber
        sub = RoslibSubscriber('ws://localhost:9090')
        seen = []
        sub.add_state_callback(lambda name, data: seen.append((name, data)))
        sub._use_quaternion = False
        sub._on_vessel_state({'data': [2.0, *state(pos=(1, 2, 3))]}, 'sookshma_01')
        assert seen and seen[0][0] == 'sookshma_01' and seen[0][1][0] == 2.0
        assert sub.get_raw_state_time('sookshma_01') == 2.0


class TestFixedRateLoop:
    def test_work_in_the_tick_does_not_stretch_the_period(self):
        """20 Hz with 30 ms of work per tick: a plain sleep(period) after the
        work gave 1 / 80 ms = 12.5 Hz."""
        class Busy(BaseController):
            ticks = 0

            def control_loop(self, vessel_states):
                Busy.ticks += 1
                time.sleep(0.03)
                return None

        c = Busy(controller_code='TEST123')
        ctrl = Mock()
        ctrl.session_ended = False
        ctrl.get_vessel_states.return_value = {'sookshma_01': object()}
        c._controller = ctrl
        c._controllers = {'sookshma_01': ctrl}
        c._control_rate_hz = 20.0
        c._running = True
        t = threading.Thread(target=c._control_loop, daemon=True)
        t.start()
        time.sleep(1.0)
        c._running = False
        t.join(1.0)
        assert Busy.ticks >= 17

    def test_overrun_does_not_burst(self):
        with patch.object(base_controller.time, 'monotonic', return_value=10.0), \
             patch.object(base_controller.time, 'sleep') as sleep:
            # Fell 0.5 s behind a 0.1 s period: next tick one period from now.
            assert BaseController._wait_for_tick(9.5, 0.1) == pytest.approx(10.1)
            sleep.assert_not_called()
            assert BaseController._wait_for_tick(10.05, 0.1) == pytest.approx(10.15)
            sleep.assert_called_once_with(pytest.approx(0.05))


# ---------------------------------------------------------------------------
# Recording, rosbridge port, controller id
# ---------------------------------------------------------------------------

class TestRecordingPrefixes:
    def start(self, vessels, namespace):
        c = BaseController(controller_code='TEST123')
        c._api_client = Mock()
        c._controller = Mock()
        c._controller.subscriber.is_connected.return_value = True
        c._controlled_vessels = vessels
        service = Mock()
        service.start_recording.return_value = True
        with patch('recording_service.RecordingService', return_value=service), \
             patch.object(c, '_report_recording_status'):
            c.start_recording(topics=None, namespace=namespace)
        return service.start_recording.call_args.kwargs

    def test_uses_vessel_names_not_the_simulation_namespace(self):
        kw = self.start(['sookshma_01'], '/sim_abc123/')
        assert kw['namespace'] == '/sookshma_01'
        assert kw['namespaces'] == ['/sookshma_01', '/sim_reset_event']

    def test_every_vessel_and_reset_events(self):
        kw = self.start(['sookshma_01', 'sookshma_02'], '/sim_abc123/')
        assert kw['namespaces'] == ['/sookshma_01', '/sookshma_02', '/sim_reset_event']


class TestRosbridgePort:
    def parse(self, monkeypatch, env):
        if env is None:
            monkeypatch.delenv('MAVSIM_ROSBRIDGE_PORT', raising=False)
        else:
            monkeypatch.setenv('MAVSIM_ROSBRIDGE_PORT', env)
        monkeypatch.setattr(sys, 'argv', ['base_controller.py', '--code', 'ABC'])
        return base_controller.create_controller_from_args().rosbridge_port

    def test_default(self, monkeypatch):
        assert self.parse(monkeypatch, None) == 9090

    def test_from_environment(self, monkeypatch):
        assert self.parse(monkeypatch, '9190') == 9190


class TestControllerId:
    def test_stable_and_hashed(self, monkeypatch):
        monkeypatch.delenv('MAVSIM_CONTROLLER_ID', raising=False)
        monkeypatch.setenv('MAVSIM_HOST_ID', 'machine-abc')
        a = base_controller._stable_controller_id()
        assert a == base_controller._stable_controller_id()
        assert a.startswith('bridge-') and 'machine-abc' not in a
        monkeypatch.setenv('MAVSIM_HOST_ID', 'machine-xyz')
        assert base_controller._stable_controller_id() != a

    def test_override(self, monkeypatch):
        monkeypatch.setenv('MAVSIM_CONTROLLER_ID', 'lab-pc-1')
        assert base_controller._stable_controller_id() == 'lab-pc-1'

    def test_sent_with_the_handshake(self, monkeypatch):
        monkeypatch.setenv('MAVSIM_CONTROLLER_ID', 'lab-pc-1')
        c = BaseController(controller_code='TEST123')
        with patch.object(base_controller.MavsimAPIClient, 'from_handshake',
                          side_effect=RuntimeError('409 already controlled')) as hs:
            assert c._connect_single_vessel('sookshma_01') is False
        assert hs.call_args.kwargs['controller_id'] == 'lab-pc-1'
