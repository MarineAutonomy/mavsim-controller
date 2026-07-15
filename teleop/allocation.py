#!/usr/bin/env python3
"""
Thrust allocation for keyboard teleop (plans/plan_teleop.md)

Maps a normalized 6DOF command vector (surge, sway, heave, roll, pitch, yaw,
each in [-1, 1]) to per-actuator commands (`th_XX` RPM, `cs_XX` degrees),
using each vessel's own thruster/control-surface geometry - no new config
fields, just `thruster_location`/`thruster_orientation`/`control_surface_location`/
`control_surface_orientation` etc, already present in the vessel config schema
(web_platform/backend/app/models/config.py's ThrusterConfig/ControlSurfaceConfig).

Thrusters are allocated with a proper pseudo-inverse over the vessel's actual
geometry (however many it has), using a static "bollard" thrust approximation
(J~0, i.e. ignoring the vessel's own forward speed) - reasonable for teleop,
which is mostly about hovering/maneuvering rather than open-water transit.

Control surfaces (rudders/fins) are NOT allocated the same way: their real
force is proportional to the SQUARE of local flow velocity (see
ros2_ws/src/mavsim/mavsim/class_vessel.py's control-surface force model) and is
genuinely zero at zero vessel speed - a real rudder does nothing until there's
way on. So surfaces just get a simple proportional-deflection heuristic keyed
to whichever single DOF their geometry most influences; they deliberately do
NOT contribute to max_wrench's auto-scaling below (thrusters are the only
actuator this module treats as always-authoritative).
"""

import numpy as np

DOF_ORDER = ('X', 'Y', 'Z', 'K', 'M', 'N')  # surge, sway, heave, roll, pitch, yaw

_EPS_J = 1e-6           # small offset from the singular J=0 point (see _thruster_kt)
_DEFAULT_RHO = 1000.0   # kg/m^3; not propagated through the handshake, so this is
                        # an engineering default (matches the example vessel yaml's
                        # freshwater density; real seawater is ~1025)


def _eul_to_rotm(eul_deg):
    """Body<-local rotation matrix from [roll,pitch,yaw] in degrees.

    Verbatim reimplementation of the intrinsic ZYX convention in
    ros2_ws/src/mavsim/mavsim/module_kinematics.py's eul_to_rotm() (order='ZYX').
    Copied rather than imported because that module belongs to the `mavsim`
    ROS2 package, which the bridge container never builds (the Dockerfile only
    builds `interfaces` from ros2_ws/src/interfaces). test_allocation.py pins
    this against hardcoded reference matrices so the two copies can't drift
    apart silently.
    """
    phi, theta, psi = np.asarray(eul_deg, dtype=float) * np.pi / 180.0
    c1, s1 = np.cos(phi), np.sin(phi)
    c2, s2 = np.cos(theta), np.sin(theta)
    c3, s3 = np.cos(psi), np.sin(psi)
    return np.array([
        [c2 * c3, -c1 * s3 + s1 * s2 * c3, s1 * s3 + c1 * s2 * c3],
        [c2 * s3, c1 * c3 + s1 * s2 * s3, -s1 * c3 + c1 * s2 * s3],
        [-s2, s1 * c2, c1 * c2],
    ])


def _thruster_kt(j_vs_kt):
    """Return (KT_fwd, KT_rev): thrust coefficient just off either side of J=0.

    J_vs_KT tables in this codebase are allowed to encode a genuine
    discontinuity/sign-flip at J=0 (e.g. matsya_test.yaml's
    J_vs_KT=[[-1,0,0,1],[0,-1,1,0]], a duplicate breakpoint at J=0 with two
    different KT values either side). Querying at literal J=0 (or a signed
    -0.0/+0.0) does not reliably resolve which side of that duplicate
    breakpoint np.interp picks - both signs of RPM can numerically land on
    the same KT. Querying at a real, small +-eps instead lands strictly
    within each side's own interval, which correctly and stably recovers the
    intended forward/reverse thrust coefficients.
    """
    if not j_vs_kt or len(j_vs_kt) != 2 or len(j_vs_kt[0]) == 0:
        return 0.0, 0.0
    j_arr = np.asarray(j_vs_kt[0], dtype=float)
    kt_arr = np.asarray(j_vs_kt[1], dtype=float)
    kt_fwd = float(np.interp(_EPS_J, j_arr, kt_arr, left=0.0, right=0.0))
    kt_rev = float(np.interp(-_EPS_J, j_arr, kt_arr, left=0.0, right=0.0))
    return kt_fwd, kt_rev


class ThrustAllocator:
    """Builds a per-vessel thruster allocation matrix + control-surface DOF
    assignment once from vessel config, then maps normalized 6DOF commands to
    actuator commands via solve().
    """

    def __init__(self, thrusters, control_surfaces, rho=_DEFAULT_RHO):
        self.rho = float(rho)

        self._th_ids = []
        self._th_meta = {}
        columns = []

        for th in (thrusters or []):
            actuator_id = int(th.get('actuator_id', 0))
            name = f'th_{actuator_id:02d}'
            location = np.asarray(th.get('thruster_location', [0.0, 0.0, 0.0]), dtype=float)
            orientation = th.get('thruster_orientation', [0.0, 0.0, 0.0])
            diameter = float(th.get('D_prop', 0.1))
            n_max = float(th.get('n_max', 1000.0))
            n_min = float(th.get('n_min', -1000.0))
            kt_fwd, kt_rev = _thruster_kt(th.get('J_vs_KT'))

            rotm = _eul_to_rotm(orientation)
            unit_thrust = rotm @ np.array([1.0, 0.0, 0.0])
            moment = np.cross(location, unit_thrust)
            columns.append(np.concatenate([unit_thrust, moment]))

            self._th_ids.append(name)
            self._th_meta[name] = {
                'KT_fwd': kt_fwd, 'KT_rev': kt_rev,
                'D': diameter, 'n_min': n_min, 'n_max': n_max,
            }

        self.B = np.stack(columns, axis=1) if columns else np.zeros((6, 0))
        self.B_pinv = (
            np.linalg.pinv(self.B) if self.B.shape[1] > 0 else np.zeros((0, 6))
        )

        capacities = []
        for name in self._th_ids:
            meta = self._th_meta[name]
            t_fwd = abs(meta['KT_fwd']) * self.rho * meta['D'] ** 4 * meta['n_max'] ** 2
            t_rev = abs(meta['KT_rev']) * self.rho * meta['D'] ** 4 * meta['n_min'] ** 2
            capacities.append(max(t_fwd, t_rev))
        capacities = np.asarray(capacities, dtype=float)
        self.max_wrench = (
            np.abs(self.B) @ capacities if capacities.size else np.zeros(6)
        )
        self.dof_available = self.max_wrench > 1e-9

        self._cs_ids = []
        self._cs_meta = {}
        for cs in (control_surfaces or []):
            actuator_id = int(cs.get('actuator_id', 0))
            name = f'cs_{actuator_id:02d}'
            location = np.asarray(cs.get('control_surface_location', [0.0, 0.0, 0.0]), dtype=float)
            orientation = cs.get('control_surface_orientation', [0.0, 0.0, 0.0])
            delta_max = float(cs.get('control_surface_delta_max', 35.0))
            delta_min = float(cs.get('control_surface_delta_min', -35.0))

            rotm = _eul_to_rotm(orientation)
            lift_dir = rotm @ np.array([0.0, 1.0, 0.0])
            moment = np.cross(location, lift_dir)
            wrench_dir = np.concatenate([lift_dir, moment])
            dof_idx = int(np.argmax(np.abs(wrench_dir)))
            sign = float(np.sign(wrench_dir[dof_idx]))
            if sign == 0.0:
                sign = 1.0

            self._cs_ids.append(name)
            self._cs_meta[name] = {
                'dof_idx': dof_idx, 'sign': sign,
                'delta_max': delta_max, 'delta_min': delta_min,
            }

    def solve(self, cmd_normalized):
        """cmd_normalized: 6-vector in [-1,1], ordered per DOF_ORDER.

        Returns {'th_01': rpm, 'cs_02': deg, ...} ready to publish as an
        interfaces/Actuator message's actuator_names/actuator_values.
        """
        cmd = np.clip(np.asarray(cmd_normalized, dtype=float), -1.0, 1.0)
        if cmd.shape != (6,):
            raise ValueError(f"cmd_normalized must have shape (6,), got {cmd.shape}")

        result = {}

        if self.B.shape[1] > 0:
            wrench = cmd * self.max_wrench
            signed_thrust = self.B_pinv @ wrench
            for name, force in zip(self._th_ids, signed_thrust):
                meta = self._th_meta[name]
                kt = meta['KT_fwd'] if force >= 0 else meta['KT_rev']
                denom = abs(kt) * self.rho * meta['D'] ** 4
                n = np.sign(force) * np.sqrt(abs(force) / denom) if denom > 1e-12 else 0.0
                result[name] = float(np.clip(n, meta['n_min'], meta['n_max']))

        for name in self._cs_ids:
            meta = self._cs_meta[name]
            raw = cmd[meta['dof_idx']] * meta['sign'] * meta['delta_max']
            result[name] = float(np.clip(raw, meta['delta_min'], meta['delta_max']))

        return result
