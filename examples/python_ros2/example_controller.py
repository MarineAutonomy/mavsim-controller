#!/usr/bin/env python3
"""
Example mavsim user controller (Python / rclpy)
==============================================

This runs OUTSIDE the bridge container (natively on your host, or in a sibling
container) and talks to the bridge purely over local ROS2:

  subscribes:  /<vessel>/vessel_state    std_msgs/Float64MultiArray   (telemetry)
  publishes:   /<vessel>/actuator_cmd    interfaces/Actuator          (commands)

It is a simple proportional heading controller that steers the vessel toward a
target point. Replace `compute_command()` with your own logic.

Prerequisites:
  - ROS2 (e.g. Humble) sourced
  - the `interfaces` package built and sourced (see ../../interfaces/README.md)
  - the SAME ROS_DOMAIN_ID as the bridge

Run:
  export ROS_DOMAIN_ID=0          # must match the bridge
  python3 example_controller.py --vessel matsya_01 --cs cs_01 --th th_01

vessel_state layout (Float64MultiArray.data), euler mode:
  [t, u, v, w, p, q, r, x, y, z, phi, theta, psi, <actuators...>]
   0  1  2  3  4  5  6  7  8  9  10    11    12
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from interfaces.msg import Actuator


class ExampleController(Node):
    def __init__(self, vessel, cs_name, th_name, target, thrust, gain, quaternion):
        super().__init__("mavsim_example_controller")
        self.vessel = vessel
        self.cs_name = cs_name
        self.th_name = th_name
        self.target_x, self.target_y = target
        self.thrust = thrust
        self.gain = gain
        self.quaternion = quaternion

        self.pub = self.create_publisher(Actuator, f"/{vessel}/actuator_cmd", 10)
        self.sub = self.create_subscription(
            Float64MultiArray, f"/{vessel}/vessel_state", self._on_state, 10
        )
        self.get_logger().info(
            f"Controlling {vessel}: subscribe /{vessel}/vessel_state, "
            f"publish /{vessel}/actuator_cmd ({cs_name}, {th_name})"
        )

    def _on_state(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 13:
            return
        x, y = data[7], data[8]
        if self.quaternion:
            # [t,u,v,w,p,q,r,x,y,z, q0,q1,q2,q3, ...] -> yaw from quaternion
            q0, q1, q2, q3 = data[10], data[11], data[12], data[13]
            yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                             1.0 - 2.0 * (q2 * q2 + q3 * q3))
        else:
            yaw = data[12]  # psi

        cs_value, th_value = self.compute_command(x, y, yaw)
        self.publish_command(cs_value, th_value)

    def compute_command(self, x, y, yaw):
        """Proportional heading control toward (target_x, target_y)."""
        heading_error = math.atan2(self.target_y - y, self.target_x - x) - yaw
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        rudder_deg = max(-30.0, min(30.0, math.degrees(heading_error) * self.gain))
        return rudder_deg, self.thrust

    def publish_command(self, cs_value, th_value):
        msg = Actuator()
        msg.actuator_names = [self.cs_name, self.th_name]
        msg.actuator_values = [float(cs_value), float(th_value)]
        msg.covariance = [0.0, 0.0]
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="mavsim example ROS2 controller")
    parser.add_argument("--vessel", required=True, help="Vessel ros_name, e.g. matsya_01")
    parser.add_argument("--cs", default="cs_01", help="Control surface actuator name")
    parser.add_argument("--th", default="th_01", help="Thruster actuator name")
    parser.add_argument("--target", nargs=2, type=float, default=[100.0, 50.0],
                        metavar=("X", "Y"), help="Target point in NED (m)")
    parser.add_argument("--thrust", type=float, default=1500.0, help="Thruster RPM")
    parser.add_argument("--gain", type=float, default=1.0, help="Heading P gain")
    parser.add_argument("--quaternion", action="store_true",
                        help="Set if the simulation publishes quaternion attitude")
    args = parser.parse_args()

    rclpy.init()
    node = ExampleController(
        vessel=args.vessel,
        cs_name=args.cs,
        th_name=args.th,
        target=tuple(args.target),
        thrust=args.thrust,
        gain=args.gain,
        quaternion=args.quaternion,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
