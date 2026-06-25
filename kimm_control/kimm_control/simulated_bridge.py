#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simulated_bridge.py

AIRS-style simulated bridge node for the KIMM robot.

Responsibilities:
- Own the final command gate for simulation.
- Provide /kimm/bridge_enable.
- Publish /kimm/bridge_status.
- Receive actuator joint commands on /kimm/command/joints.
- Publish simulated actuator joint state on /kimm/state/joints.

No IK/FK is done here. KIMM IK belongs to kimm_robot_node.py.

The internal methods configure/activate/deactivate/read/write/close are named
like a future ros2_control hardware interface, while this file remains a simple
ROS2 node for now.
"""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class SimulatedBridge(Node):
    def __init__(self):
        super().__init__("kimm_simulated_bridge")

        # ---------------- Parameters ----------------
        self.declare_parameter("auto_activate", False)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("deadband_rad", 1e-5)
        self.declare_parameter("tau_sec", 0.15)
        self.declare_parameter(
            "actuator_joint_names",
            [
                "actuator_101",
                "actuator_102",
                "actuator_103",
                "actuator_104",
                "actuator_105",
                "actuator_106",
            ],
        )

        self.auto_activate = bool(self.get_parameter("auto_activate").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.deadband_rad = float(self.get_parameter("deadband_rad").value)
        self.tau_sec = float(self.get_parameter("tau_sec").value)
        self.actuator_joint_names = list(
            self.get_parameter("actuator_joint_names").value
        )

        # ---------------- Backend state ----------------
        self.configured = False
        self.enabled = False
        self.command_positions_rad: List[float] = [0.0] * 6
        self.current_positions_rad: List[float] = [0.0] * 6
        self._last_accepted_command: Optional[List[float]] = None

        # ---------------- ROS interfaces ----------------
        self.state_joints_pub = self.create_publisher(
            JointState,
            "/kimm/state/joints",
            10,
        )
        self.bridge_status_pub = self.create_publisher(
            Bool,
            "/kimm/bridge_status",
            10,
        )

        self.command_joints_sub = self.create_subscription(
            JointState,
            "/kimm/command/joints",
            self.command_joints_callback,
            10,
        )

        self.bridge_enable_srv = self.create_service(
            SetBool,
            "/kimm/bridge_enable",
            self.bridge_enable_callback,
        )

        self.configure()
        if self.auto_activate:
            self.activate()

        self.dt = 1.0 / max(self.publish_rate_hz, 1e-6)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.publish_bridge_status_once()
        self.get_logger().info("KIMM simulated bridge is ready.")

    # ==================================================================
    # ros2_control-like backend methods
    # ==================================================================
    def configure(self):
        self.configured = True

    def activate(self):
        if not self.configured:
            self.configure()
        self.enabled = True

    def deactivate(self):
        self.enabled = False

    def read(self):
        return {
            "enabled": self.enabled,
            "actuator_positions_rad": list(self.current_positions_rad),
        }

    def write(self, joint_positions):
        if not self.enabled:
            raise RuntimeError("KIMM simulated bridge is disabled.")

        if len(joint_positions) != 6:
            raise ValueError(
                f"KIMM command requires 6 actuator positions, got {len(joint_positions)}."
            )

        cmd = [float(v) for v in joint_positions]

        if self._last_accepted_command is not None:
            if all(
                abs(cmd[i] - self._last_accepted_command[i]) <= self.deadband_rad
                for i in range(6)
            ):
                return self.read()

        self.command_positions_rad = list(cmd)
        self._last_accepted_command = list(cmd)
        return self.read()

    def close(self):
        self.deactivate()
        self.configured = False

    # ==================================================================
    # ROS callbacks
    # ==================================================================
    def command_joints_callback(self, msg):
        if len(msg.position) != 6:
            self.get_logger().warn(
                f"/kimm/command/joints expected 6 positions, got {len(msg.position)}"
            )
            return

        try:
            self.write(msg.position)
        except Exception as exc:
            self.get_logger().warn(f"Ignoring /kimm/command/joints: {exc}")

    def bridge_enable_callback(self, request, response):
        try:
            if request.data:
                self.activate()
                response.success = True
                response.message = "KIMM simulated bridge enabled."
            else:
                self.deactivate()
                response.success = True
                response.message = "KIMM simulated bridge disabled."

            self.publish_bridge_status_once()
        except Exception as exc:
            response.success = False
            response.message = f"Failed to set KIMM simulated bridge state: {exc}"
            self.get_logger().error(response.message)

        return response

    def timer_callback(self):
        self._simulate_motion_step()
        self.publish_state_once()
        self.publish_bridge_status_once()

    # ==================================================================
    # Helpers
    # ==================================================================
    def _simulate_motion_step(self):
        if self.tau_sec <= 1e-9:
            self.current_positions_rad = list(self.command_positions_rad)
            return

        alpha = min(1.0, self.dt / self.tau_sec)
        for i in range(6):
            self.current_positions_rad[i] += alpha * (
                self.command_positions_rad[i] - self.current_positions_rad[i]
            )

    def publish_state_once(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.actuator_joint_names)
        msg.position = list(self.current_positions_rad)
        self.state_joints_pub.publish(msg)

    def publish_bridge_status_once(self):
        msg = Bool()
        msg.data = bool(self.enabled)
        self.bridge_status_pub.publish(msg)

    def destroy_node(self):
        try:
            self.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
