#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kimm_robot_node.py

High-level robot node for the KIMM Stewart platform.

Responsibilities:
- Own the KIMM inverse kinematics library.
- Receive desired platform pose commands from Slicer/ROS2 on:
    /kimm/command/pose
- Convert platform pose commands into actuator joint commands.
- Publish actuator commands on:
    /kimm/command/joints
- Publish the assumed platform pose on:
    /kimm/state/pose
- Publish virtual 6-DOF platform joint state on:
    /joint_states

Important KIMM limitation:
- KIMM FK is not available. Therefore the platform pose published by this node
  is the last accepted/sent desired pose, not a measured FK-derived pose.

Safety split:
- This node does not own hardware enable/disable.
- The selected bridge node owns /kimm/bridge_enable and /kimm/bridge_status.
- This node also watches /kimm/bridge_status and avoids publishing new actuator
  commands while the bridge is known to be disabled. The bridge remains the
  final safety gate.
"""

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from kimm_control.conversions import (
    actuator_positions_to_joint_state,
    pose_dict_to_kimm_input,
    pose_dict_to_pose_stamped,
    pose_dict_to_virtual_joint_state,
    pose_stamped_to_pose_dict,
)
from kimm_control.kinematics import KimmKinematics


class KimmRobotNode(Node):
    def __init__(self):
        super().__init__("kimm_robot_node")

        # ---------------- Parameters ----------------
        self.declare_parameter("base_frame", "proximal_ring")
        self.declare_parameter("platform_frame", "distal_ring")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("require_bridge_enabled", True)

        self.declare_parameter("home_tx_m", 0.0)
        self.declare_parameter("home_ty_m", 0.0)
        self.declare_parameter("home_tz_m", 0.0)
        self.declare_parameter("home_rx_rad", 0.0)
        self.declare_parameter("home_ry_rad", 0.0)
        self.declare_parameter("home_rz_rad", 0.0)

        self.declare_parameter(
            "platform_joint_names",
            [
                "platform_x_joint",
                "platform_y_joint",
                "platform_z_joint",
                "platform_roll_joint",
                "platform_pitch_joint",
                "platform_yaw_joint",
            ],
        )

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

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.platform_frame = str(self.get_parameter("platform_frame").value)
        self.require_bridge_enabled = bool(
            self.get_parameter("require_bridge_enabled").value
        )

        self.platform_joint_names = list(
            self.get_parameter("platform_joint_names").value
        )
        self.actuator_joint_names = list(
            self.get_parameter("actuator_joint_names").value
        )

        self.home_pose = {
            "tx_m": float(self.get_parameter("home_tx_m").value),
            "ty_m": float(self.get_parameter("home_ty_m").value),
            "tz_m": float(self.get_parameter("home_tz_m").value),
            "rx_rad": float(self.get_parameter("home_rx_rad").value),
            "ry_rad": float(self.get_parameter("home_ry_rad").value),
            "rz_rad": float(self.get_parameter("home_rz_rad").value),
        }
        self.assumed_pose = dict(self.home_pose)
        self.bridge_enabled = False
        self.last_commanded_actuator_positions_rad = [0.0] * 6
        self.last_state_actuator_positions_rad = [0.0] * 6

        # ---------------- Kinematics ----------------
        package_share = Path(get_package_share_directory("kimm_control"))
        library_path = package_share / "lib" / "kimm_6rss" / "libKIMMSPKernel.so"

        self.kinematics = KimmKinematics(library_path)
        self.kinematics.configure()

        # Initialize actuator command cache at home.
        try:
            self.last_commanded_actuator_positions_rad = self._solve_ik(self.home_pose)
            self.last_state_actuator_positions_rad = list(
                self.last_commanded_actuator_positions_rad
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to initialize IK at home pose: {exc}")

        # ---------------- Publishers ----------------
        self.command_joints_pub = self.create_publisher(
            JointState,
            "/kimm/command/joints",
            10,
        )

        self.state_pose_pub = self.create_publisher(
            PoseStamped,
            "/kimm/state/pose",
            10,
        )

        self.joint_states_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        # ---------------- Subscriptions ----------------
        self.command_pose_sub = self.create_subscription(
            PoseStamped,
            "/kimm/command/pose",
            self.command_pose_callback,
            10,
        )

        self.bridge_status_sub = self.create_subscription(
            Bool,
            "/kimm/bridge_status",
            self.bridge_status_callback,
            10,
        )

        self.state_joints_sub = self.create_subscription(
            JointState,
            "/kimm/state/joints",
            self.state_joints_callback,
            10,
        )

        # ---------------- Services ----------------
        # This is a robot-level convenience service. Actual enable/disable is
        # owned by the selected bridge node through /kimm/bridge_enable.
        self.home_srv = self.create_service(
            Trigger,
            "/kimm/home",
            self.home_callback,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(
            1.0 / max(publish_rate_hz, 1e-6),
            self.publish_state,
        )

        self.get_logger().info("KIMM robot node is ready.")

    # ==================================================================
    # Kinematics / command helpers
    # ==================================================================
    def _solve_ik(self, pose):
        kimm_input = pose_dict_to_kimm_input(pose)
        actuator_positions_rad = self.kinematics.solve_ik(kimm_input)
        if len(actuator_positions_rad) != 6:
            raise RuntimeError(
                f"KIMM IK returned {len(actuator_positions_rad)} values; expected 6."
            )
        return [float(v) for v in actuator_positions_rad]

    def _publish_joint_command(self, actuator_positions_rad):
        stamp = self.get_clock().now().to_msg()
        cmd_msg = actuator_positions_to_joint_state(
            actuator_positions_rad,
            self.actuator_joint_names,
            stamp,
        )
        self.command_joints_pub.publish(cmd_msg)

    def _send_pose_command(self, pose, source="pose command"):
        if self.require_bridge_enabled and not self.bridge_enabled:
            raise RuntimeError(
                "Bridge is disabled. Enable /kimm/bridge_enable before sending commands."
            )

        actuator_positions_rad = self._solve_ik(pose)
        self._publish_joint_command(actuator_positions_rad)

        # No FK available: assume accepted command is the platform state.
        self.assumed_pose = dict(pose)
        self.last_commanded_actuator_positions_rad = list(actuator_positions_rad)

        self.get_logger().info(
            f"Published KIMM actuator command from {source}: "
            f"{[round(v, 4) for v in actuator_positions_rad]} rad"
        )

    # ==================================================================
    # ROS callbacks
    # ==================================================================
    def command_pose_callback(self, msg):
        try:
            pose = pose_stamped_to_pose_dict(msg)
            self._send_pose_command(pose, source="/kimm/command/pose")
        except Exception as exc:
            self.get_logger().error(f"Failed to process /kimm/command/pose: {exc}")

    def bridge_status_callback(self, msg):
        self.bridge_enabled = bool(msg.data)

    def state_joints_callback(self, msg):
        if len(msg.position) != 6:
            self.get_logger().warn(
                f"/kimm/state/joints expected 6 positions, got {len(msg.position)}"
            )
            return

        self.last_state_actuator_positions_rad = [float(v) for v in msg.position]

    def home_callback(self, request, response):
        try:
            self._send_pose_command(self.home_pose, source="/kimm/home")
            response.success = True
            response.message = "KIMM home command published."
        except Exception as exc:
            response.success = False
            response.message = f"Failed to publish KIMM home command: {exc}"

        return response

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()

        pose_msg = pose_dict_to_pose_stamped(
            self.assumed_pose,
            frame_id=self.base_frame,
            stamp=stamp,
        )
        self.state_pose_pub.publish(pose_msg)

        virtual_joint_state = pose_dict_to_virtual_joint_state(
            self.assumed_pose,
            self.platform_joint_names,
            stamp,
        )
        self.joint_states_pub.publish(virtual_joint_state)

    def destroy_node(self):
        try:
            self.kinematics.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KimmRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
