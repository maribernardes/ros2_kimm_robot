#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hardware_bridge.py

Hardware bridge node for the KIMM Stewart platform.

Responsibilities:
- Own the final hardware command gate.
- Provide /kimm/bridge_enable.
- Publish /kimm/bridge_status.
- Receive actuator joint commands on /kimm/command/joints.
- Convert actuator radians to Dynamixel encoder counts.
- Send accepted commands to the Dynamixel motors.
- Read Dynamixel present positions and publish /kimm/state/joints.

No IK/FK is done here.
KIMM IK belongs to kimm_robot_node.py.
"""

from typing import List, Optional
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from dynamixel_sdk import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)


class HardwareBridge(Node):
    def __init__(self):
        super().__init__("kimm_hardware_bridge")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("auto_activate", False)
        self.declare_parameter("zero_on_activate", True)
        self.declare_parameter(
            "zero_position_counts", [2048, 2048, 2048, 2048, 2048, 2048]
        )
        self.declare_parameter("zero_after_torque_delay_sec", 0.10)

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("deadband_rad", 1e-5)

        self.declare_parameter("hardware_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("protocol_version", 2.0)

        self.declare_parameter("dxl_ids", [101, 102, 103, 104, 105, 106])

        self.declare_parameter("dxl_minimum_position_value", 0)
        self.declare_parameter("dxl_maximum_position_value", 4095)
        self.declare_parameter("dxl_center", 2048)

        self.declare_parameter("dxl_odd_min", 1707)
        self.declare_parameter("dxl_odd_max", 2389)
        self.declare_parameter("dxl_even_min", 1707)
        self.declare_parameter("dxl_even_max", 2389)

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
        self.zero_on_activate = bool(self.get_parameter("zero_on_activate").value)
        self.zero_position_counts = [
            int(v) for v in self.get_parameter("zero_position_counts").value
        ]
        self.zero_after_torque_delay_sec = float(
            self.get_parameter("zero_after_torque_delay_sec").value
        )

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.deadband_rad = float(self.get_parameter("deadband_rad").value)

        self.hardware_port = str(self.get_parameter("hardware_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.protocol_version = float(self.get_parameter("protocol_version").value)

        self.dxl_ids = [int(v) for v in self.get_parameter("dxl_ids").value]

        self.dxl_minimum_position_value = int(
            self.get_parameter("dxl_minimum_position_value").value
        )
        self.dxl_maximum_position_value = int(
            self.get_parameter("dxl_maximum_position_value").value
        )
        self.dxl_center = int(self.get_parameter("dxl_center").value)

        self.dxl_odd_min = int(self.get_parameter("dxl_odd_min").value)
        self.dxl_odd_max = int(self.get_parameter("dxl_odd_max").value)
        self.dxl_even_min = int(self.get_parameter("dxl_even_min").value)
        self.dxl_even_max = int(self.get_parameter("dxl_even_max").value)

        self.actuator_joint_names = list(
            self.get_parameter("actuator_joint_names").value
        )

        if len(self.dxl_ids) != 6:
            raise RuntimeError(
                f"KIMM requires 6 Dynamixel IDs, got {len(self.dxl_ids)}"
            )

        if len(self.actuator_joint_names) != 6:
            raise RuntimeError(
                "KIMM requires 6 actuator joint names, got "
                f"{len(self.actuator_joint_names)}"
            )

        if len(self.zero_position_counts) != 6:
            raise RuntimeError(
                "KIMM zero_position_counts must contain 6 values, got "
                f"{len(self.zero_position_counts)}"
            )

        # ------------------------------------------------------------------
        # Dynamixel control table constants
        # X-series, Protocol 2.0
        # ------------------------------------------------------------------
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.LEN_GOAL_POSITION = 4
        self.ADDR_PRESENT_POSITION = 132
        self.LEN_PRESENT_POSITION = 4

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # ------------------------------------------------------------------
        # Backend state
        # ------------------------------------------------------------------
        self.configured = False
        self.enabled = False

        self.command_positions_rad: List[float] = [0.0] * 6
        self.current_positions_rad: List[float] = [0.0] * 6
        self._last_accepted_command: Optional[List[float]] = None

        self.port_handler = None
        self.packet_handler = None
        self.group_sync_write = None
        self.group_sync_read = None

        self._last_read_warning_time_sec = 0.0

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
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

        self.get_logger().info(
            "KIMM hardware bridge startup parameters: "
            f"auto_activate={self.auto_activate}, "
            f"zero_on_activate={self.zero_on_activate}, "
            f"zero_position_counts={self.zero_position_counts}"
        )

        if self.auto_activate:
            self.activate()

        period = 1.0 / max(self.publish_rate_hz, 1e-6)
        self.timer = self.create_timer(period, self.timer_callback)

        self.publish_bridge_status_once()

        self.get_logger().info(
            f"KIMM hardware bridge is ready on {self.hardware_port}. "
            "Commands are sent to hardware only after /kimm/bridge_enable is true."
        )

    # ======================================================================
    # Dynamixel helpers
    # ======================================================================
    def _check_comm(self, dxl_id, dxl_comm_result, dxl_error, action):
        if dxl_comm_result != COMM_SUCCESS:
            message = self.packet_handler.getTxRxResult(dxl_comm_result)
            raise RuntimeError(f"DXL ID {dxl_id}: failed to {action}: {message}")

        if dxl_error != 0:
            message = self.packet_handler.getRxPacketError(dxl_error)
            raise RuntimeError(f"DXL ID {dxl_id}: error during {action}: {message}")

    def _open_port(self):
        self.port_handler = PortHandler(self.hardware_port)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port: {self.hardware_port}")

        self.packet_handler = PacketHandler(self.protocol_version)

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(
                f"Failed to set Dynamixel baudrate {self.baudrate} "
                f"on {self.hardware_port}"
            )

        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            self.ADDR_GOAL_POSITION,
            self.LEN_GOAL_POSITION,
        )

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.ADDR_PRESENT_POSITION,
            self.LEN_PRESENT_POSITION,
        )

        for dxl_id in self.dxl_ids:
            ok = self.group_sync_read.addParam(dxl_id)
            if not ok:
                raise RuntimeError(f"DXL ID {dxl_id}: GroupSyncRead addParam failed")

    def _ping_motors(self):
        """Ping all configured Dynamixel IDs and report which ones respond."""

        responsive_ids = []

        for dxl_id in self.dxl_ids:
            try:
                model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(
                    self.port_handler,
                    dxl_id,
                )

                if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                    responsive_ids.append(dxl_id)
                    self.get_logger().info(
                        f"DXL ID {dxl_id} responded. Model number: {model_number}"
                    )
                else:
                    result_msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                    error_msg = self.packet_handler.getRxPacketError(dxl_error)
                    self.get_logger().warn(
                        f"DXL ID {dxl_id} did not respond. "
                        f"Result: {result_msg}; Error: {error_msg}"
                    )

            except Exception as exc:
                self.get_logger().warn(f"DXL ID {dxl_id} ping failed: {exc}")

        if len(responsive_ids) == 0:
            raise RuntimeError(
                "No Dynamixel motors responded to ping. "
                "Check motor power, U2D2 cable, baudrate, protocol version, "
                "and motor IDs."
            )

        if len(responsive_ids) != len(self.dxl_ids):
            self.get_logger().warn(
                f"Only {len(responsive_ids)}/{len(self.dxl_ids)} motors responded: "
                f"{responsive_ids}"
            )
        else:
            self.get_logger().info(
                f"All configured Dynamixel motors responded: {responsive_ids}"
            )

        return responsive_ids

    def _enable_torque(self):
        for dxl_id in self.dxl_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_ENABLE,
            )
            self._check_comm(dxl_id, dxl_comm_result, dxl_error, "enable torque")

    def _disable_torque(self):
        if self.packet_handler is None or self.port_handler is None:
            return

        for dxl_id in self.dxl_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_DISABLE,
            )

            try:
                self._check_comm(dxl_id, dxl_comm_result, dxl_error, "disable torque")
            except Exception as exc:
                self.get_logger().warn(str(exc))

    def _write_encoder_counts(self, position_counts):
        if len(position_counts) != 6:
            raise ValueError(
                f"KIMM encoder command requires 6 counts, got {len(position_counts)}."
            )

        for dxl_id, count in zip(self.dxl_ids, position_counts):
            count = int(count)

            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(count)),
                DXL_HIBYTE(DXL_LOWORD(count)),
                DXL_LOBYTE(DXL_HIWORD(count)),
                DXL_HIBYTE(DXL_HIWORD(count)),
            ]

            ok = self.group_sync_write.addParam(dxl_id, param_goal_position)
            if not ok:
                self.group_sync_write.clearParam()
                raise RuntimeError(f"DXL ID {dxl_id}: GroupSyncWrite addParam failed")

        dxl_comm_result = self.group_sync_write.txPacket()
        self.group_sync_write.clearParam()

        if dxl_comm_result != COMM_SUCCESS:
            message = self.packet_handler.getTxRxResult(dxl_comm_result)
            raise RuntimeError(f"GroupSyncWrite txPacket failed: {message}")

    def _read_encoder_counts(self):
        dxl_comm_result = self.group_sync_read.txRxPacket()

        if dxl_comm_result != COMM_SUCCESS:
            message = self.packet_handler.getTxRxResult(dxl_comm_result)
            raise RuntimeError(f"GroupSyncRead txRxPacket failed: {message}")

        position_counts = []

        for dxl_id in self.dxl_ids:
            available = self.group_sync_read.isAvailable(
                dxl_id,
                self.ADDR_PRESENT_POSITION,
                self.LEN_PRESENT_POSITION,
            )

            if not available:
                raise RuntimeError(
                    f"DXL ID {dxl_id}: present position data not available"
                )

            count = self.group_sync_read.getData(
                dxl_id,
                self.ADDR_PRESENT_POSITION,
                self.LEN_PRESENT_POSITION,
            )

            position_counts.append(int(count))

        return position_counts

    # ======================================================================
    # Radians/counts conversion
    # ======================================================================
    def radians_to_encoder_counts(self, actuator_positions_rad):
        """Convert six actuator angles in radians to Dynamixel encoder counts.

        Convention inherited from sp_control.py:
        - Python index 0,2,4: count = center + angle
        - Python index 1,3,5: count = center - angle
        """

        enc_res = 4096.0 / 360.0
        position_counts = []

        for idx, angle_rad in enumerate(actuator_positions_rad):
            angle_deg = math.degrees(float(angle_rad))

            if idx % 2 == 0:
                count = int(round(self.dxl_center + enc_res * angle_deg))
                count = max(self.dxl_odd_min, min(self.dxl_odd_max, count))
            else:
                count = int(round(self.dxl_center - enc_res * angle_deg))
                count = max(self.dxl_even_min, min(self.dxl_even_max, count))

            count = max(
                self.dxl_minimum_position_value,
                min(self.dxl_maximum_position_value, count),
            )

            position_counts.append(count)

        return position_counts

    def encoder_counts_to_radians(self, position_counts):
        """Convert Dynamixel encoder counts to six actuator angles in radians."""

        enc_res_deg = 360.0 / 4096.0
        actuator_positions_rad = []

        for idx, count in enumerate(position_counts):
            count = int(count)

            if idx % 2 == 0:
                angle_deg = (count - self.dxl_center) * enc_res_deg
            else:
                angle_deg = (self.dxl_center - count) * enc_res_deg

            actuator_positions_rad.append(math.radians(angle_deg))

        return actuator_positions_rad

    # ======================================================================
    # ros2_control-like lifecycle methods
    # ======================================================================
    def configure(self):
        """Open the Dynamixel port and prepare group read/write.

        This does not enable torque.
        """

        if self.configured:
            return

        self._open_port()
        self._ping_motors()

        self.configured = True

        try:
            counts = self._read_encoder_counts()
            self.current_positions_rad = self.encoder_counts_to_radians(counts)
        except Exception as exc:
            self.get_logger().warn(
                f"Hardware configured, but initial state read failed: {exc}"
            )

        self.get_logger().info("KIMM hardware bridge configured.")

    def activate(self):
        """Enable Dynamixel torque and accept commands."""

        if not self.configured:
            self.configure()

        self._enable_torque()
        self.enabled = True

        self.get_logger().warn("KIMM hardware bridge ENABLED. Robot can move.")

        if self.zero_on_activate:
            if self.zero_after_torque_delay_sec > 0.0:
                time.sleep(self.zero_after_torque_delay_sec)

            self.get_logger().warn(
                "Commanding KIMM actuators to zero encoder center counts: "
                f"{self.zero_position_counts}"
            )

            self._write_encoder_counts(self.zero_position_counts)

            self.command_positions_rad = [0.0] * 6
            self.current_positions_rad = [0.0] * 6
            self._last_accepted_command = [0.0] * 6

            try:
                self.read()
            except Exception as exc:
                self.get_logger().warn(
                    f"Zero command was sent, but readback failed: {exc}"
                )
        else:
            self.get_logger().info(
                "zero_on_activate is false; not commanding actuator zero positions."
            )

    def deactivate(self):
        """Disable command acceptance and Dynamixel torque."""

        self.enabled = False
        self._disable_torque()

        self.get_logger().info("KIMM hardware bridge disabled.")

    def read(self):
        """Read Dynamixel present positions."""

        if not self.configured:
            return {
                "enabled": self.enabled,
                "actuator_positions_rad": list(self.current_positions_rad),
            }

        try:
            position_counts = self._read_encoder_counts()
            self.current_positions_rad = self.encoder_counts_to_radians(position_counts)
        except Exception as exc:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec - self._last_read_warning_time_sec > 2.0:
                self.get_logger().warn(f"Failed to read KIMM hardware state: {exc}")
                self._last_read_warning_time_sec = now_sec

        return {
            "enabled": self.enabled,
            "actuator_positions_rad": list(self.current_positions_rad),
        }

    def write(self, joint_positions):
        """Send actuator joint command to Dynamixel motors."""

        if not self.enabled:
            raise RuntimeError("KIMM hardware bridge is disabled.")

        if not self.configured:
            raise RuntimeError("KIMM hardware bridge is not configured.")

        if len(joint_positions) != 6:
            raise ValueError(
                f"KIMM command requires 6 actuator positions, got {len(joint_positions)}."
            )

        cmd = [float(v) for v in joint_positions]

        if self._last_accepted_command is not None:
            within_deadband = all(
                abs(cmd[i] - self._last_accepted_command[i]) <= self.deadband_rad
                for i in range(6)
            )

            if within_deadband:
                return self.read()

        position_counts = self.radians_to_encoder_counts(cmd)
        self.get_logger().info(
            f"Writing KIMM actuator command counts: {position_counts}"
        )

        self._write_encoder_counts(position_counts)

        self.command_positions_rad = list(cmd)
        self._last_accepted_command = list(cmd)

        return self.read()

    def close(self):
        """Disable torque and close the Dynamixel port."""

        try:
            if self.enabled:
                self.deactivate()
            else:
                self._disable_torque()
        finally:
            if self.group_sync_read is not None:
                try:
                    self.group_sync_read.clearParam()
                except Exception:
                    pass

            if self.group_sync_write is not None:
                try:
                    self.group_sync_write.clearParam()
                except Exception:
                    pass

            if self.port_handler is not None:
                try:
                    self.port_handler.closePort()
                except Exception as exc:
                    try:
                        self.get_logger().warn(f"Failed to close Dynamixel port: {exc}")
                    except Exception:
                        pass

            self.port_handler = None
            self.packet_handler = None
            self.group_sync_write = None
            self.group_sync_read = None
            self.configured = False
            self.enabled = False

    # ======================================================================
    # ROS callbacks
    # ======================================================================
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
                response.message = "KIMM hardware bridge enabled."
            else:
                self.deactivate()
                response.success = True
                response.message = "KIMM hardware bridge disabled."

            self.publish_bridge_status_once()

        except Exception as exc:
            response.success = False
            response.message = f"Failed to set KIMM hardware bridge state: {exc}"
            self.get_logger().error(response.message)

        return response

    def timer_callback(self):
        self.read()
        self.publish_state_once()
        self.publish_bridge_status_once()

    # ======================================================================
    # Publishers
    # ======================================================================
    def publish_state_once(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.actuator_joint_names)
        msg.position = list(self.current_positions_rad)
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)

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
    node = HardwareBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
