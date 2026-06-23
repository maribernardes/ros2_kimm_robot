from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from kimm_control.conversions import (
    actuator_positions_to_joint_state,
    pose_dict_to_pose_stamped,
    pose_dict_to_virtual_joint_state,
    pose_stamped_to_pose_dict,
)
from kimm_control.kinematics import KimmKinematics
from kimm_control.simulated_bridge import SimulatedBridge


class KimmRobotNode(Node):
    def __init__(self):
        super().__init__("kimm_robot_node")

        self.declare_parameter("sim", True)

        self.declare_parameter("base_frame", "proximal_ring")
        self.declare_parameter("platform_frame", "distal_ring")
        self.declare_parameter("publish_rate_hz", 20.0)

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

        self.sim = bool(self.get_parameter("sim").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.platform_frame = str(self.get_parameter("platform_frame").value)

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

        package_share = Path(get_package_share_directory("kimm_control"))
        library_path = package_share / "lib" / "kimm_6rss" / "libKIMMSPKernel.so"

        kinematics = KimmKinematics(library_path)

        if self.sim:
            self.get_logger().info("Using simulated hardware bridge.")
            self.bridge = SimulatedBridge(
                kinematics=kinematics,
                home_pose=self.home_pose,
            )
        else:
            raise NotImplementedError(
                "Real hardware bridge is not implemented yet. Use sim:=true."
            )

        self.bridge.configure()
        self.bridge.activate()

        self.command_pose_sub = self.create_subscription(
            PoseStamped,
            "/kimm/command/pose",
            self.command_pose_callback,
            10,
        )

        self.state_pose_pub = self.create_publisher(
            PoseStamped,
            "/kimm/state/pose",
            10,
        )

        self.state_joints_pub = self.create_publisher(
            JointState,
            "/kimm/state/joints",
            10,
        )

        self.joint_states_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.bridge_status_pub = self.create_publisher(
            Bool,
            "/kimm/bridge_status",
            10,
        )

        self.bridge_enable_srv = self.create_service(
            SetBool,
            "/kimm/bridge_enable",
            self.bridge_enable_callback,
        )

        self.home_srv = self.create_service(
            Trigger,
            "/kimm/home",
            self.home_callback,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_state,
        )

        self.get_logger().info("KIMM robot node is ready.")

    def command_pose_callback(self, msg):
        state = self.bridge.read()

        if not state["enabled"]:
            self.get_logger().warn("Ignoring pose command because bridge is disabled.")
            return

        try:
            pose = pose_stamped_to_pose_dict(msg)
            self.bridge.write_pose(pose)
        except Exception as exc:
            self.get_logger().error(f"Failed to process /kimm/command/pose: {exc}")

    def bridge_enable_callback(self, request, response):
        if request.data:
            self.bridge.activate()
            response.success = True
            response.message = "KIMM bridge enabled."
        else:
            self.bridge.deactivate()
            response.success = True
            response.message = "KIMM bridge disabled."

        return response

    def home_callback(self, request, response):
        try:
            self.bridge.write_pose(self.home_pose)
            response.success = True
            response.message = "KIMM robot moved to home pose."
        except Exception as exc:
            response.success = False
            response.message = f"Failed to home KIMM robot: {exc}"

        return response

    def publish_state(self):
        state = self.bridge.read()
        stamp = self.get_clock().now().to_msg()

        pose_msg = pose_dict_to_pose_stamped(
            state["pose"],
            frame_id=self.base_frame,
            stamp=stamp,
        )
        self.state_pose_pub.publish(pose_msg)

        actuator_joint_state = actuator_positions_to_joint_state(
            state["actuator_positions_rad"],
            self.actuator_joint_names,
            stamp,
        )
        self.state_joints_pub.publish(actuator_joint_state)

        virtual_joint_state = pose_dict_to_virtual_joint_state(
            state["pose"],
            self.platform_joint_names,
            stamp,
        )
        self.joint_states_pub.publish(virtual_joint_state)

        status_msg = Bool()
        status_msg.data = bool(state["enabled"])
        self.bridge_status_pub.publish(status_msg)

    def destroy_node(self):
        try:
            self.bridge.close()
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