import math

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


def quaternion_to_rpy(q):
    """Convert quaternion to roll, pitch, yaw in radians.

    Convention:
      roll  about X
      pitch about Y
      yaw   about Z
    """

    x = q.x
    y = q.y
    z = q.z
    w = q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rpy_to_quaternion(roll, pitch, yaw):
    """Convert roll, pitch, yaw in radians to quaternion tuple."""

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


def pose_stamped_to_pose_dict(msg: PoseStamped):
    """Convert geometry_msgs/PoseStamped to internal pose dictionary.

    Internal units:
      translation: meters
      rotation: radians
    """

    roll, pitch, yaw = quaternion_to_rpy(msg.pose.orientation)

    return {
        "tx_m": float(msg.pose.position.x),
        "ty_m": float(msg.pose.position.y),
        "tz_m": float(msg.pose.position.z),
        "rx_rad": float(roll),
        "ry_rad": float(pitch),
        "rz_rad": float(yaw),
    }


def pose_dict_to_pose_stamped(pose, frame_id, stamp):
    """Convert internal pose dictionary to geometry_msgs/PoseStamped."""

    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.pose.position.x = float(pose["tx_m"])
    msg.pose.position.y = float(pose["ty_m"])
    msg.pose.position.z = float(pose["tz_m"])

    qx, qy, qz, qw = rpy_to_quaternion(
        float(pose["rx_rad"]),
        float(pose["ry_rad"]),
        float(pose["rz_rad"]),
    )

    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    return msg


def pose_dict_to_kimm_input(pose):
    """Convert internal pose dictionary to KIMM-native input.

    KIMM input order:
      [Rx_deg, Ry_deg, Rz_deg, Tx_mm, Ty_mm, Tz_mm]

    Your existing KIMM example uses this six-value order. The README also
    documents Rx/Ry/Rz in degrees and Tx/Ty/Tz in mm.
    """

    return [
        math.degrees(float(pose["rx_rad"])),
        math.degrees(float(pose["ry_rad"])),
        math.degrees(float(pose["rz_rad"])),
        float(pose["tx_m"]) * 1000.0,
        float(pose["ty_m"]) * 1000.0,
        float(pose["tz_m"]) * 1000.0,
    ]


def pose_dict_to_virtual_joint_state(pose, joint_names, stamp):
    """Create /joint_states for the virtual platform joints in kimm_description.

    Joint order:
      platform_x_joint      tx in meters
      platform_y_joint      ty in meters
      platform_z_joint      tz in meters
      platform_roll_joint   rx in radians
      platform_pitch_joint  ry in radians
      platform_yaw_joint    rz in radians
    """

    msg = JointState()
    msg.header.stamp = stamp
    msg.name = list(joint_names)
    msg.position = [
        float(pose["tx_m"]),
        float(pose["ty_m"]),
        float(pose["tz_m"]),
        float(pose["rx_rad"]),
        float(pose["ry_rad"]),
        float(pose["rz_rad"]),
    ]

    msg.velocity = [0.0] * len(msg.name)
    msg.effort = [0.0] * len(msg.name)

    return msg


def actuator_positions_to_joint_state(actuator_positions_rad, joint_names, stamp):
    """Create /kimm/state/joints for the six actuator angles.

    JointState.position is in radians for revolute joints.
    """

    msg = JointState()
    msg.header.stamp = stamp
    msg.name = list(joint_names)
    msg.position = [float(x) for x in actuator_positions_rad]

    msg.velocity = [0.0] * len(msg.name)
    msg.effort = [0.0] * len(msg.name)


    return msg