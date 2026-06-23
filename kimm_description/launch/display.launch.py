from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Frame names
    world_frame = LaunchConfiguration("world_frame")
    base_frame = LaunchConfiguration("base_frame")

    # Temporary scanner/world to robot-base transform.
    # Later, registration should replace this.
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    robot_description_path = PathJoinSubstitution([
        FindPackageShare("kimm_description"),
        "urdf",
        "kimm_model.urdf.xacro",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                robot_description_path,
            ]),
            value_type=str,
        )
    }

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("kimm_description"),
        "rviz",
        "kimm_display.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_frame",
            default_value="world",
            description="Global scanner/world frame.",
        ),

        DeclareLaunchArgument(
            "base_frame",
            default_value="proximal_ring",
            description="Robot base frame. For KIMM this is the proximal ring.",
        ),

        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="Temporary world-to-base x translation in meters.",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="Temporary world-to-base y translation in meters.",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.0",
            description="Temporary world-to-base z translation in meters.",
        ),
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
            description="Temporary world-to-base roll in radians.",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
            description="Temporary world-to-base pitch in radians.",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Temporary world-to-base yaw in radians.",
        ),

        # Temporary TF for visualization:
        #   world -> proximal_ring
        #
        # Later, remove/disable this when a registration node publishes
        # the measured scanner/world -> proximal_ring transform.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_proximal_ring_static_tf",
            arguments=[
                x, y, z,
                roll, pitch, yaw,
                world_frame, base_frame,
            ],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            output="screen",
        ),
    ])