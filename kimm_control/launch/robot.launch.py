from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    rviz = LaunchConfiguration("rviz")

    world_frame = LaunchConfiguration("world_frame")
    base_frame = LaunchConfiguration("base_frame")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    control_config_file = PathJoinSubstitution([
        FindPackageShare("kimm_control"),
        "config",
        "kimm_control.yaml",
    ])

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

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("kimm_description"),
        "rviz",
        "kimm_display.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="Use simulated hardware bridge if true.",
        ),

        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz if true.",
        ),

        DeclareLaunchArgument(
            "world_frame",
            default_value="world",
            description="Global scanner/world frame.",
        ),

        DeclareLaunchArgument(
            "base_frame",
            default_value="proximal_ring",
            description="KIMM robot base frame.",
        ),

        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        # Temporary registration transform:
        #   world/scanner -> proximal_ring
        #
        # Later, this should be replaced by the registration node.
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
            package="kimm_control",
            executable="kimm_robot_node",
            name="kimm_robot_node",
            output="screen",
            parameters=[
                control_config_file,
                {
                    "sim": ParameterValue(sim, value_type=bool),
                },
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(rviz),
            output="screen",
        ),
    ])