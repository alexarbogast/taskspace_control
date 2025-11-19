from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            choices=["pose_controller"],
            default_value="pose_controller",
            description="Which controller should be started?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="robot6R",
            choices=["robot6R", "robot7R"],
            description="Select which robot configuration to use",
        )
    )

    robot_type = LaunchConfiguration("robot_type")
    controller = LaunchConfiguration("controller")

    pose_control_demo_node = Node(
        package="taskspace_control_examples",
        executable="pose_control_demo",
        name="pose_control_demo",
        output="screen",
        parameters=[{"controller": controller, "robot_type": robot_type}],
    )
    return LaunchDescription(declared_arguments + [pose_control_demo_node])
