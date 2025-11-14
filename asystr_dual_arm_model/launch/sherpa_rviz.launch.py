import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # Path to sherpa URDF Xacro
    urdf_path = os.path.join(
        get_package_share_path('asystr_dual_arm_model'),
        'urdf',
        'sherpa_dual_arm.urdf.xacro'
    )

    # Generate robot_description parameter
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI (for sliders)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # RViz with default config (if exists)
    rviz_config_path = os.path.join(
        get_package_share_path('asystr_dual_arm_model'),
        'rviz',
        'default.rviz'
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])

