import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python import get_package_share_path


def generate_launch_description():

    config_arg = DeclareLaunchArgument(
        "config",
        default_value="RPR",
        description="Set wrist configuration. Options: RPR/YPR/2DOF"
    )

    def get_urdf_file(context, *args, **kwargs):
        config = LaunchConfiguration("config").perform(context)

        urdf_path = os.path.join(
            get_package_share_path('asystr_dual_arm_model'),
            'urdf', 'dual_arm.urdf.xacro'
        )

        robot_description = ParameterValue(
            Command(['xacro ', urdf_path, " config:=", config]),
            value_type=str
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': robot_description}]
        )

        return [robot_state_publisher_node]

    rviz_config_path = os.path.join(
        get_package_share_path('asystr_dual_arm_model'),
        'rviz', 'default.rviz'
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    tray_pose_broadcaster_node = Node(
        package="asystr_dual_arm_model",
        executable="tray_pose_broadcaster",
        name="tray_pose_broadcaster",
        output="screen"
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=get_urdf_file),
        joint_state_publisher_gui_node,
        rviz2_node,
        tray_pose_broadcaster_node
    ])
