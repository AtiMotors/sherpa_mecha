import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python import get_package_share_path

def generate_launch_description():

    setup_arg = DeclareLaunchArgument(
        "setup",
        default_value= "base",
        description="Deploy 3DOF base, 3DOF wrist or 6DOF robot. Options: base/full/wrist"
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value= "RPR",
        description="Set wrist configuration. Options: RPR/YPR/2DOF"
    )

    chiral_arg = DeclareLaunchArgument(
        "chiral",
        default_value= "no",
        description="Flip the robot. Options: yes/no"
    )

    ns_arg = DeclareLaunchArgument(
        "ns",
        default_value= "",
        description="Set namespace for robot model"
    )

    def get_urdf_file(context, *args, **kwargs):

        setup = LaunchConfiguration("setup").perform(context)
        config = LaunchConfiguration("config").perform(context)
        chiral = LaunchConfiguration("chiral").perform(context)
        ns = LaunchConfiguration("ns").perform(context)

        if setup == "base":
            urdf_file = "asystr_sensible.urdf.xacro"
        else:
            urdf_file = ["asystr_sensible_", config, ".urdf.xacro"]

        urdf_path = PathJoinSubstitution([
            str(get_package_share_path('asystr_sensible_model')),
            "urdf",
            urdf_file
        ])

        if not setup == "base":
            robot_description = ParameterValue(Command(['xacro ', urdf_path, " setup:=", setup, " chiral:=", chiral, " ns:=", ns]), value_type=str)
        else:
            robot_description = ParameterValue(Command(['xacro ', urdf_path, " chiral:=", chiral, " ns:=", ns]), value_type=str)

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': robot_description}]
        )

        return [robot_state_publisher_node]

    rviz_config_path = os.path.join(get_package_share_path('asystr_sensible_model'),
                                    'rviz', 'default.rviz')
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        setup_arg,
        config_arg,
        chiral_arg,
        ns_arg,
        OpaqueFunction(function=get_urdf_file),
        joint_state_publisher_gui_node,
        rviz2_node
    ])