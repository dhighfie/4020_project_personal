#!/usr/bin/env python3
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    res = []

    # Match the working slider_control_pump.launch.py arguments first.
    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mycobot_280_m5/mycobot_280_m5_with_pump.urdf",
        ),
    )
    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_280"),
            "config/mycobot.rviz",
        ),
    )
    gui_launch_arg = DeclareLaunchArgument("gui", default_value="true")
    serial_port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port to use",
    )
    baud_rate_arg = DeclareLaunchArgument(
        "baud",
        default_value="115200",
        description="Baud rate to use",
    )

    # Probe-specific arguments.
    probe_package_arg = DeclareLaunchArgument(
        "probe_package",
        default_value="group_project_group_5",
        description="ROS package name that contains ros2_autonomy_probe.py",
    )
    target_topic_arg = DeclareLaunchArgument(
        "target_topic",
        default_value="",
        description="Optional topic to inspect (example: /joint_states).",
    )
    sample_topic_arg = DeclareLaunchArgument(
        "sample_topic",
        default_value="false",
        description="Whether the probe should sample one message from target_topic.",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="probe_reports",
        description="Directory where probe reports are saved.",
    )
    camera_mount_arg = DeclareLaunchArgument("camera_mount", default_value="")
    vision_method_arg = DeclareLaunchArgument("vision_method_first", default_value="")
    workspace_origin_arg = DeclareLaunchArgument("workspace_origin_defined", default_value="")
    milestone_arg = DeclareLaunchArgument("priority_milestone", default_value="")

    res.extend(
        [
            model_launch_arg,
            rvizconfig_launch_arg,
            gui_launch_arg,
            serial_port_arg,
            baud_rate_arg,
            probe_package_arg,
            target_topic_arg,
            sample_topic_arg,
            output_dir_arg,
            camera_mount_arg,
            vision_method_arg,
            workspace_origin_arg,
            milestone_arg,
        ]
    )

    # Log the configuration clearly so you can see what launch is doing.
    res.append(LogInfo(msg="Starting autonomy_probe.launch.py"))
    res.append(LogInfo(msg=["  model=", LaunchConfiguration("model")]))
    res.append(LogInfo(msg=["  rvizconfig=", LaunchConfiguration("rvizconfig")]))
    res.append(LogInfo(msg=["  gui=", LaunchConfiguration("gui")]))
    res.append(LogInfo(msg=["  port=", LaunchConfiguration("port")]))
    res.append(LogInfo(msg=["  baud=", LaunchConfiguration("baud")]))
    res.append(LogInfo(msg=["  target_topic=", LaunchConfiguration("target_topic")]))
    res.append(LogInfo(msg=["  sample_topic=", LaunchConfiguration("sample_topic")]))

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    res.append(LogInfo(msg="Launching robot_state_publisher"))
    res.append(robot_state_publisher_node)

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
        output="screen",
    )
    res.append(LogInfo(msg="Launching joint_state_publisher_gui (if gui:=true)"))
    res.append(joint_state_publisher_gui_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    res.append(LogInfo(msg="Launching rviz2"))
    res.append(rviz_node)

    slider_control_node = Node(
        package="mycobot_280",
        executable="slider_control",
        parameters=[
            {"port": LaunchConfiguration("port")},
            {"baud": LaunchConfiguration("baud")},
        ],
        name="slider_control",
        output="screen",
    )
    res.append(LogInfo(msg="Launching mycobot slider_control"))
    res.append(slider_control_node)

    # Run the probe in ROS-node mode as an additional process.
    probe_script_path = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("probe_package")),
            "ros2_autonomy_probe.py",
        ]
    )

    probe_process = ExecuteProcess(
        cmd=[
            "python3",
            probe_script_path,
            "--ros-node",
            "--ros-args",
            "-p",
            ["target_topic:=", LaunchConfiguration("target_topic")],
            "-p",
            ["sample_topic:=", LaunchConfiguration("sample_topic")],
            "-p",
            ["output_dir:=", LaunchConfiguration("output_dir")],
            "-p",
            ["camera_mount:=", LaunchConfiguration("camera_mount")],
            "-p",
            ["vision_method_first:=", LaunchConfiguration("vision_method_first")],
            "-p",
            ["workspace_origin_defined:=", LaunchConfiguration("workspace_origin_defined")],
            "-p",
            ["priority_milestone:=", LaunchConfiguration("priority_milestone")],
        ],
        output="screen",
        shell=False,
    )
    res.append(LogInfo(msg="Launching ros2_autonomy_probe.py --ros-node"))
    res.append(probe_process)

    res.append(LogInfo(msg="autonomy_probe.launch.py setup complete; processes running."))
    return LaunchDescription(res)
