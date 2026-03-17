#!/usr/bin/env python3
import os

from ament_index_python import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PYTHON_EXEC = "/usr/bin/python3"


def _first_existing_path(candidates):
    for package_name, rel_path in candidates:
        try:
            base = get_package_share_directory(package_name)
        except PackageNotFoundError:
            continue
        full = os.path.join(base, rel_path)
        if os.path.exists(full):
            return full
    return ""


def generate_launch_description() -> LaunchDescription:
    here = os.path.dirname(os.path.abspath(__file__))
    default_image = os.path.join(here, "assets", "aikit_280pi_block_sample.png")

    model_default = _first_existing_path(
        [
            ("mycobot_description", "urdf/mycobot_280_m5/mycobot_280_m5_with_pump.urdf"),
            ("mycobot_description", "urdf/mycobot_280_pi/mycobot_280_pi_with_pump.urdf"),
        ]
    ) or "mycobot_280_m5_with_pump.urdf"
    rviz_default = _first_existing_path(
        [
            ("mycobot_280", "config/mycobot.rviz"),
            ("mycobot_280pi", "config/mycobot_pi.rviz"),
        ]
    ) or "mycobot.rviz"

    image_path_arg = DeclareLaunchArgument("image_path", default_value=default_image)
    target_color_arg = DeclareLaunchArgument("target_color", default_value="red")
    fail_rate_arg = DeclareLaunchArgument("fail_rate", default_value="0.0")
    show_window_arg = DeclareLaunchArgument("show_window", default_value="false")
    tool_pitch_arg = DeclareLaunchArgument("tool_pitch_rad", default_value="-1.5708")
    wrist_roll_arg = DeclareLaunchArgument("wrist_roll_rad", default_value="0.0")
    red_bin_x_arg = DeclareLaunchArgument("red_bin_x", default_value="0.16")
    red_bin_y_arg = DeclareLaunchArgument("red_bin_y", default_value="-0.16")
    green_bin_x_arg = DeclareLaunchArgument("green_bin_x", default_value="0.21")
    green_bin_y_arg = DeclareLaunchArgument("green_bin_y", default_value="-0.16")
    blue_bin_x_arg = DeclareLaunchArgument("blue_bin_x", default_value="0.26")
    blue_bin_y_arg = DeclareLaunchArgument("blue_bin_y", default_value="-0.16")
    yellow_bin_x_arg = DeclareLaunchArgument("yellow_bin_x", default_value="0.31")
    yellow_bin_y_arg = DeclareLaunchArgument("yellow_bin_y", default_value="-0.16")
    model_arg = DeclareLaunchArgument("model", default_value=model_default)
    rviz_arg = DeclareLaunchArgument("rvizconfig", default_value=rviz_default)

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    camera = ExecuteProcess(
        cmd=[
            PYTHON_EXEC,
            os.path.join(here, "image_replay_camera.py"),
            "--ros-args",
            "-p",
            ["image_path:=", LaunchConfiguration("image_path")],
            "-p",
            "fps:=4.0",
        ],
        output="screen",
        shell=False,
    )

    detector = ExecuteProcess(
        cmd=[
            PYTHON_EXEC,
            os.path.join(here, "block_detector.py"),
            "--ros-args",
            "-p",
            ["show_window:=", LaunchConfiguration("show_window")],
            "-p",
            "publish_period_sec:=1.0",
            "-p",
            "min_area:=400",
        ],
        output="screen",
        shell=False,
    )

    planner = ExecuteProcess(
        cmd=[
            PYTHON_EXEC,
            os.path.join(here, "pickup_path_planner_node.py"),
            "--ros-args",
            "-p",
            ["target_color:=", LaunchConfiguration("target_color")],
            "-p",
            "cooldown_sec:=4.0",
            "-p",
            ["red_bin_x:=", LaunchConfiguration("red_bin_x")],
            "-p",
            ["red_bin_y:=", LaunchConfiguration("red_bin_y")],
            "-p",
            ["green_bin_x:=", LaunchConfiguration("green_bin_x")],
            "-p",
            ["green_bin_y:=", LaunchConfiguration("green_bin_y")],
            "-p",
            ["blue_bin_x:=", LaunchConfiguration("blue_bin_x")],
            "-p",
            ["blue_bin_y:=", LaunchConfiguration("blue_bin_y")],
            "-p",
            ["yellow_bin_x:=", LaunchConfiguration("yellow_bin_x")],
            "-p",
            ["yellow_bin_y:=", LaunchConfiguration("yellow_bin_y")],
        ],
        output="screen",
        shell=False,
    )

    backend = ExecuteProcess(
        cmd=[
            PYTHON_EXEC,
            os.path.join(here, "mock_pick_backend.py"),
            "--ros-args",
            "-p",
            ["fail_rate:=", LaunchConfiguration("fail_rate")],
        ],
        output="screen",
        shell=False,
    )

    rviz_bridge = ExecuteProcess(
        cmd=[
            PYTHON_EXEC,
            os.path.join(here, "rviz_joint_motion_bridge.py"),
            "--ros-args",
            "-p",
            ["tool_pitch_rad:=", LaunchConfiguration("tool_pitch_rad")],
            "-p",
            ["wrist_roll_rad:=", LaunchConfiguration("wrist_roll_rad")],
        ],
        output="screen",
        shell=False,
    )

    return LaunchDescription(
        [
            image_path_arg,
            target_color_arg,
            fail_rate_arg,
            show_window_arg,
            tool_pitch_arg,
            wrist_roll_arg,
            red_bin_x_arg,
            red_bin_y_arg,
            green_bin_x_arg,
            green_bin_y_arg,
            blue_bin_x_arg,
            blue_bin_y_arg,
            yellow_bin_x_arg,
            yellow_bin_y_arg,
            model_arg,
            rviz_arg,
            LogInfo(msg="Starting mock camera + pick pipeline + RViz animation"),
            rsp,
            rviz,
            rviz_bridge,
            camera,
            detector,
            planner,
            backend,
        ]
    )
