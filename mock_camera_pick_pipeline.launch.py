#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration

PYTHON_EXEC = "/usr/bin/python3"


def generate_launch_description() -> LaunchDescription:
    here = os.path.dirname(os.path.abspath(__file__))
    default_image = os.path.join(here, "assets", "aikit_280pi_block_sample.png")

    image_path_arg = DeclareLaunchArgument("image_path", default_value=default_image)
    target_color_arg = DeclareLaunchArgument("target_color", default_value="red")
    fail_rate_arg = DeclareLaunchArgument("fail_rate", default_value="0.0")
    show_window_arg = DeclareLaunchArgument("show_window", default_value="true")
    red_bin_x_arg = DeclareLaunchArgument("red_bin_x", default_value="0.16")
    red_bin_y_arg = DeclareLaunchArgument("red_bin_y", default_value="-0.16")
    green_bin_x_arg = DeclareLaunchArgument("green_bin_x", default_value="0.21")
    green_bin_y_arg = DeclareLaunchArgument("green_bin_y", default_value="-0.16")
    blue_bin_x_arg = DeclareLaunchArgument("blue_bin_x", default_value="0.26")
    blue_bin_y_arg = DeclareLaunchArgument("blue_bin_y", default_value="-0.16")
    yellow_bin_x_arg = DeclareLaunchArgument("yellow_bin_x", default_value="0.31")
    yellow_bin_y_arg = DeclareLaunchArgument("yellow_bin_y", default_value="-0.16")

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

    return LaunchDescription(
        [
            image_path_arg,
            target_color_arg,
            fail_rate_arg,
            show_window_arg,
            red_bin_x_arg,
            red_bin_y_arg,
            green_bin_x_arg,
            green_bin_y_arg,
            blue_bin_x_arg,
            blue_bin_y_arg,
            yellow_bin_x_arg,
            yellow_bin_y_arg,
            LogInfo(msg="Starting mock camera -> detect -> plan -> pick pipeline"),
            camera,
            detector,
            planner,
            backend,
        ]
    )
