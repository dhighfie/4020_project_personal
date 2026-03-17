#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration

PYTHON_EXEC = "/usr/bin/python3"


def generate_launch_description() -> LaunchDescription:
    here = os.path.dirname(os.path.abspath(__file__))
    backend_script = os.path.join(here, "mock_pick_backend.py")

    request_topic_arg = DeclareLaunchArgument("request_topic", default_value="/pick/request")
    status_topic_arg = DeclareLaunchArgument("status_topic", default_value="/pick/status")
    fail_rate_arg = DeclareLaunchArgument("fail_rate", default_value="0.0")

    backend = ExecuteProcess(
        cmd=[
            PYTHON_EXEC,
            backend_script,
            "--ros-args",
            "-p",
            ["request_topic:=", LaunchConfiguration("request_topic")],
            "-p",
            ["status_topic:=", LaunchConfiguration("status_topic")],
            "-p",
            ["fail_rate:=", LaunchConfiguration("fail_rate")],
        ],
        output="screen",
        shell=False,
    )

    return LaunchDescription(
        [
            request_topic_arg,
            status_topic_arg,
            fail_rate_arg,
            LogInfo(msg="Starting mock pick pipeline backend"),
            backend,
        ]
    )
