#!/usr/bin/env python3
"""
ROS 2 probe helper for turning a manual slider-based robot setup into an
autonomous pipeline.

What this script does:
1) Captures ROS graph info (nodes/topics/services/actions)
2) Helps identify the command topic used by the manual slider path
3) Lets you sample topic messages while moving sliders
4) Asks project-planning questions and saves answers to a report

Run this while your launch file is already running:
    python ros2_autonomy_probe.py

The script uses the `ros2` CLI, so make sure your ROS environment is sourced.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import List, Optional

try:
    import rclpy
    from rclpy.node import Node
except ImportError:  # Allows plain Python use on systems without rclpy in path.
    rclpy = None
    Node = object  # type: ignore[assignment]


TIMEOUT_SEC = 8


@dataclass
class TopicInfo:
    name: str
    msg_type: str


def run_cmd(args: List[str], timeout: int = TIMEOUT_SEC) -> tuple[int, str, str]:
    try:
        proc = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return proc.returncode, proc.stdout.strip(), proc.stderr.strip()
    except subprocess.TimeoutExpired:
        return 124, "", f"Timed out after {timeout}s: {' '.join(args)}"
    except FileNotFoundError:
        return 127, "", f"Command not found: {args[0]}"


def prompt(text: str, default: Optional[str] = None) -> str:
    suffix = f" [{default}]" if default else ""
    value = input(f"{text}{suffix}: ").strip()
    return value if value else (default or "")


def yes_no(text: str, default_yes: bool = True) -> bool:
    default = "Y/n" if default_yes else "y/N"
    answer = input(f"{text} ({default}): ").strip().lower()
    if not answer:
        return default_yes
    return answer in {"y", "yes"}


def print_section(title: str) -> None:
    print("\n" + "=" * 72)
    print(title)
    print("=" * 72)


def ros2_available() -> bool:
    if not shutil.which("ros2"):
        return False
    code, _, _ = run_cmd(["ros2", "--help"], timeout=5)
    return code == 0


def get_ros_graph_snapshot() -> dict:
    snapshot = {}

    commands = {
        "nodes": ["ros2", "node", "list"],
        "topics_with_types": ["ros2", "topic", "list", "-t"],
        "services": ["ros2", "service", "list"],
        "actions": ["ros2", "action", "list"],
    }

    for key, cmd in commands.items():
        code, out, err = run_cmd(cmd)
        snapshot[key] = {
            "command": cmd,
            "returncode": code,
            "stdout": out,
            "stderr": err,
        }

    return snapshot


def parse_topics_with_types(raw: str) -> List[TopicInfo]:
    topics: List[TopicInfo] = []
    for line in raw.splitlines():
        line = line.strip()
        if not line:
            continue
        # Expected format: /topic_name [msg/type]
        if "[" in line and "]" in line:
            name = line.split("[", 1)[0].strip()
            msg_type = line.split("[", 1)[1].split("]", 1)[0].strip()
            topics.append(TopicInfo(name=name, msg_type=msg_type))
        else:
            topics.append(TopicInfo(name=line, msg_type="UNKNOWN"))
    return topics


def find_candidate_topics(topics: List[TopicInfo]) -> List[TopicInfo]:
    keywords = (
        "joint",
        "cmd",
        "command",
        "slider",
        "trajectory",
        "state",
        "control",
        "pump",
    )
    ranked = []
    for t in topics:
        score = sum(1 for k in keywords if k in t.name.lower())
        if score > 0:
            ranked.append((score, t))
    ranked.sort(key=lambda item: (-item[0], item[1].name))
    return [t for _, t in ranked]


def topic_info(topic_name: str) -> dict:
    code, out, err = run_cmd(["ros2", "topic", "info", "-v", topic_name])
    return {
        "command": ["ros2", "topic", "info", "-v", topic_name],
        "returncode": code,
        "stdout": out,
        "stderr": err,
    }


def sample_topic_once(topic_name: str) -> dict:
    # --once is supported on common ROS 2 distros. Timeout prevents hanging.
    code, out, err = run_cmd(["ros2", "topic", "echo", "--once", topic_name], timeout=12)
    return {
        "command": ["ros2", "topic", "echo", "--once", topic_name],
        "returncode": code,
        "stdout": out,
        "stderr": err,
    }


def choose_topic(candidates: List[TopicInfo], all_topics: List[TopicInfo]) -> Optional[TopicInfo]:
    print_section("Candidate Topics")
    if candidates:
        for i, t in enumerate(candidates, start=1):
            print(f"{i:2d}. {t.name} [{t.msg_type}]")
    else:
        print("No keyword-based candidates found.")

    print("\nType a number to select a candidate, or type a topic name manually.")
    raw = input("Selection (blank to skip): ").strip()
    if not raw:
        return None

    if raw.isdigit():
        idx = int(raw)
        if 1 <= idx <= len(candidates):
            return candidates[idx - 1]
        print("Invalid number.")
        return None

    for t in all_topics:
        if t.name == raw:
            return t
    return TopicInfo(name=raw, msg_type="UNKNOWN")


def collect_project_questions() -> dict:
    print_section("Project Questions (Answer what you know)")
    answers = {}
    answers["ros_distro"] = prompt("ROS 2 distro (e.g., Humble/Foxy/Jazzy)", "")
    answers["robot_model"] = prompt("Exact robot model (e.g., myCobot 280 M5)", "myCobot 280 M5")
    answers["camera_mount"] = prompt(
        "Camera setup (fixed overhead / wrist-mounted / none yet)",
        "fixed overhead",
    )
    answers["vision_method_first"] = prompt(
        "First vision method (color threshold / ML detector / mock data)",
        "color threshold",
    )
    answers["target_objects"] = prompt(
        "Objects to sort first (comma-separated, e.g., red_block,blue_block)",
        "red_block",
    )
    answers["destinations"] = prompt(
        "Destinations/bins first (comma-separated, e.g., red_bin,blue_bin)",
        "red_bin",
    )
    answers["workspace_origin_defined"] = prompt(
        "Do you already have a workspace origin/reference frame defined? (yes/no + notes)",
        "no",
    )
    answers["table_height_or_z_reference"] = prompt(
        "Table height or z reference (mm) if known",
        "",
    )
    answers["gripper_or_pump_control_known"] = prompt(
        "Do you know how pump on/off is commanded yet? (topic/service/direct in slider_control)",
        "unknown",
    )
    answers["safe_bounds_xyz_mm"] = prompt(
        "Safe workspace bounds x,y,z in mm (free text)",
        "",
    )
    answers["max_speed_policy"] = prompt(
        "Initial speed policy for testing (e.g., slow only)",
        "slow only",
    )
    answers["priority_milestone"] = prompt(
        "First milestone (e.g., scripted move / pick one block / full pick-place)",
        "scripted move",
    )
    return answers


def build_next_steps(answers: dict, topic_pick: Optional[TopicInfo], topic_probe: dict) -> List[str]:
    steps: List[str] = []
    if topic_pick is None:
        steps.append("Identify the exact command topic used by slider_control while moving a GUI slider.")
    else:
        steps.append(
            f"Confirm whether `{topic_pick.name}` is the command path by observing value changes while moving one slider."
        )

    if topic_pick and topic_pick.msg_type == "sensor_msgs/msg/JointState":
        steps.append("Build a test publisher node that sends JointState messages (single-joint move first).")
    elif topic_pick and topic_pick.msg_type != "UNKNOWN":
        steps.append(f"Create a minimal publisher for `{topic_pick.msg_type}` to reproduce a manual pose automatically.")
    else:
        steps.append("Use `ros2 topic list -t` and `ros2 topic info -v <topic>` to determine the command message type.")

    if "fixed" in answers.get("camera_mount", "").lower():
        steps.append("Plan camera-to-robot calibration for a fixed overhead camera (pixel-to-world transform).")
    else:
        steps.append("Define camera extrinsics approach before implementing pixel-to-world conversion.")

    if answers.get("vision_method_first", "").lower().startswith("color"):
        steps.append("Start with color-based detection for one object class before using an ML detector.")
    if not answers.get("workspace_origin_defined", "").lower().startswith("y"):
        steps.append("Define and measure a physical workspace origin and record x/y/z reference values.")

    if topic_probe.get("returncode") != 0:
        steps.append("Re-run topic sampling while continuously moving one slider so `ros2 topic echo --once` captures a message.")

    return steps


def save_report(report: dict, topic_pick: Optional[TopicInfo], next_steps: List[str], output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = output_dir / f"ros2_autonomy_probe_{stamp}.json"
    md_path = output_dir / f"ros2_autonomy_probe_{stamp}.md"

    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    answers = report.get("project_answers", {})
    md_lines = [
        "# ROS 2 Autonomy Probe Report",
        "",
        f"- Timestamp: `{report.get('timestamp', '')}`",
        f"- Selected topic: `{topic_pick.name}`" if topic_pick else "- Selected topic: (none)",
        "",
        "## Project Answers",
    ]
    if answers:
        for k, v in answers.items():
            md_lines.append(f"- `{k}`: {v}")
    else:
        md_lines.append("- (none captured in this run)")
    md_lines.extend(["", "## Recommended Next Steps"])
    for i, step in enumerate(next_steps, start=1):
        md_lines.append(f"{i}. {step}")
    md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    return json_path, md_path


class AutonomyProbeNode(Node):
    """Non-interactive ROS node mode that snapshots the ROS graph and saves a report."""

    def __init__(self) -> None:
        super().__init__("autonomy_probe")
        self.declare_parameter("target_topic", "")
        self.declare_parameter("sample_topic", False)
        self.declare_parameter("output_dir", str(Path.cwd() / "probe_reports"))
        self.declare_parameter("camera_mount", "")
        self.declare_parameter("vision_method_first", "")
        self.declare_parameter("workspace_origin_defined", "")
        self.declare_parameter("priority_milestone", "")
        self._done = False
        self._timer = self.create_timer(0.5, self._run_once)

    def _run_once(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.cancel()

        self.get_logger().info("Starting ROS graph probe snapshot...")

        if not ros2_available():
            self.get_logger().error("`ros2` CLI not available. Source your ROS environment and retry.")
            self._shutdown(1)
            return

        report = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "host": {
                "python": sys.version.split()[0],
                "platform": sys.platform,
            },
            "mode": "ros_node",
        }
        graph = get_ros_graph_snapshot()
        report["ros_graph"] = graph

        topics = parse_topics_with_types(graph.get("topics_with_types", {}).get("stdout", ""))
        topic_by_name = {t.name: t for t in topics}
        selected_topic_name = str(self.get_parameter("target_topic").value or "").strip()
        topic_pick = topic_by_name.get(selected_topic_name) if selected_topic_name else None
        if selected_topic_name and topic_pick is None:
            topic_pick = TopicInfo(name=selected_topic_name, msg_type="UNKNOWN")
        report["selected_topic"] = asdict(topic_pick) if topic_pick else None

        topic_probe_info = {}
        topic_probe_sample = {}
        if topic_pick:
            self.get_logger().info(f"Inspecting topic: {topic_pick.name}")
            topic_probe_info = topic_info(topic_pick.name)
            if bool(self.get_parameter("sample_topic").value):
                self.get_logger().info("Sampling one message from selected topic...")
                topic_probe_sample = sample_topic_once(topic_pick.name)
        report["selected_topic_info"] = topic_probe_info
        report["selected_topic_sample"] = topic_probe_sample

        answers = {
            "ros_distro": "",
            "robot_model": "myCobot 280 M5",
            "camera_mount": str(self.get_parameter("camera_mount").value or ""),
            "vision_method_first": str(self.get_parameter("vision_method_first").value or ""),
            "workspace_origin_defined": str(self.get_parameter("workspace_origin_defined").value or ""),
            "priority_milestone": str(self.get_parameter("priority_milestone").value or ""),
            "notes": "ROS node mode is non-interactive. Use script mode for questionnaire prompts.",
        }
        report["project_answers"] = answers

        next_steps = build_next_steps(answers, topic_pick, topic_probe_sample or {})
        report["recommended_next_steps"] = next_steps

        output_dir = Path(str(self.get_parameter("output_dir").value))
        json_path, md_path = save_report(report, topic_pick, next_steps, output_dir)
        self.get_logger().info(f"Saved JSON report: {json_path}")
        self.get_logger().info(f"Saved Markdown report: {md_path}")
        self._shutdown(0)

    def _shutdown(self, code: int) -> None:
        self.get_logger().info("Autonomy probe node finished.")
        # Delay-free shutdown is fine here because this node is single-shot.
        rclpy.shutdown(context=self.context)
        self._exit_code = code


def run_ros_node_mode() -> int:
    if rclpy is None:
        print("`rclpy` is not available. This mode requires a ROS 2 Python environment.", file=sys.stderr)
        return 1
    rclpy.init()
    node = AutonomyProbeNode()
    node._exit_code = 0
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok(context=node.context):
            rclpy.shutdown(context=node.context)
        node.destroy_node()
    return int(getattr(node, "_exit_code", 0))


def run_interactive_mode() -> int:
    print_section("ROS 2 Autonomy Probe")
    print("Run this while your `slider_control_pump.launch.py` system is active.")

    if not ros2_available():
        print("\n`ros2` CLI was not found or not working.")
        print("Make sure ROS 2 is installed and your environment is sourced, then retry.")
        return 1

    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "host": {
            "python": sys.version.split()[0],
            "platform": sys.platform,
        },
        "mode": "interactive_script",
    }

    graph = get_ros_graph_snapshot()
    report["ros_graph"] = graph

    print_section("ROS Graph Summary")
    nodes_out = graph["nodes"]["stdout"]
    topic_out = graph["topics_with_types"]["stdout"]
    nodes = [n for n in nodes_out.splitlines() if n.strip()]
    topics = parse_topics_with_types(topic_out)

    print(f"Nodes found: {len(nodes)}")
    for n in nodes:
        print(f"  - {n}")
    print(f"Topics found: {len(topics)}")

    candidates = find_candidate_topics(topics)
    topic_pick = choose_topic(candidates, topics)
    report["selected_topic"] = asdict(topic_pick) if topic_pick else None

    topic_probe_info = {}
    topic_probe_sample = {}
    if topic_pick:
        print_section(f"Topic Info: {topic_pick.name}")
        topic_probe_info = topic_info(topic_pick.name)
        print(topic_probe_info.get("stdout") or topic_probe_info.get("stderr") or "(no output)")

        if yes_no("Sample one message from this topic now? Move a slider first if needed", default_yes=True):
            print("Waiting for one message...")
            topic_probe_sample = sample_topic_once(topic_pick.name)
            print("\n--- Sample Output ---")
            print(topic_probe_sample.get("stdout") or topic_probe_sample.get("stderr") or "(no output)")

    report["selected_topic_info"] = topic_probe_info
    report["selected_topic_sample"] = topic_probe_sample

    answers = collect_project_questions()
    report["project_answers"] = answers

    next_steps = build_next_steps(answers, topic_pick, topic_probe_sample or {})
    report["recommended_next_steps"] = next_steps

    print_section("Recommended Next Steps")
    for i, step in enumerate(next_steps, start=1):
        print(f"{i}. {step}")

    json_path, md_path = save_report(report, topic_pick, next_steps, Path.cwd() / "probe_reports")

    print_section("Saved")
    print(f"JSON report: {json_path}")
    print(f"Markdown report: {md_path}")
    print("\nShare the report or the selected topic/message sample and I can help write the first auto-controller node.")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS 2 autonomy probe (interactive tool or ROS node mode)")
    parser.add_argument(
        "--ros-node",
        action="store_true",
        help="Run as a non-interactive rclpy node that saves a probe report and exits.",
    )
    args = parser.parse_args()

    if args.ros_node:
        return run_ros_node_mode()
    return run_interactive_mode()


if __name__ == "__main__":
    raise SystemExit(main())
