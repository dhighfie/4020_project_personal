#!/usr/bin/env python3
"""
Mock pick executor for PC-only testing.

Subscribes: /pick/request (std_msgs/String JSON payload)
Publishes:  /pick/status  (std_msgs/String JSON payload)

Example request payload:
{
  "request_id": "demo-1",
  "color": "red",
  "pick_pose": {"x": 0.20, "y": 0.10, "z": 0.03, "yaw": 0.0},
  "place": "red_bin"
}
"""

import json
import random
import threading
import time
from datetime import datetime, timezone
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockPickBackend(Node):
    def __init__(self) -> None:
        super().__init__("mock_pick_backend")

        self.declare_parameter("request_topic", "/pick/request")
        self.declare_parameter("status_topic", "/pick/status")
        self.declare_parameter("reject_when_busy", True)
        self.declare_parameter("fail_rate", 0.0)
        self.declare_parameter("seed", 42)

        self.declare_parameter("x_min", 0.05)
        self.declare_parameter("x_max", 0.32)
        self.declare_parameter("y_min", -0.18)
        self.declare_parameter("y_max", 0.18)
        self.declare_parameter("z_min", 0.00)
        self.declare_parameter("z_max", 0.18)

        request_topic = str(self.get_parameter("request_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.reject_when_busy = bool(self.get_parameter("reject_when_busy").value)
        self.fail_rate = float(self.get_parameter("fail_rate").value)

        self._rng = random.Random(int(self.get_parameter("seed").value))
        self._busy = False
        self._lock = threading.Lock()

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.request_sub = self.create_subscription(String, request_topic, self._on_request, 10)

        self.get_logger().info(f"Mock backend ready. request_topic={request_topic} status_topic={status_topic}")

    def _publish_status(
        self,
        request_id: str,
        state: str,
        ok: bool,
        detail: str,
        payload: Optional[dict] = None,
    ) -> None:
        msg_dict = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "request_id": request_id,
            "state": state,
            "ok": ok,
            "detail": detail,
        }
        if payload:
            msg_dict["request"] = payload

        msg = String()
        msg.data = json.dumps(msg_dict)
        self.status_pub.publish(msg)
        self.get_logger().info(f"[{request_id}] {state} | ok={ok} | {detail}")

    def _validate_request(self, request: dict) -> Tuple[bool, str]:
        if not isinstance(request, dict):
            return False, "Request must be a JSON object."

        color = request.get("color")
        if not isinstance(color, str) or not color.strip():
            return False, "Missing required field: color (string)."

        pick_pose = request.get("pick_pose")
        if pick_pose is None:
            return True, "No pick_pose provided; simulating locate + plan only."

        if not isinstance(pick_pose, dict):
            return False, "pick_pose must be an object with x/y/z/yaw."

        for key in ("x", "y", "z", "yaw"):
            if key not in pick_pose:
                return False, f"pick_pose missing field: {key}."
            if not isinstance(pick_pose[key], (int, float)):
                return False, f"pick_pose.{key} must be numeric."

        x = float(pick_pose["x"])
        y = float(pick_pose["y"])
        z = float(pick_pose["z"])
        bounds = {
            "x": (float(self.get_parameter("x_min").value), float(self.get_parameter("x_max").value)),
            "y": (float(self.get_parameter("y_min").value), float(self.get_parameter("y_max").value)),
            "z": (float(self.get_parameter("z_min").value), float(self.get_parameter("z_max").value)),
        }
        if not (bounds["x"][0] <= x <= bounds["x"][1]):
            return False, f"x out of bounds [{bounds['x'][0]}, {bounds['x'][1]}]."
        if not (bounds["y"][0] <= y <= bounds["y"][1]):
            return False, f"y out of bounds [{bounds['y'][0]}, {bounds['y'][1]}]."
        if not (bounds["z"][0] <= z <= bounds["z"][1]):
            return False, f"z out of bounds [{bounds['z'][0]}, {bounds['z'][1]}]."

        return True, "Request validated."

    def _on_request(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_status("unknown", "invalid", False, f"Invalid JSON: {exc}")
            return

        request_id = str(request.get("request_id") or f"req-{int(time.time() * 1000)}")

        valid, detail = self._validate_request(request)
        if not valid:
            self._publish_status(request_id, "invalid", False, detail, request)
            return

        with self._lock:
            if self._busy and self.reject_when_busy:
                self._publish_status(request_id, "rejected", False, "Backend busy.", request)
                return
            self._busy = True

        self._publish_status(request_id, "accepted", True, detail, request)
        worker = threading.Thread(target=self._run_sequence, args=(request_id, request), daemon=True)
        worker.start()

    def _run_sequence(self, request_id: str, request: dict) -> None:
        motion_plan = request.get("motion_plan", [])
        phases = [("locate", 0.4, "Locate target block in scene."), ("plan", 0.5, "Build approach/pick/place waypoints.")]
        if isinstance(motion_plan, list) and motion_plan:
            for step in motion_plan:
                if not isinstance(step, dict):
                    continue
                name = str(step.get("name", "motion_step"))
                if "pose" in step and isinstance(step["pose"], dict):
                    pose = step["pose"]
                    detail = (
                        f"Move to pose x={float(pose.get('x', 0.0)):.3f}, "
                        f"y={float(pose.get('y', 0.0)):.3f}, z={float(pose.get('z', 0.0)):.3f}"
                    )
                    phases.append((name, 0.5, detail))
                elif "action" in step:
                    phases.append((name, 0.3, f"Tool action: {step['action']}"))
                else:
                    phases.append((name, 0.3, "Execute step."))
        else:
            # Fallback default sequence if no explicit plan was provided.
            phases.extend(
                [
                    ("approach", 0.6, "Move to approach pose."),
                    ("descend", 0.5, "Move to pick pose."),
                    ("grasp", 0.4, "Actuate gripper/pump."),
                    ("retreat", 0.5, "Lift from table."),
                    ("place", 0.7, "Move to destination and release."),
                ]
            )
        try:
            for state, delay_s, detail in phases:
                self._publish_status(request_id, state, True, detail)
                time.sleep(delay_s)

            if self._rng.random() < self.fail_rate:
                self._publish_status(
                    request_id,
                    "failed",
                    False,
                    "Simulated runtime failure (IK unreachable or grasp miss).",
                )
                return

            self._publish_status(request_id, "completed", True, "Mock pick cycle complete.")
        finally:
            with self._lock:
                self._busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPickBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
