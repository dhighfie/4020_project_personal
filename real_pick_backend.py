#!/usr/bin/env python3
"""
Real pick executor for myCobot over serial.

Subscribes: /pick/request (std_msgs/String JSON payload)
Publishes:  /pick/status  (std_msgs/String JSON payload)

Uses the same request format as mock_pick_backend.py, including optional
"motion_plan" with "pose" and "action" steps.
"""

import json
import math
import threading
import time
from datetime import datetime, timezone
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RealPickBackend(Node):
    def __init__(self) -> None:
        super().__init__("real_pick_backend")

        self.declare_parameter("request_topic", "/pick/request")
        self.declare_parameter("status_topic", "/pick/status")
        self.declare_parameter("reject_when_busy", True)
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("speed", 40)
        self.declare_parameter("coord_mode", 0)  # 0=angular/point-to-point, 1=linear
        self.declare_parameter("default_pause_sec", 0.4)
        self.declare_parameter("move_timeout_sec", 8.0)
        self.declare_parameter("position_tolerance_mm", 8.0)
        self.declare_parameter("rz_tolerance_deg", 8.0)
        self.declare_parameter("x_min", 0.05)
        self.declare_parameter("x_max", 0.32)
        self.declare_parameter("y_min", -0.18)
        self.declare_parameter("y_max", 0.18)
        self.declare_parameter("z_min", 0.00)
        self.declare_parameter("z_max", 0.18)

        # Tool orientation in degrees for send_coords [x,y,z,rx,ry,rz].
        self.declare_parameter("rx_deg", -180.0)
        self.declare_parameter("ry_deg", 0.0)

        # Tool action config.
        self.declare_parameter("tool_action_mode", "basic_output")  # none|basic_output|gripper
        self.declare_parameter("pump_pin_a", 2)
        self.declare_parameter("pump_pin_b", 5)
        self.declare_parameter("pump_on_a", 0)
        self.declare_parameter("pump_on_b", 0)
        self.declare_parameter("pump_off_a", 1)
        self.declare_parameter("pump_off_b", 1)
        self.declare_parameter("gripper_speed", 60)

        request_topic = str(self.get_parameter("request_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.reject_when_busy = bool(self.get_parameter("reject_when_busy").value)
        self.speed = int(self.get_parameter("speed").value)
        self.coord_mode = int(self.get_parameter("coord_mode").value)
        self.default_pause_sec = float(self.get_parameter("default_pause_sec").value)
        self.move_timeout_sec = float(self.get_parameter("move_timeout_sec").value)
        self.position_tolerance_mm = float(self.get_parameter("position_tolerance_mm").value)
        self.rz_tolerance_deg = float(self.get_parameter("rz_tolerance_deg").value)

        self._busy = False
        self._lock = threading.Lock()
        self._robot = self._connect_robot()

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.request_sub = self.create_subscription(String, request_topic, self._on_request, 10)
        self.get_logger().info(
            f"Real backend ready. request_topic={request_topic} status_topic={status_topic}"
        )

    def _connect_robot(self):
        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)

        import_error = None
        try:
            from pymycobot.mycobot import MyCobot

            robot = MyCobot(port, baud)
            time.sleep(1.0)
            self.get_logger().info(f"Connected to myCobot on {port} @ {baud}")
            return robot
        except Exception as exc:  # pragma: no cover - hardware/environment dependent
            import_error = exc

        raise RuntimeError(
            f"Failed to connect myCobot on {port} @ {baud}. "
            f"Ensure pymycobot is installed and serial permissions are set. Error: {import_error}"
        )

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

    def _validate_pose_bounds(self, pose: dict) -> Tuple[bool, str]:
        for key in ("x", "y", "z"):
            if key not in pose:
                return False, "pose missing x/y/z."
            if not isinstance(pose[key], (int, float)):
                return False, "pose x/y/z must be numeric."

        x = float(pose["x"])
        y = float(pose["y"])
        z = float(pose["z"])
        x_min = float(self.get_parameter("x_min").value)
        x_max = float(self.get_parameter("x_max").value)
        y_min = float(self.get_parameter("y_min").value)
        y_max = float(self.get_parameter("y_max").value)
        z_min = float(self.get_parameter("z_min").value)
        z_max = float(self.get_parameter("z_max").value)

        if not (x_min <= x <= x_max):
            return False, f"x out of bounds [{x_min}, {x_max}]"
        if not (y_min <= y <= y_max):
            return False, f"y out of bounds [{y_min}, {y_max}]"
        if not (z_min <= z <= z_max):
            return False, f"z out of bounds [{z_min}, {z_max}]"
        return True, "pose validated"

    def _validate_request(self, request: dict) -> Tuple[bool, str]:
        if not isinstance(request, dict):
            return False, "Request must be a JSON object."
        color = request.get("color")
        if not isinstance(color, str) or not color.strip():
            return False, "Missing required field: color (string)."

        pick_pose = request.get("pick_pose")
        if isinstance(pick_pose, dict):
            ok, detail = self._validate_pose_bounds(pick_pose)
            if not ok:
                return False, detail

        motion_plan = request.get("motion_plan")
        if motion_plan is not None and not isinstance(motion_plan, list):
            return False, "motion_plan must be a list when provided."
        return True, "Request validated."

    def _on_request(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_status("unknown", "invalid", False, f"Invalid JSON: {exc}")
            return

        request_id = str(request.get("request_id") or "req-unknown")
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

    def _yaw_rad_to_deg(self, yaw_rad: float) -> float:
        return math.degrees(float(yaw_rad))

    def _move_pose(self, pose: dict) -> None:
        x_m = float(pose["x"])
        y_m = float(pose["y"])
        z_m = float(pose["z"])
        yaw_rad = float(pose.get("yaw", 0.0))

        ok, detail = self._validate_pose_bounds({"x": x_m, "y": y_m, "z": z_m})
        if not ok:
            raise ValueError(detail)

        coords = [
            x_m * 1000.0,
            y_m * 1000.0,
            z_m * 1000.0,
            float(self.get_parameter("rx_deg").value),
            float(self.get_parameter("ry_deg").value),
            self._yaw_rad_to_deg(yaw_rad),
        ]

        self._robot.send_coords(coords, self.speed, self.coord_mode)
        self._wait_until_reached(coords)

    def _wait_until_reached(self, target_coords: list) -> None:
        t0 = time.time()
        while time.time() - t0 < self.move_timeout_sec:
            try:
                current = self._robot.get_coords()
            except Exception:
                time.sleep(0.15)
                continue

            if not isinstance(current, (list, tuple)) or len(current) < 6:
                time.sleep(0.15)
                continue

            dx = abs(float(current[0]) - float(target_coords[0]))
            dy = abs(float(current[1]) - float(target_coords[1]))
            dz = abs(float(current[2]) - float(target_coords[2]))
            drz = abs(float(current[5]) - float(target_coords[5]))
            if dx <= self.position_tolerance_mm and dy <= self.position_tolerance_mm and dz <= self.position_tolerance_mm and drz <= self.rz_tolerance_deg:
                return
            time.sleep(0.15)

        # Soft-timeout: continue execution, but do not silently ignore.
        raise TimeoutError("Move timeout waiting for target pose.")

    def _tool_action(self, action: str) -> None:
        mode = str(self.get_parameter("tool_action_mode").value).strip().lower()
        if mode == "none":
            return

        action = action.strip().lower()
        if mode == "basic_output":
            pin_a = int(self.get_parameter("pump_pin_a").value)
            pin_b = int(self.get_parameter("pump_pin_b").value)
            if action == "pump_on":
                a_val = int(self.get_parameter("pump_on_a").value)
                b_val = int(self.get_parameter("pump_on_b").value)
            elif action == "pump_off":
                a_val = int(self.get_parameter("pump_off_a").value)
                b_val = int(self.get_parameter("pump_off_b").value)
            else:
                return
            self._robot.set_basic_output(pin_a, a_val)
            self._robot.set_basic_output(pin_b, b_val)
            return

        if mode == "gripper":
            speed = int(self.get_parameter("gripper_speed").value)
            if action == "pump_on":
                self._robot.set_gripper_state(0, speed)
            elif action == "pump_off":
                self._robot.set_gripper_state(1, speed)

    def _default_phases(self, request: dict) -> list:
        motion_plan = request.get("motion_plan", [])
        if isinstance(motion_plan, list) and motion_plan:
            phases = []
            for step in motion_plan:
                if not isinstance(step, dict):
                    continue
                name = str(step.get("name", "motion_step"))
                phases.append((name, step))
            if phases:
                return phases

        pick_pose = request.get("pick_pose")
        if isinstance(pick_pose, dict):
            return [("move_pick", {"pose": pick_pose})]
        return []

    def _run_sequence(self, request_id: str, request: dict) -> None:
        phases = self._default_phases(request)
        if not phases:
            self._publish_status(request_id, "invalid", False, "No executable pose/action found.", request)
            with self._lock:
                self._busy = False
            return

        try:
            for state, step in phases:
                if "pose" in step and isinstance(step["pose"], dict):
                    pose = step["pose"]
                    detail = (
                        f"Move to x={float(pose.get('x', 0.0)):.3f}, "
                        f"y={float(pose.get('y', 0.0)):.3f}, z={float(pose.get('z', 0.0)):.3f}"
                    )
                    self._publish_status(request_id, state, True, detail)
                    self._move_pose(pose)
                elif "action" in step:
                    action = str(step["action"])
                    self._publish_status(request_id, state, True, f"Tool action: {action}")
                    self._tool_action(action)
                else:
                    self._publish_status(request_id, state, True, "Skipping unknown step format.")
                time.sleep(self.default_pause_sec)

            self._publish_status(request_id, "completed", True, "Real pick cycle complete.")
        except Exception as exc:
            self._publish_status(request_id, "failed", False, f"Execution error: {exc}")
        finally:
            with self._lock:
                self._busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealPickBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

