#!/usr/bin/env python3
"""
Bridge mock pick requests to /joint_states so RViz can animate the robot.
"""

import json
import math
from collections import deque
from typing import Deque, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class RvizJointMotionBridge(Node):
    def __init__(self) -> None:
        super().__init__("rviz_joint_motion_bridge")
        self.declare_parameter("request_topic", "/pick/request")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("update_hz", 30.0)
        self.declare_parameter("max_speed_rad_s", 1.2)
        self.declare_parameter("arm_l1_m", 0.110)
        self.declare_parameter("arm_l2_m", 0.096)
        self.declare_parameter("base_height_m", 0.090)
        # Keep end effector top-down during pick/place.
        self.declare_parameter("tool_pitch_rad", -1.5708)
        self.declare_parameter("wrist_roll_rad", 0.0)

        self.joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]
        self.joint_limits = [
            (-2.9321, 2.9321),
            (-2.4434, 2.4434),
            (-2.6179, 2.6179),
            (-2.6179, 2.6179),
            (-2.7052, 2.7925),
            (-math.pi, math.pi),
        ]

        self.current = [0.0] * len(self.joint_names)
        self.target = list(self.current)
        self.queue: Deque[List[float]] = deque()

        request_topic = str(self.get_parameter("request_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.max_speed = float(self.get_parameter("max_speed_rad_s").value)

        self.joint_pub = self.create_publisher(JointState, joint_states_topic, 20)
        self.request_sub = self.create_subscription(String, request_topic, self._on_request, 10)
        self.timer = self.create_timer(1.0 / self.update_hz, self._tick)
        self.get_logger().info(
            f"RViz joint bridge active. request_topic={request_topic} joint_states_topic={joint_states_topic}"
        )

    def _pose_to_joints(self, pose: dict) -> List[float]:
        x = float(pose.get("x", 0.20))
        y = float(pose.get("y", 0.00))
        z = float(pose.get("z", 0.08))
        yaw = float(pose.get("yaw", 0.0))

        # Simple planar IK approximation for visualization only.
        l1 = float(self.get_parameter("arm_l1_m").value)
        l2 = float(self.get_parameter("arm_l2_m").value)
        base_h = float(self.get_parameter("base_height_m").value)

        q1 = math.atan2(y, x)
        r = max(0.001, math.hypot(x, y))
        z_rel = z - base_h
        d = math.hypot(r, z_rel)
        d = clamp(d, 0.02, l1 + l2 - 1e-4)

        cos_elbow = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        cos_elbow = clamp(cos_elbow, -1.0, 1.0)
        elbow = math.acos(cos_elbow)

        shoulder = math.atan2(z_rel, r) - math.atan2(l2 * math.sin(elbow), l1 + l2 * cos_elbow)
        q2 = shoulder
        q3 = -elbow
        tool_pitch = float(self.get_parameter("tool_pitch_rad").value)
        wrist_roll = float(self.get_parameter("wrist_roll_rad").value)
        # Wrist compensation enforces a near-constant tool pitch.
        q4 = tool_pitch - (q2 + q3)
        q5 = wrist_roll
        q6 = yaw

        out = [q1, q2, q3, q4, q5, q6]
        for i, (lo, hi) in enumerate(self.joint_limits):
            out[i] = clamp(out[i], lo, hi)
        return out

    def _on_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Ignoring invalid request JSON.")
            return

        motion_plan = payload.get("motion_plan", [])
        new_targets: List[List[float]] = []
        if isinstance(motion_plan, list):
            for step in motion_plan:
                if isinstance(step, dict) and isinstance(step.get("pose"), dict):
                    new_targets.append(self._pose_to_joints(step["pose"]))

        if not new_targets:
            pick_pose = payload.get("pick_pose")
            if isinstance(pick_pose, dict):
                new_targets.append(self._pose_to_joints(pick_pose))

        if not new_targets:
            return

        self.queue.clear()
        self.queue.extend(new_targets)
        self.target = self.queue.popleft()
        self.get_logger().info(f"Loaded {len(new_targets)} RViz joint waypoints from request.")

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current
        self.joint_pub.publish(msg)

    def _tick(self) -> None:
        dt = 1.0 / self.update_hz
        max_step = self.max_speed * dt

        done = True
        for i in range(len(self.current)):
            delta = self.target[i] - self.current[i]
            if abs(delta) > 0.01:
                done = False
                self.current[i] += clamp(delta, -max_step, max_step)

        if done and self.queue:
            self.target = self.queue.popleft()

        self._publish_joint_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizJointMotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
