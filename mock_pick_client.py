#!/usr/bin/env python3
"""
Send one mock pick request and print status updates.
"""

import argparse
import json
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TERMINAL_STATES = {"completed", "failed", "rejected", "invalid"}


class MockPickClient(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mock_pick_client")

        self.request_topic = args.request_topic
        self.status_topic = args.status_topic
        self.wait_sec = args.wait_sec
        self.request_id = args.request_id or f"req-{uuid.uuid4().hex[:8]}"
        self.done = False
        self.success = False
        self._sent = False
        self._start = time.monotonic()

        self.pub = self.create_publisher(String, self.request_topic, 10)
        self.sub = self.create_subscription(String, self.status_topic, self._on_status, 10)
        self.create_timer(0.2, self._tick)

        self.request = {
            "request_id": self.request_id,
            "color": args.color,
            "pick_pose": {
                "x": args.x,
                "y": args.y,
                "z": args.z,
                "yaw": args.yaw,
            },
            "place": args.place,
        }
        self.get_logger().info(
            f"Prepared request_id={self.request_id} request_topic={self.request_topic} status_topic={self.status_topic}"
        )

    def _tick(self) -> None:
        if not self._sent:
            msg = String()
            msg.data = json.dumps(self.request)
            self.pub.publish(msg)
            self._sent = True
            self.get_logger().info(f"Sent request: {msg.data}")

        if (time.monotonic() - self._start) > self.wait_sec and not self.done:
            self.done = True
            self.success = False
            self.get_logger().error(f"Timeout after {self.wait_sec}s waiting for terminal status.")

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Non-JSON status ignored: {msg.data}")
            return

        if payload.get("request_id") != self.request_id:
            return

        state = str(payload.get("state", "unknown"))
        detail = str(payload.get("detail", ""))
        ok = bool(payload.get("ok", False))
        self.get_logger().info(f"Status: state={state} ok={ok} detail={detail}")

        if state in TERMINAL_STATES:
            self.done = True
            self.success = state == "completed" and ok


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send one mock pick request.")
    parser.add_argument("--request-topic", default="/pick/request")
    parser.add_argument("--status-topic", default="/pick/status")
    parser.add_argument("--request-id", default="")
    parser.add_argument("--color", default="red")
    parser.add_argument("--x", type=float, default=0.20)
    parser.add_argument("--y", type=float, default=0.00)
    parser.add_argument("--z", type=float, default=0.03)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--place", default="red_bin")
    parser.add_argument("--wait-sec", type=float, default=12.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MockPickClient(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        success = node.success
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
