#!/usr/bin/env python3
"""
Pickup + path planning ROS 2 node.

Subscribes:
  /blocks/detections (std_msgs/String JSON)
  /pick/status       (std_msgs/String JSON)

Publishes:
  /pick/request      (std_msgs/String JSON)
  /pick/plan         (std_msgs/String JSON)
  /visualization_marker (Marker)

This node chooses a target block, maps image pixel to robot XY, builds a
pick/place motion plan, and publishes an executable request for the backend.
"""

import json
import time
from datetime import datetime, timezone
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class PickupPathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("pickup_path_planner")
        self.declare_parameter("detection_topic", "/blocks/detections")
        self.declare_parameter("request_topic", "/pick/request")
        self.declare_parameter("status_topic", "/pick/status")
        self.declare_parameter("plan_topic", "/pick/plan")
        self.declare_parameter("target_color", "red")
        self.declare_parameter("cooldown_sec", 3.0)
        self.declare_parameter("require_single_target", False)
        self.declare_parameter("marker_topic", "/visualization_marker")
        self.declare_parameter("marker_frame", "g_base")
        self.declare_parameter("publish_bin_markers", True)

        # Camera pixel -> robot XY workspace mapping.
        self.declare_parameter("x_min", 0.10)
        self.declare_parameter("x_max", 0.30)
        self.declare_parameter("y_min", -0.12)
        self.declare_parameter("y_max", 0.12)

        # Designated place bins.
        self.declare_parameter("red_bin_x", 0.16)
        self.declare_parameter("red_bin_y", -0.16)
        self.declare_parameter("green_bin_x", 0.21)
        self.declare_parameter("green_bin_y", -0.16)
        self.declare_parameter("blue_bin_x", 0.26)
        self.declare_parameter("blue_bin_y", -0.16)
        self.declare_parameter("yellow_bin_x", 0.31)
        self.declare_parameter("yellow_bin_y", -0.16)

        # Path profile.
        self.declare_parameter("z_pick", 0.030)
        self.declare_parameter("z_approach", 0.110)
        self.declare_parameter("z_lift", 0.140)
        self.declare_parameter("z_place", 0.060)
        self.declare_parameter("yaw", 0.0)

        self._last_sent_time = 0.0
        self._active_request_id: Optional[str] = None

        detection_topic = str(self.get_parameter("detection_topic").value)
        request_topic = str(self.get_parameter("request_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        plan_topic = str(self.get_parameter("plan_topic").value)
        self.target_color = str(self.get_parameter("target_color").value).strip().lower()
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.require_single_target = bool(self.get_parameter("require_single_target").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.publish_bin_markers = bool(self.get_parameter("publish_bin_markers").value)

        self.request_pub = self.create_publisher(String, request_topic, 10)
        self.plan_pub = self.create_publisher(String, plan_topic, 10)
        self.marker_pub = self.create_publisher(Marker, str(self.get_parameter("marker_topic").value), 10)
        self.detection_sub = self.create_subscription(String, detection_topic, self._on_detection, 10)
        self.status_sub = self.create_subscription(String, status_topic, self._on_status, 10)
        self.get_logger().info(
            "PickupPathPlanner ready "
            f"detection_topic={detection_topic} request_topic={request_topic} plan_topic={plan_topic} "
            f"target_color={self.target_color}"
        )
        if self.publish_bin_markers:
            self.create_timer(1.0, self._publish_bin_markers)

    def _pixel_to_robot_xy(self, cx: float, cy: float, width: int, height: int) -> Tuple[float, float]:
        x_min = float(self.get_parameter("x_min").value)
        x_max = float(self.get_parameter("x_max").value)
        y_min = float(self.get_parameter("y_min").value)
        y_max = float(self.get_parameter("y_max").value)

        px = max(0.0, min(1.0, cx / max(1, width)))
        py = max(0.0, min(1.0, cy / max(1, height)))
        robot_x = x_min + px * (x_max - x_min)
        robot_y = y_max - py * (y_max - y_min)
        return robot_x, robot_y

    def _destination_for_color(self, color: str) -> Dict[str, float]:
        bins = {
            "red": {"x": float(self.get_parameter("red_bin_x").value), "y": float(self.get_parameter("red_bin_y").value)},
            "green": {
                "x": float(self.get_parameter("green_bin_x").value),
                "y": float(self.get_parameter("green_bin_y").value),
            },
            "blue": {"x": float(self.get_parameter("blue_bin_x").value), "y": float(self.get_parameter("blue_bin_y").value)},
            "yellow": {
                "x": float(self.get_parameter("yellow_bin_x").value),
                "y": float(self.get_parameter("yellow_bin_y").value),
            },
        }
        return bins.get(color, {"x": 0.20, "y": -0.16})

    def _build_motion_plan(self, pick_pose: Dict[str, float], place_xy: Dict[str, float]) -> List[dict]:
        z_approach = float(self.get_parameter("z_approach").value)
        z_lift = float(self.get_parameter("z_lift").value)
        z_place = float(self.get_parameter("z_place").value)
        yaw = float(self.get_parameter("yaw").value)

        return [
            {"name": "approach_pick", "pose": {"x": pick_pose["x"], "y": pick_pose["y"], "z": z_approach, "yaw": yaw}},
            {"name": "descend_pick", "pose": pick_pose},
            {"name": "grasp", "action": "pump_on"},
            {"name": "lift", "pose": {"x": pick_pose["x"], "y": pick_pose["y"], "z": z_lift, "yaw": yaw}},
            {
                "name": "move_to_place",
                "pose": {"x": place_xy["x"], "y": place_xy["y"], "z": z_approach, "yaw": yaw},
            },
            {
                "name": "descend_place",
                "pose": {"x": place_xy["x"], "y": place_xy["y"], "z": z_place, "yaw": yaw},
            },
            {"name": "release", "action": "pump_off"},
            {"name": "retreat", "pose": {"x": place_xy["x"], "y": place_xy["y"], "z": z_lift, "yaw": yaw}},
        ]

    def _choose_target(self, detections: List[dict]) -> Optional[dict]:
        if not detections:
            return None
        matches = [d for d in detections if str(d.get("color", "")).lower() == self.target_color]
        if matches:
            return max(matches, key=lambda d: float(d.get("area", 0.0)))
        if self.require_single_target:
            return None
        return max(detections, key=lambda d: float(d.get("area", 0.0)))

    def _publish_pick_marker(self, color: str, pick_pose: Dict[str, float]) -> None:
        color_rgba = {
            "red": (1.0, 0.1, 0.1, 0.95),
            "green": (0.1, 1.0, 0.1, 0.95),
            "blue": (0.1, 0.4, 1.0, 0.95),
            "yellow": (1.0, 1.0, 0.1, 0.95),
        }.get(color, (0.9, 0.9, 0.9, 0.95))

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.marker_frame
        marker.ns = "detected_blocks"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(pick_pose["x"])
        marker.pose.position.y = float(pick_pose["y"])
        marker.pose.position.z = float(pick_pose["z"]) / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = float(pick_pose["z"])
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color_rgba
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.marker_pub.publish(marker)

    def _publish_destination_marker(self, color: str, place_xy: Dict[str, float]) -> None:
        color_rgba = {
            "red": (1.0, 0.1, 0.1, 0.95),
            "green": (0.1, 1.0, 0.1, 0.95),
            "blue": (0.1, 0.4, 1.0, 0.95),
            "yellow": (1.0, 1.0, 0.1, 0.95),
        }.get(color, (0.9, 0.9, 0.9, 0.95))

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.marker_frame
        marker.ns = "designated_bin_target"
        marker.id = 2
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(place_xy["x"])
        marker.pose.position.y = float(place_xy["y"])
        marker.pose.position.z = 0.01
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.02
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color_rgba
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.marker_pub.publish(marker)

    def _publish_bin_markers(self) -> None:
        bins = [
            ("red", self._destination_for_color("red"), (1.0, 0.1, 0.1, 0.55), 101),
            ("green", self._destination_for_color("green"), (0.1, 1.0, 0.1, 0.55), 102),
            ("blue", self._destination_for_color("blue"), (0.1, 0.4, 1.0, 0.55), 103),
            ("yellow", self._destination_for_color("yellow"), (1.0, 1.0, 0.1, 0.55), 104),
        ]
        for _, pose, rgba, marker_id in bins:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self.marker_frame
            marker.ns = "designated_bins"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pose["x"])
            marker.pose.position.y = float(pose["y"])
            marker.pose.position.z = 0.01
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.02
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            self.marker_pub.publish(marker)

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        rid = str(payload.get("request_id", ""))
        state = str(payload.get("state", ""))
        if self._active_request_id and rid == self._active_request_id:
            if state in {"completed", "failed", "invalid", "rejected"}:
                self._active_request_id = None

    def _on_detection(self, msg: String) -> None:
        if self._active_request_id is not None:
            return

        now = time.time()
        if now - self._last_sent_time < self.cooldown_sec:
            return

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Ignoring invalid JSON detection payload.")
            return

        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            return

        target = self._choose_target(detections)
        if target is None:
            return

        width = int(payload.get("image_width", 0))
        height = int(payload.get("image_height", 0))
        cx = float(target.get("cx", 0.0))
        cy = float(target.get("cy", 0.0))
        color = str(target.get("color", "unknown")).lower()

        pick_x, pick_y = self._pixel_to_robot_xy(cx, cy, width, height)
        pick_pose = {
            "x": round(pick_x, 4),
            "y": round(pick_y, 4),
            "z": float(self.get_parameter("z_pick").value),
            "yaw": float(self.get_parameter("yaw").value),
        }
        place_xy = self._destination_for_color(color)
        motion_plan = self._build_motion_plan(pick_pose, place_xy)

        request_id = f"auto-{int(now * 1000)}"
        place_pose = {
            "x": float(place_xy["x"]),
            "y": float(place_xy["y"]),
            "z": float(self.get_parameter("z_place").value),
            "yaw": float(self.get_parameter("yaw").value),
        }
        request = {
            "request_id": request_id,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "source": "pickup_path_planner_node",
            "color": color,
            "pick_pose": pick_pose,
            "place": f"{color}_bin",
            "place_pose": place_pose,
            "motion_plan": motion_plan,
            "vision": {
                "cx": cx,
                "cy": cy,
                "area": float(target.get("area", 0.0)),
                "image_width": width,
                "image_height": height,
            },
        }

        # Publish plan for observability and request for execution.
        plan_msg = String()
        plan_msg.data = json.dumps(
            {
                "request_id": request_id,
                "color": color,
                "pick_pose": pick_pose,
                "place_pose": place_pose,
                "motion_plan": motion_plan,
            }
        )
        self.plan_pub.publish(plan_msg)

        request_msg = String()
        request_msg.data = json.dumps(request)
        self.request_pub.publish(request_msg)

        self._publish_pick_marker(color, pick_pose)
        self._publish_destination_marker(color, place_xy)
        self._active_request_id = request_id
        self._last_sent_time = now
        self.get_logger().info(
            f"Planned + sent request color={color} pixel=({cx:.1f},{cy:.1f}) "
            f"pick=({pick_pose['x']:.3f},{pick_pose['y']:.3f}) "
            f"place=({place_xy['x']:.3f},{place_xy['y']:.3f})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickupPathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

