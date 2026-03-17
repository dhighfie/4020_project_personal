#!/usr/bin/env python3
"""
Publish a still image as a ROS camera feed.
"""

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageReplayCamera(Node):
    def __init__(self) -> None:
        super().__init__("image_replay_camera")
        self.declare_parameter(
            "image_path",
            str(Path(__file__).resolve().parent / "assets" / "aikit_280pi_block_sample.png"),
        )
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("fps", 4.0)
        self.declare_parameter("stamp_overlay", False)

        self.image_path = Path(str(self.get_parameter("image_path").value)).expanduser().resolve()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.fps = float(self.get_parameter("fps").value)
        self.stamp_overlay = bool(self.get_parameter("stamp_overlay").value)
        if self.fps <= 0:
            raise ValueError("fps must be > 0.")

        self.frame = cv2.imread(str(self.image_path), cv2.IMREAD_COLOR)
        if self.frame is None:
            raise FileNotFoundError(f"Could not load image: {self.image_path}")

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.timer = self.create_timer(1.0 / self.fps, self._tick)
        self.get_logger().info(
            f"Publishing {self.image_path} to {self.image_topic} at {self.fps:.2f} FPS "
            f"({self.frame.shape[1]}x{self.frame.shape[0]})"
        )

    def _tick(self) -> None:
        frame = self.frame.copy()
        if self.stamp_overlay:
            stamp = self.get_clock().now().to_msg()
            text = f"{stamp.sec}.{stamp.nanosec:09d}"
            cv2.putText(frame, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageReplayCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
