import cv2
import json
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class BlockDetector(Node):
    def __init__(self):
        super().__init__("block_detector")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("detections_topic", "/blocks/detections")
        self.declare_parameter("publish_detections", True)
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("min_area", 500)
        self.declare_parameter("show_window", True)

        self.bridge = CvBridge()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.publish_detections = bool(self.get_parameter("publish_detections").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.last_publish_time = 0.0

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.detection_pub = self.create_publisher(String, self.detections_topic, 10)

        self.get_logger().info(
            f"Block detector started image_topic={self.image_topic} detections_topic={self.detections_topic}"
        )

    def detect_color_blocks(self, hsv, frame, color_name, lower1, upper1, lower2=None, upper2=None):
        mask1 = cv2.inRange(hsv, lower1, upper1)

        if lower2 is not None and upper2 is not None:
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = mask1

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2
            cy = y + h // 2

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

            label = f"{color_name} block"
            cv2.putText(
                frame,
                label,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            detections.append(
                {
                    "color": color_name.lower(),
                    "cx": float(cx),
                    "cy": float(cy),
                    "area": float(area),
                    "bbox": {"x": int(x), "y": int(y), "w": int(w), "h": int(h)},
                }
            )

        return detections

    def publish_detections_if_due(self, frame, detections):
        if not self.publish_detections:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_publish_time) < self.publish_period_sec:
            return

        payload = {
            "timestamp_ns": self.get_clock().now().nanoseconds,
            "image_width": int(frame.shape[1]),
            "image_height": int(frame.shape[0]),
            "num_detections": len(detections),
            "detections": detections,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.detection_pub.publish(msg)
        self.last_publish_time = now
        if detections:
            best = max(detections, key=lambda d: d["area"])
            self.get_logger().info(
                f"Published {len(detections)} detections; best={best['color']} @ ({best['cx']:.0f}, {best['cy']:.0f})"
            )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        all_detections = []

        # Red (2 ranges because HSV red wraps around)
        all_detections.extend(self.detect_color_blocks(
            hsv, frame, "Red",
            (0, 120, 70), (10, 255, 255),
            (170, 120, 70), (180, 255, 255)
        ))

        # Blue
        all_detections.extend(self.detect_color_blocks(
            hsv, frame, "Blue",
            (100, 120, 70), (130, 255, 255)
        ))

        # Yellow
        all_detections.extend(self.detect_color_blocks(
            hsv, frame, "Yellow",
            (20, 120, 70), (35, 255, 255)
        ))

        # Green
        all_detections.extend(self.detect_color_blocks(
            hsv, frame, "Green",
            (40, 70, 70), (85, 255, 255)
        ))

        self.publish_detections_if_due(frame, all_detections)

        if self.show_window:
            cv2.imshow("Block Detection", frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BlockDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
