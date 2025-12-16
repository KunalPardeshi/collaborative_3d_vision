#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloPoseNode(Node):
    def __init__(self):
        super().__init__("yolo_pose_node")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("annotated_topic", "/yolo/annotated")
        self.declare_parameter("model", "yolov8n-pose.pt")
        self.declare_parameter("imgsz", 960)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)

        self.bridge = CvBridge()
        self.model = YOLO(self.get_parameter("model").value)

        image_topic = self.get_parameter("image_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value

        self.sub = self.create_subscription(Image, image_topic, self.cb, 10)
        self.pub = self.create_publisher(Image, annotated_topic, 10)

        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(f"Publishing:  {annotated_topic}")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model.predict(
            frame,
            imgsz=int(self.get_parameter("imgsz").value),
            conf=float(self.get_parameter("conf").value),
            iou=float(self.get_parameter("iou").value),
            verbose=False
        )

        annotated = results[0].plot()
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)

def main():
    rclpy.init()
    node = YoloPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
