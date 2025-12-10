import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np

from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('rgb_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/camera_info')
        self.declare_parameter('use_depth', True)
        self.declare_parameter('model_path', 'yolov8n.pt')  # change to pose/medium etc if you want

        self.use_depth = self.get_parameter('use_depth').get_parameter_value().bool_value
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded')

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        # Subscribers (using message_filters for sync)
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)

        if self.use_depth:
            self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_sub, self.depth_sub, self.info_sub],
                queue_size=10,
                slop=0.1
            )
            self.sync.registerCallback(self.rgb_depth_info_cb)
            self.get_logger().info('YoloDetector in DEPTH mode')
        else:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_sub, self.info_sub],
                queue_size=10,
                slop=0.1
            )
            self.sync.registerCallback(self.rgb_info_cb)
            self.get_logger().info('YoloDetector in RGB-ONLY mode')

        # Publisher: we’ll publish an annotated image for the HMI node
        self.image_pub = self.create_publisher(Image, '/svo_yolo/annotated_image', 10)

    def update_intrinsics(self, info_msg: CameraInfo):
        if self.fx is None:
            K = info_msg.k
            self.fx = K[0]
            self.fy = K[4]
            self.cx = K[2]
            self.cy = K[5]
            self.get_logger().info(f'Camera intrinsics set: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')

    def rgb_depth_info_cb(self, rgb_msg, depth_msg, info_msg):
        self.process(rgb_msg, info_msg, depth_msg)

    def rgb_info_cb(self, rgb_msg, info_msg):
        self.process(rgb_msg, info_msg, None)

    def process(self, rgb_msg, info_msg, depth_msg):
        self.update_intrinsics(info_msg)

        # Convert RGB
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

        # Convert depth if available
        depth = None
        if depth_msg is not None:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')  # float32 depth in meters

        # Run YOLO
        results = self.model(frame, verbose=False)[0]

        # Draw detections
        if results.boxes is not None:
            boxes = results.boxes
            for box in boxes:
                cls_id = int(box.cls)
                label = self.model.names[cls_id]
                conf = float(box.conf)

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                # Bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Depth-based distance (approx at box center)
                center_u = int((x1 + x2) / 2)
                center_v = int((y1 + y2) / 2)

                distance_str = ''
                if depth is not None and 0 <= center_v < depth.shape[0] and 0 <= center_u < depth.shape[1]:
                    Z = float(depth[center_v, center_u])
                    if np.isfinite(Z) and Z > 0:
                        distance_str = f'{Z:.2f}m'
                    else:
                        distance_str = 'NaN'
                else:
                    distance_str = 'no depth'

                text = f'{label} {conf:.2f} {distance_str}'
                cv2.putText(frame, text, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish annotated frame
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = rgb_msg.header
        self.image_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
