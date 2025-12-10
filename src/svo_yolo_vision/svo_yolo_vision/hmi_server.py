import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from flask import Flask, Response, render_template_string

HTML_PAGE = """
<!doctype html>
<html>
<head>
  <title>SVO YOLO HMI</title>
  <style>
    body { background: #111; color: #eee; text-align: center; font-family: sans-serif; }
    img { max-width: 90vw; border: 2px solid #444; margin-top: 20px; }
  </style>
</head>
<body>
  <h1>SVO YOLO Object Detection</h1>
  <p>Streaming annotated frames from ROS2</p>
  <img src="/stream" />
</body>
</html>
"""

class ImageBuffer(Node):
    def __init__(self):
        super().__init__('hmi_image_buffer')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()

        self.subscription = self.create_subscription(
            Image,
            '/svo_yolo/annotated_image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.latest_frame = frame

def ros_spin(node):
    rclpy.spin(node)

def create_app(image_buffer: ImageBuffer):
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template_string(HTML_PAGE)

    def gen():
        while True:
            with image_buffer.lock:
                frame = image_buffer.latest_frame.copy() if image_buffer.latest_frame is not None else None
            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            else:
                # No frame yet; small sleep to avoid busy loop
                import time
                time.sleep(0.05)

    @app.route('/stream')
    def stream():
        return Response(gen(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    return app

def main(args=None):
    rclpy.init(args=args)
    image_buffer = ImageBuffer()

    # Run ROS2 in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(image_buffer,), daemon=True)
    ros_thread.start()

    # Run Flask
    app = create_app(image_buffer)
    # 0.0.0.0 so you can open from another PC on the network if needed
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    image_buffer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
