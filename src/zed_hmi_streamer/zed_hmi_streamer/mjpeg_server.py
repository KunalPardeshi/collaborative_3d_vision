import threading
import time
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse
import uvicorn

latest_lock = threading.Lock()
latest_jpeg: Optional[bytes] = None

class ZedImageSub(Node):
    def __init__(self):
        super().__init__("zed_hmi_image_sub")
        self.bridge = CvBridge()

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.get_logger().info(f"Subscribing to: {topic}")

        self.sub = self.create_subscription(Image, topic, self.cb, 10)

        self.frames = 0
        self.t0 = time.time()

    def cb(self, msg: Image):
        global latest_jpeg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return

        with latest_lock:
            latest_jpeg = jpg.tobytes()

        self.frames += 1
        t1 = time.time()
        if t1 - self.t0 > 2.0:
            fps = self.frames / (t1 - self.t0)
            self.get_logger().info(f"Incoming FPS: {fps:.1f}")
            self.frames = 0
            self.t0 = t1

app = FastAPI()

def gen():
    boundary = b"--frame"
    while True:
        with latest_lock:
            jpg = latest_jpeg
        if jpg is None:
            time.sleep(0.02)
            continue
        yield boundary + b"\r\n"
        yield b"Content-Type: image/jpeg\r\n\r\n"
        yield jpg + b"\r\n"
        time.sleep(0.01)

@app.get("/stream")
def stream():
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/", response_class=HTMLResponse)
def root():
    return """
    <html>
      <head><title>ZED Stream</title></head>
      <body style="margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh;">
        <img src="/stream" style="max-width:100%;max-height:100%;"/>
      </body>
    </html>
    """

def main():
    rclpy.init()
    node = ZedImageSub()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    # IMPORTANT: 0.0.0.0 = accessible from other machines via Jetson IP
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
