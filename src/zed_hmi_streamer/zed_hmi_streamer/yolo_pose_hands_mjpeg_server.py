import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

import mediapipe as mp
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse
import uvicorn

latest_lock = threading.Lock()
latest_jpeg: Optional[bytes] = None


def clamp_roi(x1, y1, x2, y2, w, h) -> Tuple[int, int, int, int]:
    x1 = max(0, min(w - 1, int(x1)))
    y1 = max(0, min(h - 1, int(y1)))
    x2 = max(0, min(w, int(x2)))
    y2 = max(0, min(h, int(y2)))
    if x2 <= x1 + 2: x2 = min(w, x1 + 3)
    if y2 <= y1 + 2: y2 = min(h, y1 + 3)
    return x1, y1, x2, y2


class PoseHandsStreamer(Node):
    """
    YOLO Pose for body joints (runs every N frames),
    MediaPipe Hands for fingers (runs on wrist ROIs).
    """
    def __init__(self):
        super().__init__("zed_yolo_pose_hands_hmi")
        self.bridge = CvBridge()

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("pose_model", "yolo11n-pose.pt")
        self.declare_parameter("pose_imgsz", 320)
        self.declare_parameter("pose_conf", 0.35)
        self.declare_parameter("pose_every", 5)      # run YOLO pose every N frames
        self.declare_parameter("hand_pad", 90)       # pixels around wrist
        self.declare_parameter("max_hands", 4)

        self.topic = self.get_parameter("image_topic").value
        self.pose_model_path = self.get_parameter("pose_model").value
        self.pose_imgsz = int(self.get_parameter("pose_imgsz").value)
        self.pose_conf = float(self.get_parameter("pose_conf").value)
        self.pose_every = int(self.get_parameter("pose_every").value)
        self.hand_pad = int(self.get_parameter("hand_pad").value)
        self.max_hands = int(self.get_parameter("max_hands").value)

        self.get_logger().info(f"Subscribing: {self.topic}")
        self.get_logger().info(f"Pose model: {self.pose_model_path} | imgsz={self.pose_imgsz} conf={self.pose_conf} every={self.pose_every}")

        self.pose_model = YOLO(self.pose_model_path)

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=self.max_hands,
            model_complexity=0,               # fastest
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.sub = self.create_subscription(Image, self.topic, self.cb, 10)

        self.frame_i = 0
        self.last_pose = None  # cache last YOLO pose result

        self.frames = 0
        self.t0 = time.time()

    def cb(self, msg: Image):
        global latest_jpeg
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame_bgr.shape[:2]
        out = frame_bgr.copy()

        # --- YOLO pose (throttled) ---
        self.frame_i += 1
        if (self.frame_i % self.pose_every) == 1 or self.last_pose is None:
            self.last_pose = self.pose_model.predict(
                source=frame_bgr,
                imgsz=self.pose_imgsz,
                conf=self.pose_conf,
                verbose=False
            )[0]

        pose_res = self.last_pose

        # Draw pose skeleton quickly using Ultralytics plot (ok, but not free)
        # If this is too slow, we can draw only keypoints ourselves.
        try:
            out = pose_res.plot()
        except Exception:
            pass

        # --- Get wrist positions and run MediaPipe Hands on ROIs ---
        # YOLO pose keypoint order (COCO): left_wrist=9, right_wrist=10
        rois = []
        try:
            kxy = pose_res.keypoints.xy  # shape: (n, k, 2)
            kxy = kxy.cpu().numpy() if hasattr(kxy, "cpu") else np.array(kxy)
            for person in kxy:
                lw = person[9]   # (x,y)
                rw = person[10]
                for (x, y) in (lw, rw):
                    if x <= 0 or y <= 0:
                        continue
                    x1, y1, x2, y2 = clamp_roi(x - self.hand_pad, y - self.hand_pad, x + self.hand_pad, y + self.hand_pad, w, h)
                    rois.append((x1, y1, x2, y2))
        except Exception:
            rois = []

        # Deduplicate ROIs a bit (optional)
        rois = rois[:8]  # safety cap

        for (x1, y1, x2, y2) in rois:
            crop_bgr = frame_bgr[y1:y2, x1:x2]
            if crop_bgr.size == 0:
                continue
            crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)

            res = self.hands.process(crop_rgb)
            if not res.multi_hand_landmarks:
                continue

            # draw landmarks back on output image
            for hand_lms in res.multi_hand_landmarks:
                # draw on crop first
                crop_draw = out[y1:y2, x1:x2]
                crop_rgb2 = cv2.cvtColor(crop_draw, cv2.COLOR_BGR2RGB)
                self.mp_draw.draw_landmarks(
                    crop_rgb2,
                    hand_lms,
                    self.mp_hands.HAND_CONNECTIONS
                )
                out[y1:y2, x1:x2] = cv2.cvtColor(crop_rgb2, cv2.COLOR_RGB2BGR)

            # optional ROI rectangle
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 1)

        ok, jpg = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with latest_lock:
                latest_jpeg = jpg.tobytes()

        self.frames += 1
        t1 = time.time()
        if t1 - self.t0 > 2.0:
            fps = self.frames / (t1 - self.t0)
            self.get_logger().info(f"Pose+Hands stream FPS: {fps:.2f}")
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
      <head><title>ZED + Pose + Fingers</title></head>
      <body style="margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh;">
        <img src="/stream" style="max-width:100%;max-height:100%;"/>
      </body>
    </html>
    """

def main():
    rclpy.init()
    node = PoseHandsStreamer()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    uvicorn.run(app, host="0.0.0.0", port=8092, log_level="warning")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
