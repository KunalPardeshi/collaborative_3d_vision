import threading
import time
from typing import Optional

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


def mjpeg_gen(getter_fn):
    boundary = b"--frame"
    while True:
        jpg = getter_fn()
        if jpg is None:
            time.sleep(0.02)
            continue
        yield boundary + b"\r\n"
        yield b"Content-Type: image/jpeg\r\n\r\n"
        yield jpg + b"\r\n"
        time.sleep(0.01)


class CombinedHMI(Node):
    def __init__(self):
        super().__init__("combined_hmi")

        self.bridge = CvBridge()

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")

        # speed knobs
        self.declare_parameter("seg_imgsz", 512)
        self.declare_parameter("seg_conf", 0.25)

        self.declare_parameter("pose_imgsz", 320)
        self.declare_parameter("pose_conf", 0.35)
        self.declare_parameter("pose_every", 6)  # throttle pose inference

        self.declare_parameter("hand_pad", 80)
        self.declare_parameter("max_hands", 4)

        self.topic = self.get_parameter("image_topic").value
        self.seg_imgsz = int(self.get_parameter("seg_imgsz").value)
        self.seg_conf = float(self.get_parameter("seg_conf").value)

        self.pose_imgsz = int(self.get_parameter("pose_imgsz").value)
        self.pose_conf = float(self.get_parameter("pose_conf").value)
        self.pose_every = int(self.get_parameter("pose_every").value)

        self.hand_pad = int(self.get_parameter("hand_pad").value)
        self.max_hands = int(self.get_parameter("max_hands").value)

        # latest jpegs
        self._lock = threading.Lock()
        self._raw_jpg: Optional[bytes] = None
        self._seg_jpg: Optional[bytes] = None
        self._pose_jpg: Optional[bytes] = None
        self._hands_jpg: Optional[bytes] = None

        # models
        self.get_logger().info("Loading YOLO seg + pose + MediaPipe hands...")
        self.seg_model = YOLO("yolo11n-seg.pt")
        self.pose_model = YOLO("yolo11n-pose.pt")

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=self.max_hands,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.frame_i = 0
        self.last_pose = None

        self.sub = self.create_subscription(Image, self.topic, self.cb, 10)

        self._fps_frames = 0
        self._fps_t0 = time.time()

    def _set(self, which: str, jpg: bytes):
        with self._lock:
            setattr(self, which, jpg)

    def get_raw(self):
        with self._lock:
            return self._raw_jpg

    def get_seg(self):
        with self._lock:
            return self._seg_jpg

    def get_pose(self):
        with self._lock:
            return self._pose_jpg

    def get_hands(self):
        with self._lock:
            return self._hands_jpg

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        # RAW
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            self._set("_raw_jpg", jpg.tobytes())

        # SEG (small objects)
        try:
            seg_res = self.seg_model.predict(frame, imgsz=self.seg_imgsz, conf=self.seg_conf, verbose=False)[0]
            seg_out = seg_res.plot()
            ok, jpg = cv2.imencode(".jpg", seg_out, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ok:
                self._set("_seg_jpg", jpg.tobytes())
        except Exception:
            pass

        # POSE (throttled)
        self.frame_i += 1
        if (self.frame_i % self.pose_every) == 1 or self.last_pose is None:
            try:
                self.last_pose = self.pose_model.predict(frame, imgsz=self.pose_imgsz, conf=self.pose_conf, verbose=False)[0]
            except Exception:
                self.last_pose = None

        pose_out = frame
        wrist_rois = []
        if self.last_pose is not None:
            try:
                pose_out = self.last_pose.plot()
            except Exception:
                pose_out = frame

            # wrist ROIs for hands: left wrist=9, right wrist=10
            try:
                kxy = self.last_pose.keypoints.xy
                kxy = kxy.cpu().numpy() if hasattr(kxy, "cpu") else np.array(kxy)
                for person in kxy:
                    for (x, y) in (person[9], person[10]):
                        if x <= 0 or y <= 0:
                            continue
                        x1 = max(0, int(x - self.hand_pad))
                        y1 = max(0, int(y - self.hand_pad))
                        x2 = min(w, int(x + self.hand_pad))
                        y2 = min(h, int(y + self.hand_pad))
                        if x2 > x1 + 5 and y2 > y1 + 5:
                            wrist_rois.append((x1, y1, x2, y2))
            except Exception:
                wrist_rois = []

        # publish pose jpg
        ok, jpg = cv2.imencode(".jpg", pose_out, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            self._set("_pose_jpg", jpg.tobytes())

        # HANDS (fingers) on ROIs
        hands_out = pose_out.copy() if isinstance(pose_out, np.ndarray) else frame.copy()
        for (x1, y1, x2, y2) in wrist_rois[:8]:
            crop_bgr = frame[y1:y2, x1:x2]
            if crop_bgr.size == 0:
                continue
            crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
            res = self.hands.process(crop_rgb)
            if not res.multi_hand_landmarks:
                continue

            # draw landmarks on the output in-place
            crop_draw = hands_out[y1:y2, x1:x2]
            crop_rgb2 = cv2.cvtColor(crop_draw, cv2.COLOR_BGR2RGB)
            for hand_lms in res.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(crop_rgb2, hand_lms, self.mp_hands.HAND_CONNECTIONS)
            hands_out[y1:y2, x1:x2] = cv2.cvtColor(crop_rgb2, cv2.COLOR_RGB2BGR)
            cv2.rectangle(hands_out, (x1, y1), (x2, y2), (0, 255, 255), 1)

        ok, jpg = cv2.imencode(".jpg", hands_out, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            self._set("_hands_jpg", jpg.tobytes())

        # log overall callback rate
        self._fps_frames += 1
        t1 = time.time()
        if t1 - self._fps_t0 > 2.0:
            fps = self._fps_frames / (t1 - self._fps_t0)
            self.get_logger().info(f"Combined pipeline callback FPS: {fps:.2f}")
            self._fps_frames = 0
            self._fps_t0 = t1


app = FastAPI()
node_ref = {"node": None}

@app.get("/", response_class=HTMLResponse)
def dashboard():
    return """
    <html>
      <head>
        <title>Combined HMI</title>
        <style>
          body { margin:0; background:#111; color:#eee; font-family: sans-serif; }
          .grid { display:grid; grid-template-columns: 1fr 1fr; gap:10px; padding:10px; }
          .card { background:#1b1b1b; padding:10px; border-radius:12px; }
          img { width:100%; border-radius:10px; }
        </style>
      </head>
      <body>
        <div class="grid">
          <div class="card"><h3>Raw</h3><img src="/raw/stream"></div>
          <div class="card"><h3>Small Objects (Seg)</h3><img src="/seg/stream"></div>
          <div class="card"><h3>Pose</h3><img src="/pose/stream"></div>
          <div class="card"><h3>Pose + Fingers</h3><img src="/hands/stream"></div>
        </div>
      </body>
    </html>
    """

@app.get("/raw/stream")
def raw_stream():
    n = node_ref["node"]
    return StreamingResponse(mjpeg_gen(n.get_raw), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/seg/stream")
def seg_stream():
    n = node_ref["node"]
    return StreamingResponse(mjpeg_gen(n.get_seg), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/pose/stream")
def pose_stream():
    n = node_ref["node"]
    return StreamingResponse(mjpeg_gen(n.get_pose), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/hands/stream")
def hands_stream():
    n = node_ref["node"]
    return StreamingResponse(mjpeg_gen(n.get_hands), media_type="multipart/x-mixed-replace; boundary=frame")


def main():
    rclpy.init()
    node = CombinedHMI()
    node_ref["node"] = node

    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    uvicorn.run(app, host="0.0.0.0", port=8085, log_level="warning")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
