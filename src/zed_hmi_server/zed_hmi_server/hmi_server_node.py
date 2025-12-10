#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from collab_vision_msgs.msg import AdvancedDetections  # <--- our custom detections

import cv2
from flask import Flask, Response, jsonify, request

# Flask app
app = Flask(__name__)
node_ref = None  # global reference to ROS node


class HMIServerNode(Node):
    def __init__(self):
        super().__init__("hmi_server")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_objects = []  # list of dicts
        self.lock = threading.Lock()

        # --- IMAGE TOPIC (ZED RGB) ---
        self.declare_parameter(
            "image_topic",
            "/zed/zed_node/rgb/color/rect/image"
        )
        image_topic = (
            self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
        )

        # --- DETECTIONS TOPIC (from dummy / future advanced detector) ---
        self.declare_parameter(
            "detections_topic",
            "/collab_vision/advanced_detections"
        )
        detections_topic = (
            self.get_parameter("detections_topic")
            .get_parameter_value()
            .string_value
        )

        # --- TARGET POSE TOPIC (for manipulator) ---
        self.declare_parameter(
            "target_topic",
            "/manipulator/target_pose"
        )
        target_topic = (
            self.get_parameter("target_topic")
            .get_parameter_value()
            .string_value
        )

        # Subscribers
        self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )

        self.create_subscription(
            AdvancedDetections,
            detections_topic,
            self.detections_callback,
            10,
        )

        # Publisher for target pose
        self.target_pub = self.create_publisher(
            PoseStamped,
            target_topic,
            10,
        )

        self.get_logger().info(
            f"[HMI] Subscribing to image:      {image_topic}"
        )
        self.get_logger().info(
            f"[HMI] Subscribing to detections: {detections_topic}"
        )
        self.get_logger().info(
            f"[HMI] Publishing target poses:   {target_topic}"
        )

    # ---------- ROS CALLBACKS ----------

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self.lock:
            self.latest_frame = cv_image

    def detections_callback(self, msg: AdvancedDetections):
        """Convert AdvancedDetections → list of Python dicts."""
        objs = []

        for o in msg.objects:
            # Collect keypoints for persons
            kps = []
            for kp in o.keypoints:
                kps.append({
                    "name": kp.name,
                    "u": float(kp.u),
                    "v": float(kp.v),
                    "x": float(kp.x),
                    "y": float(kp.y),
                    "z": float(kp.z),
                    "confidence": float(kp.confidence),
                })

            objs.append({
                "id": int(o.id),
                "label": o.label,
                "confidence": float(o.confidence),
                "bbox": [float(o.u_min), float(o.v_min),
                         float(o.u_max), float(o.v_max)],
                "position": [float(o.x), float(o.y), float(o.z)],
                "is_person": bool(o.is_person),
                "is_graspable": bool(o.is_graspable),
                "keypoints": kps,
            })

        with self.lock:
            self.latest_objects = objs

    # ---------- HELPERS ----------

    def get_latest_frame_and_objects(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
            objects = list(self.latest_objects)
        return frame, objects

    def publish_target_pose(self, obj_dict):
        """Publish PoseStamped for selected object."""
        X, Y, Z = obj_dict["position"]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "zed_camera_center"  # for now, camera frame

        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = Z

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"[HMI] Publishing target for id={obj_dict['id']}: "
            f"({X:.3f}, {Y:.3f}, {Z:.3f})"
        )
        self.target_pub.publish(pose)


# ---------- STREAMING & API ----------

def mjpeg_generator():
    """Yield JPEG frames (with boxes + skeleton drawn)."""
    global node_ref
    while True:
        if node_ref is None:
            time.sleep(0.1)
            continue

        frame, objects = node_ref.get_latest_frame_and_objects()

        if frame is not None:
            # Draw boxes & skeletons
            for obj in objects:
                u_min, v_min, u_max, v_max = obj["bbox"]
                label = obj["label"]

                # Bounding box
                cv2.rectangle(
                    frame,
                    (int(u_min), int(v_min)),
                    (int(u_max), int(v_max)),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    f"{label} (id={obj['id']})",
                    (int(u_min), max(0, int(v_min) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

                # If person → draw skeleton
                if obj["is_person"]:
                    kps = obj["keypoints"]

                    def kp_by_name(name):
                        for k in kps:
                            if k["name"] == name:
                                return (int(k["u"]), int(k["v"]))
                        return None

                    # draw joints
                    for k in kps:
                        cv2.circle(
                            frame,
                            (int(k["u"]), int(k["v"])),
                            3,
                            (0, 255, 255),
                            -1,
                        )

                    # simple skeleton connections
                    skeleton_pairs = [
                        ("left_shoulder", "right_shoulder"),
                        ("left_shoulder", "left_elbow"),
                        ("left_elbow", "left_wrist"),
                        ("right_shoulder", "right_elbow"),
                        ("right_elbow", "right_wrist"),
                        ("left_shoulder", "left_hip"),
                        ("right_shoulder", "right_hip"),
                    ]

                    for a, b in skeleton_pairs:
                        pa = kp_by_name(a)
                        pb = kp_by_name(b)
                        if pa and pb:
                            cv2.line(frame, pa, pb, (0, 255, 255), 2)

            ok, jpeg = cv2.imencode(".jpg", frame)
            if not ok:
                time.sleep(0.05)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                jpeg.tobytes() +
                b"\r\n"
            )
        else:
            time.sleep(0.05)


@app.route("/stream")
def stream():
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/detections")
def detections():
    global node_ref
    if node_ref is None:
        return jsonify([])

    _, objects = node_ref.get_latest_frame_and_objects()
    return jsonify(objects)


@app.route("/select_object", methods=["POST"])
def select_object():
    """Receive clicked object ID from frontend and publish Pose."""
    global node_ref
    data = request.get_json()
    obj_id = data.get("id", None)
    if obj_id is None:
        return jsonify({"status": "error", "msg": "no id provided"}), 400

    _, objects = node_ref.get_latest_frame_and_objects()
    selected = None
    for obj in objects:
        if obj["id"] == obj_id:
            selected = obj
            break

    if selected is None:
        return jsonify({"status": "error", "msg": "object not found"}), 404

    node_ref.publish_target_pose(selected)
    return jsonify({"status": "ok"})


@app.route("/")
def index():
    html = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ZED HMI – Advanced Vision</title>
  <style>
    body { background:#111; color:#eee; font-family:sans-serif; text-align:center; }
    #container { position:relative; display:inline-block; }
    #video { display:block; max-width:100%; }
    #overlay { position:absolute; left:0; top:0; pointer-events:none; }
    #status { margin-top:10px; font-size:14px; }
  </style>
</head>
<body>
  <h1>ZED HMI – Click an Object</h1>
  <div id="container">
    <img id="video" src="/stream" alt="ZED Stream">
    <canvas id="overlay"></canvas>
  </div>
  <div id="status">Waiting for detections...</div>

  <script>
    const video = document.getElementById('video');
    const canvas = document.getElementById('overlay');
    const ctx = canvas.getContext('2d');
    const statusDiv = document.getElementById('status');

    let detections = [];

    function resizeCanvas() {
      canvas.width = video.clientWidth;
      canvas.height = video.clientHeight;
    }

    video.onload = function() { resizeCanvas(); };
    window.onresize = resizeCanvas;

    async function fetchDetections() {
      try {
        const res = await fetch('/detections');
        if (!res.ok) return;
        detections = await res.json();
        drawBoxes();
        if (detections.length > 0) {
          statusDiv.textContent = 'Detections: ' + detections.length + ' (click on a box)';
        } else {
          statusDiv.textContent = 'No detections.';
        }
      } catch (e) {
        console.error(e);
        statusDiv.textContent = 'Error fetching detections';
      }
    }

    function drawBoxes() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.lineWidth = 2;
      ctx.font = '14px sans-serif';

      detections.forEach(obj => {
        const [u_min, v_min, u_max, v_max] = obj.bbox;
        ctx.strokeStyle = 'lime';
        ctx.strokeRect(u_min, v_min, u_max - u_min, v_max - v_min);
        ctx.fillStyle = 'lime';
        ctx.fillText(obj.label + ' (id=' + obj.id + ')', u_min + 4, v_min - 4);
      });
    }

    canvas.addEventListener('click', async function(evt) {
      const rect = canvas.getBoundingClientRect();
      const x = evt.clientX - rect.left;
      const y = evt.clientY - rect.top;

      for (let obj of detections) {
        const [u_min, v_min, u_max, v_max] = obj.bbox;
        if (x >= u_min && x <= u_max && y >= v_min && y <= v_max) {
          statusDiv.textContent = 'Selected object id=' + obj.id + ', sending to robot...';
          try {
            const res = await fetch('/select_object', {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({ id: obj.id })
            });
            const data = await res.json();
            if (res.ok) {
              statusDiv.textContent = 'Target pose published for object id=' + obj.id;
            } else {
              statusDiv.textContent = 'Error: ' + data.msg;
            }
          } catch (e) {
            console.error(e);
            statusDiv.textContent = 'Failed to send selection';
          }
          return;
        }
      }

      statusDiv.textContent = 'Clicked, but no object at that location.';
    });

    setInterval(fetchDetections, 200);
  </script>
</body>
</html>
"""
    return html


def ros_spin_thread():
    rclpy.spin(node_ref)


def main(args=None):
    global node_ref
    rclpy.init(args=args)

    node_ref = HMIServerNode()

    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    node_ref.get_logger().info("[HMI] Starting Flask server on port 5000")
    app.run(host="0.0.0.0", port=5000, debug=False)

    node_ref.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
