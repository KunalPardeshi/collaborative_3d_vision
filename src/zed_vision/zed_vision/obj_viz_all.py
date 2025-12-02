#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Point

import tf2_ros

# ------------------------------------------------------------
# Robust import for ObjectsStamped across ZED ROS2 versions
# ------------------------------------------------------------
ObjectsStamped = None
_import_errors = []

try:
    from zed_interfaces.msg import ObjectsStamped as _OS
    ObjectsStamped = _OS
except Exception as e:
    _import_errors.append(f"zed_interfaces: {e}")

if ObjectsStamped is None:
    try:
        from stereolabs_interfaces.msg import ObjectsStamped as _OS
        ObjectsStamped = _OS
    except Exception as e:
        _import_errors.append(f"stereolabs_interfaces: {e}")

if ObjectsStamped is None:
    try:
        from zed_msgs.msg import ObjectsStamped as _OS
        ObjectsStamped = _OS
    except Exception as e:
        _import_errors.append(f"zed_msgs: {e}")

if ObjectsStamped is None:
    try:
        from rosidl_runtime_py.utilities import get_message
        for t in [
            "zed_interfaces/msg/ObjectsStamped",
            "stereolabs_interfaces/msg/ObjectsStamped",
            "zed_msgs/msg/ObjectsStamped",
        ]:
            try:
                ObjectsStamped = get_message(t)
                if ObjectsStamped is not None:
                    break
            except Exception as e:
                _import_errors.append(f"{t}: {e}")
    except Exception as e:
        _import_errors.append(f"rosidl_runtime_py get_message: {e}")

if ObjectsStamped is None:
    raise ImportError(
        "Could not import ObjectsStamped from any known ZED interface package.\n"
        f"Errors: {_import_errors}\n"
        "Run: ros2 topic info /zed/zed_node/obj_det/objects -v\n"
        "and check the Type: field."
    )


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def confidence_to_rgb(conf: float) -> Tuple[float, float, float]:
    """ red -> yellow -> green for conf in [0..1]. """
    c = clamp(conf, 0.0, 1.0)
    if c < 0.5:
        r = 1.0
        g = c / 0.5
    else:
        r = 1.0 - (c - 0.5) / 0.5
        g = 1.0
    b = 0.0
    return r, g, b


def make_bbox_corners(center, dims):
    cx, cy, cz = center
    dx, dy, dz = dims
    hx, hy, hz = dx / 2.0, dy / 2.0, dz / 2.0
    return [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]


def bbox_edge_pairs():
    return [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]


def get_obj_id(obj) -> int:
    """
    Robustly get a stable ID from different ZED wrapper versions.
    """
    for name in ["instance_id", "id", "object_id", "tracking_id", "unique_object_id"]:
        if hasattr(obj, name):
            try:
                return int(getattr(obj, name))
            except Exception:
                pass
    # fallback: hash of position (not stable across frames, but prevents crash)
    return int(abs(hash(tuple(obj.position))) % 100000)


def get_obj_label(obj) -> str:
    """
    Robustly get class label.
    """
    for name in ["label", "class_name", "raw_label"]:
        if hasattr(obj, name):
            val = getattr(obj, name)
            # some wrappers store label as int enum -> str conversion optional
            return str(val)
    return "UNKNOWN"


class ObjVizAll(Node):
    def __init__(self):
        super().__init__("obj_viz_all")

        # Parameters
        self.declare_parameter("allowed_classes", [])
        self.declare_parameter("show_only_closest", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("tf_parent_frame", "zed_camera_center")

        self.allowed_classes: List[str] = self.get_parameter("allowed_classes").value
        self.show_only_closest: bool = self.get_parameter("show_only_closest").value
        self.publish_tf: bool = self.get_parameter("publish_tf").value
        self.tf_parent_frame: str = self.get_parameter("tf_parent_frame").value

        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            ObjectsStamped,
            "/zed/zed_node/obj_det/objects",
            self.cb_objects,
            qos_best_effort
        )

        self.pub = self.create_publisher(
            MarkerArray,
            "/obj_markers_all",
            qos_best_effort
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Subscribed: /zed/zed_node/obj_det/objects")
        self.get_logger().info("Publishing: /obj_markers_all")

    def cb_objects(self, msg: ObjectsStamped):
        objects = msg.objects
        if not objects:
            self.pub.publish(MarkerArray(markers=[]))
            return

        # Filter by class if list provided
        if self.allowed_classes:
            filtered = []
            for o in objects:
                if get_obj_label(o) in self.allowed_classes:
                    filtered.append(o)
            objects = filtered

        if not objects:
            self.pub.publish(MarkerArray(markers=[]))
            return

        # Compute distance for each object
        obj_with_dist = []
        for o in objects:
            px = float(o.position[0])
            py = float(o.position[1])
            pz = float(o.position[2])
            dist = math.sqrt(px * px + py * py + pz * pz)
            obj_with_dist.append((o, dist))

        # closest-only mode
        if self.show_only_closest:
            obj_with_dist.sort(key=lambda x: x[1])
            obj_with_dist = obj_with_dist[:1]

        ma = MarkerArray()
        lifetime_ns = int(0.2 * 1e9)
        stamp_now = self.get_clock().now().to_msg()

        for (obj, dist) in obj_with_dist:
            oid = get_obj_id(obj)
            label = get_obj_label(obj)

            x = float(obj.position[0])
            y = float(obj.position[1])
            z = float(obj.position[2])

            dx = float(obj.dimensions_3d[0])
            dy = float(obj.dimensions_3d[1])
            dz = float(obj.dimensions_3d[2])

            conf_raw = float(obj.confidence)
            conf = conf_raw / 100.0 if conf_raw > 1.0 else conf_raw
            r, g, b = confidence_to_rgb(conf)

            ns_base = f"obj_{oid}"
            frame_id = msg.header.frame_id if msg.header.frame_id else self.tf_parent_frame

            # Solid cube
            cube = Marker()
            cube.header = msg.header
            cube.ns = ns_base + "_cube"
            cube.id = oid
            cube.type = Marker.CUBE
            cube.action = Marker.ADD

            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = z
            cube.pose.orientation.w = 1.0

            cube.scale.x = dx
            cube.scale.y = dy
            cube.scale.z = dz

            cube.color.r = r
            cube.color.g = g
            cube.color.b = b
            cube.color.a = 0.35

            cube.lifetime.nanosec = lifetime_ns
            ma.markers.append(cube)

            # Wireframe outline
            corners = make_bbox_corners((x, y, z), (dx, dy, dz))
            edges = bbox_edge_pairs()

            outline = Marker()
            outline.header = msg.header
            outline.ns = ns_base + "_outline"
            outline.id = oid + 100000
            outline.type = Marker.LINE_LIST
            outline.action = Marker.ADD
            outline.scale.x = 0.01

            outline.color.r = r
            outline.color.g = g
            outline.color.b = b
            outline.color.a = 1.0

            for a, c in edges:
                outline.points.append(self._pt(corners[a]))
                outline.points.append(self._pt(corners[c]))

            outline.lifetime.nanosec = lifetime_ns
            ma.markers.append(outline)

            # Text label
            text = Marker()
            text.header = msg.header
            text.ns = ns_base + "_text"
            text.id = oid + 200000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + dz / 2.0 + 0.08
            text.pose.orientation.w = 1.0

            text.scale.z = 0.08
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0

            text.text = f"{label} id:{oid} {conf:.2f}\n{dist:.2f} m"
            text.lifetime.nanosec = lifetime_ns
            ma.markers.append(text)

            # TF
            if self.publish_tf:
                self._publish_obj_tf(frame_id, oid, obj, stamp_now)

        self.pub.publish(ma)

    def _pt(self, xyz):
        p = Point()
        p.x = float(xyz[0])
        p.y = float(xyz[1])
        p.z = float(xyz[2])
        return p

    def _publish_obj_tf(self, parent_frame: str, oid: int, obj, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = f"obj_{oid}"

        t.transform.translation.x = float(obj.position[0])
        t.transform.translation.y = float(obj.position[1])
        t.transform.translation.z = float(obj.position[2])
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ObjVizAll()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
