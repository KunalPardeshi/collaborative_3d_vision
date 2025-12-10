#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose

# Try to support multiple ZED interface packages
try:
    from zed_interfaces.msg import ObjectsStamped
except ImportError:
    try:
        from stereolabs_interfaces.msg import ObjectsStamped  # older/newer wrappers
    except ImportError:
        from zed_msgs.msg import ObjectsStamped  # fallback


class ObjectCoordsPublisher(Node):
    """
    Subscribe to ZED object detections and publish their 3D positions as a PoseArray.

    Input:
        /zed/zed_node/obj_det/objects   (ObjectsStamped)

    Output:
        /object_poses                   (PoseArray, in camera frame)
    """

    def __init__(self):
        super().__init__('object_coords_publisher')

        # Subscriber: ZED object detections
        self.obj_sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.objects_callback,
            10
        )

        # Publisher: PoseArray of object positions
        self.poses_pub = self.create_publisher(
            PoseArray,
            '/object_poses',
            10
        )

        self.logged_attr_warning = False  # to avoid spamming attributes every frame

        self.get_logger().info('ObjectCoordsPublisher node started.')
        self.get_logger().info('Subscribing to /zed/zed_node/obj_det/objects')
        self.get_logger().info('Publishing poses on /object_poses')

    # ---- helper to extract position regardless of field name ----
    def extract_position(self, obj):
        """
        Try multiple common field names to get a 3D position for the object.

        Returns:
            (x, y, z) in meters, or None if no usable field found.
        """
        candidate_fields = [
            'position',       # most common (geometry_msgs/Point)
            'position_3d',    # some ZED wrappers
            'position_3d_rel',
            'translation',    # generic name in some interfaces
        ]

        for field in candidate_fields:
            if hasattr(obj, field):
                p = getattr(obj, field)
                # Expect something with x, y, z
                if hasattr(p, 'x') and hasattr(p, 'y') and hasattr(p, 'z'):
                    return float(p.x), float(p.y), float(p.z)

        # First time we fail, print available attributes for debugging
        if not self.logged_attr_warning:
            attrs = [a for a in dir(obj) if not a.startswith('_')]
            self.get_logger().warn(
                'Object has no recognized position field. '
                f'Available attributes: {attrs}'
            )
            self.logged_attr_warning = True

        return None

    # ---- main callback ----
    def objects_callback(self, msg: ObjectsStamped):
        objects = msg.objects

        pose_array = PoseArray()
        pose_array.header = msg.header  # keep same frame + timestamp

        if not objects:
            # Publish an empty PoseArray so downstream knows this frame had 0 objects
            self.poses_pub.publish(pose_array)
            return

        for obj in objects:
            pos = self.extract_position(obj)
            if pos is None:
                # Skip this object, cannot get position
                continue

            x, y, z = pos

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z

            # Identity orientation (no rotation info from OD)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

            # Extract some identity info for logging
            obj_id = getattr(obj, 'instance_id', None)
            if obj_id is None:
                obj_id = getattr(obj, 'id', None)
            if obj_id is None:
                obj_id = getattr(obj, 'object_id', None)
            if obj_id is None:
                obj_id = getattr(obj, 'tracking_id', None)

            label = getattr(obj, 'label', 'unknown')

            # Distance from camera (Euclidean)
            distance = (x ** 2 + y ** 2 + z ** 2) ** 0.5

            self.get_logger().info(
                f'Object: id={obj_id}, label={label}, '
                f'pos=({x:.3f}, {y:.3f}, {z:.3f}) m, '
                f'distance={distance:.3f} m'
            )

        # Publish array (may be empty if all objects lacked a usable position field)
        self.poses_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectCoordsPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ignore double-shutdown issues
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
