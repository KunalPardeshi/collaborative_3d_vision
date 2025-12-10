# OBJECT_COORDINATES

ZED X Object Coordinates Publisher + PoseArray (ROS 2 Humble on Jetson Orin)

This document explains how we convert **ZED SDK object detections** into
a list of 3D positions (X, Y, Z) published as a ROS 2 `PoseArray`, ready
to be consumed by other modules (planning, TF, manipulation, logging).

Pipeline:

```text
ZED X / ZED2 / SVO
   ↓
ZED SDK 5.x (Multi-Class Object Detection)
   ↓
zed-ros2-wrapper (ROS 2 Humble, composable node)
   ↓
/zed/zed_node/obj_det/objects   [ObjectsStamped]
   ↓
object_coords node (zed_vision package)
   ↓
/object_poses                   [geometry_msgs/PoseArray]
