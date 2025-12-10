# collaborative_3d_vision

> 3D vision stack for the **COLLABORATIVE MANIPULATION** project – running on **Jetson Orin + ROS 2 Humble** with a **Stereolabs ZED X** stereo camera.

This repository focuses on the **3D Vision Department** only:

- Streaming ZED X into RViz2
- ZED SDK multi-class object detection (live + SVO)
- Custom 3D visualization in RViz2 (cubes, wireframes, labels, distances)
- Clean ROS 2 Humble workspace structure under `~/zed_ws`

Later, this will plug into a larger pipeline:
`ZED → TF → Planner → Igus Rebel Arm`.

---

## Repository Structure

```text
~/zed_ws
├── README.md                 # This file
├── docs/
│   ├── INSTALL_JETSON_ZED_ROS2.md   # Full install guide for a fresh Jetson
│   └── 3D_VISION_PIPELINE.md        # Detailed vision + OD + RViz docs
└── src/
    ├── zed-ros2-wrapper/     # Official Stereolabs ROS 2 wrapper (cloned separately)
    └── zed_vision/           # Custom vision package
        └── zed_vision/
            └── obj_viz_all.py
