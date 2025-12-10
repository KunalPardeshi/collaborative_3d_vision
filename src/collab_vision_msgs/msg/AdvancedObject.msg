# One detected object (person or other)

# Basic info
int32 id              # tracking ID
string label          # class label, e.g. "person", "bottle"
float32 confidence    # detection confidence (0.0 - 1.0)

# 2D bounding box in image space (pixels)
float32 u_min         # left
float32 v_min         # top
float32 u_max         # right
float32 v_max         # bottom

# 3D centroid in camera frame (meters)
float32 x
float32 y
float32 z

# Flags
bool is_person        # true if human
bool is_graspable     # true if robot should be allowed to pick it

# Human pose (used only if is_person == true)
HumanKeypoint[] keypoints
