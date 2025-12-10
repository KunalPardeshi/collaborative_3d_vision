# One body joint (2D + 3D)

string name           # e.g. "left_wrist"

# 2D position in image (pixels)
float32 u             # column
float32 v             # row

# 3D position in camera frame (meters)
float32 x
float32 y
float32 z

# Confidence of this joint (0.0 - 1.0)
float32 confidence
