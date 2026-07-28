# Computer-vision target detection result published by the Jetson CV pipeline.
# Forwarded to the Jetson as CV_MSG (id=1) so the gimbal controller can aim.
# All coordinates are in the camera frame (metres).
std_msgs/Header header

float32 x           # target position  – right  (m)
float32 y           # target position  – up     (m)
float32 z           # target position  – forward (m)
float32 confidence  # detection confidence [0.0, 1.0]
