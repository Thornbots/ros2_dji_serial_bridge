# Computer-vision target detection result published by the Jetson CV pipeline.
# Forwarded to the Jetson as CV_MSG (id=1) so the gimbal controller can aim.
# All coordinates are in the camera frame (metres, m/s, m/s²).
std_msgs/Header header

float32 x           # target position  – right  (m)
float32 y           # target position  – up     (m)
float32 z           # target position  – forward (m)
float32 v_x         # target velocity  – right  (m/s)
float32 v_y         # target velocity  – up     (m/s)
float32 v_z         # target velocity  – forward (m/s)
float32 a_x         # target accel     – right  (m/s²)
float32 a_y         # target accel     – up     (m/s²)
float32 a_z         # target accel     – forward (m/s²)
float32 confidence  # detection confidence [0.0, 1.0]
