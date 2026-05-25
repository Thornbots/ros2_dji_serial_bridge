# Pose and velocity of the robot chassis plus gimbal angles.
# Sent by the Type-C at 100 Hz (POSE_MSG, id=2).
std_msgs/Header header

float32 x          # chassis X position  (odometry, metres)
float32 y          # chassis Y position  (odometry, metres)
float32 vel_x      # chassis X velocity  (m/s)
float32 vel_y      # chassis Y velocity  (m/s)
float32 head_pitch # gimbal pitch angle  (radians, encoder value)
float32 head_yaw   # gimbal yaw angle relative to world (radians)
