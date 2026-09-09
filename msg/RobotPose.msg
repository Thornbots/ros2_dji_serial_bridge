# Pose and velocity of the robot chassis plus gimbal angles.
# Sent by the MCB at 100 Hz (POSE_MSG, id=2). header.stamp is the Jetson's
# arrival time, not an MCB clock; no MCB timestamp crosses the wire.
std_msgs/Header header

float32 x          # chassis X position  (odometry, metres)
float32 y          # chassis Y position  (odometry, metres)
float32 vel_x      # chassis X velocity  (m/s)
float32 vel_y      # chassis Y velocity  (m/s)
float32 head_pitch # gimbal pitch angle  (radians, encoder value)
float32 head_yaw   # gimbal yaw angle relative to world (radians)

# Odometry health, reported with every pose so the verdict always matches the
# x/y it arrived with. Non-zero means x/y/vel_x/vel_y are not trustworthy;
# head_pitch/head_yaw come from the gimbal encoders and stay good regardless.
# See UART_PROTOCOL.md for the wire layout.
uint8 ODOM_OK = 0
uint8 ODOM_ENCODER_FAULT = 1   # wheel encoder fault or implausible reading
uint8 ODOM_IMU_FAULT = 2       # IMU fault, saturation, or failed calibration
uint8 ODOM_SLIP = 3            # wheels turning without matching motion
uint8 ODOM_UNKNOWN = 4         # MCB knows odom is bad, not why

uint8 odom_status  # one of the ODOM_* constants above
