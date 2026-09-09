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

# Which source produced x/y/vel_x/vel_y, reported with every pose so the
# answer always matches the values it arrived with. head_pitch/head_yaw are
# gimbal encoder values and are unaffected by any of these.
# See UART_PROTOCOL.md for the wire layout.
uint8 ODOM_PODS = 0             # odometry pods healthy, best accuracy
uint8 ODOM_DRIVETRAIN = 1       # drivetrain odometry, degraded by wheel slip
uint8 ODOM_I2C_DEAD = 2         # I2C bus dead, no pod data, no fallback
uint8 ODOM_I2C_DEAD_DRIVETRAIN = 3  # I2C bus dead, drivetrain odometry instead

uint8 odom_status  # one of the ODOM_* constants above
