# Pose and velocity of the robot chassis plus gimbal angles.
# Sent by the MCB at 90 Hz (POSE, id=2). header.stamp is when the MCB
# started sending the frame: its last byte's read time less the frame's wire
# time at the baud. No MCB clock crosses the wire, and USB latency isn't
# taken off.
std_msgs/Header header

float32 x          # chassis X position  (odometry, metres)
float32 y          # chassis Y position  (odometry, metres)
float32 vel_x      # chassis X velocity  (m/s)
float32 vel_y      # chassis Y velocity  (m/s)
float32 head_pitch # gimbal pitch angle  (radians, encoder value)
float32 head_yaw   # gimbal yaw relative to world (radians), CCW-positive from above

# Chassis heading in the world, same sense and zero as head_yaw (thornbots_pkg's
# URDF turns both joints about +z). Not on the wire yet: 0 until the firmware
# sends them (UART_PROTOCOL.md "Proposed: POSE chassis yaw").
float32 chassis_yaw       # radians
float32 chassis_yaw_rate  # rad/s

# Which source produced x/y/vel_x/vel_y, reported with every pose so the
# answer always matches the values it arrived with. head_pitch/head_yaw are
# gimbal encoder values and are unaffected by any of these.
# See UART_PROTOCOL.md for the wire layout.
uint8 ODOM_PODS = 0             # odometry pods healthy, best accuracy
uint8 ODOM_DRIVETRAIN = 1       # drivetrain odometry, degraded by wheel slip
uint8 ODOM_I2C_DEAD = 2         # I2C bus dead, no pod data, no fallback
uint8 ODOM_I2C_DEAD_DRIVETRAIN = 3  # I2C bus dead, drivetrain odometry instead

uint8 odom_status  # one of the ODOM_* constants above
