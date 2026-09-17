# Tracked-target state published by thornbots_pkg's target_tracker.py.
# ROS-internal only -- never crosses the UART wire (see CVTarget.msg for
# the lean wire-format counterpart). Frame: odom (see target_tracker.py).
# Armor model: panel k sits at centre + radius_k * (cos(yaw + k*pi/2),
# sin(yaw + k*pi/2), 0), with radius_k = radius for even k, other_radius odd.
std_msgs/Header header            # stamp = detection stamp; frame_id = odom

uint32 robot_track_id             # changes on robot switch, never plate handoff
geometry_msgs/Point centre        # chassis centre, m
geometry_msgs/Vector3 velocity    # chassis centre velocity, m/s
float32[6] variance               # EKF diagonal (x,y,z,vx,vy,vz)
geometry_msgs/Point panel         # latest selected panel, for the raw fallback
float32 yaw                       # tracked panel's outward-normal yaw, rad (unwrapped)
float32 yaw_rate                  # spin rate, rad/s, + = counter-clockwise from above
float32 yaw_rate_variance         # EKF variance of yaw_rate
float32 radius                    # centre to tracked panel, m
float32 other_radius              # centre to the other panel pair, m
bool valid                        # false until converged; don't lead on it
