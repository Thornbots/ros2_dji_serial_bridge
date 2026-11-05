# Tracked-target state published by thornbots_pkg's target_tracker.py.
# Crosses the UART as TARGET_STATE (id=7) via mcb_relay and the bridge's
# ~/target_state; the MCB plans aim and fire from it. variance and
# yaw_rate_variance stay on the Jetson. Frame: odom (see target_tracker.py).
# Armor model: panel k sits at center + radius[k % 2] * (cos(yaw + k*pi/2),
# sin(yaw + k*pi/2), 0) + (0, 0, z_offset[k % 2]). Pair [0] is the tracked
# panel's (k = 0, 2), pair [1] the other (k = 1, 3).
std_msgs/Header header            # stamp = the time this state describes (its publish time); frame_id = odom
builtin_interfaces/Time detection_stamp  # newest contributing detection's capture time, after camera latency correction

uint32 robot_track_id             # changes on robot switch, never plate handoff
float32 confidence                # detector confidence of the tracked panel, 0-1
geometry_msgs/Point center        # chassis center, m
geometry_msgs/Vector3 velocity    # chassis center velocity, m/s
geometry_msgs/Vector3 acceleration  # chassis center acceleration, m/s^2
float32[6] variance               # EKF diagonal (x,y,z,vx,vy,vz)
geometry_msgs/Point panel         # tracked panel's latest measured position, for the !valid fallback
float32 yaw                       # tracked panel's outward-normal yaw, rad (unwrapped)
float32 yaw_rate                  # spin rate, rad/s, + = counter-clockwise from above
float32 yaw_rate_variance         # EKF variance of yaw_rate
float32[2] radius                 # center to panel, m, per pair
float32[2] z_offset               # panel height above center.z, m, per pair
bool valid                        # false until converged: aim at panel, don't lead or fire
bool type_c_based_patrol          # MCB patrol permission; set by mcb_relay, false from the tracker
