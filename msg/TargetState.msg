# Tracked-target state published by sentry_pkg's target_tracker.py.
# ROS-internal only -- never crosses the UART wire (see CVTarget.msg for
# the lean wire-format counterpart). Frame: odom (see target_tracker.py).
std_msgs/Header header            # stamp = detection stamp; frame_id = odom

uint32 robot_track_id             # changes on robot switch, never plate handoff
geometry_msgs/Point centre        # spin-centre position, m
geometry_msgs/Vector3 velocity    # spin-centre velocity, m/s
float32[6] variance               # KF diagonal (x,y,z,vx,vy,vz)
geometry_msgs/Point panel         # latest selected panel, for the raw fallback
float32 spin_hz                   # 0 if unknown
float32 spin_phase                # radians, 0 if unknown
uint8 estimator                   # 0 = running_mean, 1 = width_refined
bool valid                        # false until converged; don't lead on it
