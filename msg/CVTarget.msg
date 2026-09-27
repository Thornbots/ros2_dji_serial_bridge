# Computer-vision aim point and fire decision, published by thornbots_pkg's
# point_to_cv_target.py, relayed by mcb_relay and sent to the MCB as CV_MSG
# (id=1) so the gimbal controller can aim and the launcher can fire.
# header.stamp crosses the wire as a millisecond stamp, so the MCB can age
# the point; fire/delay_ms ride the same frame as the aim they were solved
# for, which is why there is no separate fire message.
# x/y/z is a WORLD-FRAME POSITION in odom (REP-105; z up), the frame of
# POSE_MSG's x/y, so the MCB holds it while the chassis moves and turns.
# Not a root- or camera-frame offset and not a barrel attitude -- Type-C
# applies its own ballistics (gravity, drag, muzzle geometry) on top of this
# point. No velocity/spin fields on the wire by design -- see UART_PROTOCOL.md.
std_msgs/Header header   # stamp = decision time, wire stamp_ms; frame_id = odom

float32 x               # aim point position, odom x (m)
float32 y               # aim point position, odom y (m)
float32 z               # aim point position, odom z, up (m)
float32 confidence      # detection confidence [0.0, 1.0]
bool lead_applied        # true if x/y/z includes the intercept lead solve
bool track_valid         # true if backed by a converged target_tracker estimate
bool fire                # true = fire this aim point
uint16 delay_ms          # ms after header.stamp to fire (0 = immediate)
