# Computer-vision aim point and fire decision, published by thornbots_pkg's
# point_to_cv_target.py, relayed by mcb_relay and sent to the MCB as CV_MSG
# (id=1) so the gimbal controller can aim and the launcher can fire.
# header.stamp crosses the wire as a millisecond stamp, so the MCB can age
# the point; fire/delay_ms ride the same frame as the aim they were solved
# for, which is why there is no separate fire message.
# x/y/z is a ROOT-FRAME POSITION (REP-103: x forward, y left, z up), not a
# camera-frame offset and not a barrel attitude -- Type-C applies its own
# ballistics (gravity, drag, muzzle geometry) on top of this point. No
# velocity/spin fields on the wire by design -- see UART_PROTOCOL.md.
std_msgs/Header header   # header.stamp = decision time, wire stamp_ms

float32 x               # aim point position -- forward (m), root frame
float32 y               # aim point position -- left    (m), root frame
float32 z               # aim point position -- up      (m), root frame
float32 confidence      # detection confidence [0.0, 1.0]
bool lead_applied        # true if x/y/z includes the intercept lead solve
bool track_valid         # true if backed by a converged target_tracker estimate
bool fire                # true = fire this aim point
uint16 delay_ms          # ms after header.stamp to fire (0 = immediate)
