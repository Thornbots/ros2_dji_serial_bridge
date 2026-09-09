# Computer-vision aim point published by thornbots_pkg's point_to_cv_target.py,
# relayed by mcb_relay and sent to the MCB as CV_MSG (id=1) so the gimbal
# controller can aim. header does not cross the wire, so the MCB cannot age
# this point; publish at a rate the gimbal can trust.
# x/y/z is a ROOT-FRAME POSITION (REP-103: x forward, y left, z up), not a
# camera-frame offset and not a barrel attitude -- Type-C applies its own
# ballistics (gravity, drag, muzzle geometry) on top of this point. No
# velocity/spin fields on the wire by design -- see UART_PROTOCOL.md.
std_msgs/Header header

float32 x               # aim point position -- forward (m), root frame
float32 y               # aim point position -- left    (m), root frame
float32 z               # aim point position -- up      (m), root frame
float32 confidence      # detection confidence [0.0, 1.0]
bool lead_applied        # true if x/y/z includes the intercept lead solve
bool track_valid         # true if backed by a converged target_tracker estimate
