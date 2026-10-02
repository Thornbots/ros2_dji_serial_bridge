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
uint8 FLAG_LEAD_APPLIED = 1   # x/y/z includes the intercept lead solve
uint8 FLAG_TRACK_VALID = 2    # backed by a converged target_tracker estimate
uint8 FLAG_FIRE = 4           # fire this aim point delay_ms after header.stamp
uint8 FLAG_TARGET = 8         # x/y/z is an aim point; clear = no target, the MCB patrols
uint8 flags                   # FLAG_* bits; crosses the wire as CV_MSG's flags byte
uint16 delay_ms          # ms after header.stamp to fire (0 = immediate)
