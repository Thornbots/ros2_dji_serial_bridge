# Computer-vision aim point and fire decision, published by thornbots_pkg's
# point_to_cv_target.py, relayed by mcb_relay and sent to the MCB as CV_TARGET
# (id=1). Fields after header are in wire order; see UART_PROTOCOL.md.
# header.stamp crosses the wire as a millisecond stamp, so the MCB can age
# the point. fire and delay_ms ride the same frame as the aim they were
# solved for, which is why there is no separate fire message.
# x/y/z is a WORLD-FRAME POSITION in odom (REP-105; z up), the frame of
# POSE's x/y. Not a root- or camera-frame offset and not a barrel
# attitude: the MCB applies its own ballistics. Every message is an aim
# point; with no target it's a patrol point with fire clear, or none.

std_msgs/Header header       # stamp = decision time, wire stamp_ms; frame_id = odom
float32 x                    # aim point, odom x (m)
float32 y                    # aim point, odom y (m)
float32 z                    # aim point, odom z, up (m)
uint16 delay_ms              # ms after header.stamp to fire (0 = immediate)

# Booleans packed into the single flags byte.
# Bit layout on the wire (MSB→LSB): reserved x5|turn_to_hit|
# type_c_based_patrol|fire -- table in UART_PROTOCOL.md.
bool fire                    # fire this aim point delay_ms after header.stamp
bool type_c_based_patrol     # true lets the MCB patrol on its own, false stops it
bool turn_to_hit             # true lets the MCB turn toward where it got hit
