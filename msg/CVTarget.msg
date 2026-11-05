# Idle look point, published by thornbots_pkg's point_to_cv_target only while
# no target state is fresh, relayed by mcb_relay and sent to the MCB as
# CV_TARGET (id=1). Aiming and firing at a tracked target ride TargetState.
# Fields after header are in wire order; see UART_PROTOCOL.md.
# header.stamp stays on the Jetson: no stamp crosses the wire. The bridge
# drops the message when it is 200 ms old or older.
# x/y/z is a WORLD-FRAME POSITION in raw, never-relocalized odom, the frame of
# POSE's x/y. Not a root- or camera-frame offset and not a barrel
# attitude: the MCB applies its own ballistics.

std_msgs/Header header       # stamp = decision time; frame_id = odom
float32 x                    # look point, odom x (m)
float32 y                    # look point, odom y (m)
float32 z                    # look point, odom z, up (m)

# Booleans packed into the single flags byte (bit layout: UART_PROTOCOL.md).
bool type_c_based_patrol     # true lets the MCB patrol on its own, false stops it
bool turn_to_hit             # true permits a firmware hit turn
