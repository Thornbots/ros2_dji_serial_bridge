# PING (id=5), either direction. Number semantics are in UART_PROTOCOL.md.
# From the MCB, header.stamp is when the MCB started sending the frame, as in
# RobotPose. To the MCB, the header is not sent.
std_msgs/Header header

uint8 number
