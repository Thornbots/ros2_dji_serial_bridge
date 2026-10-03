# One raw byte over the BYTE frame (id=5), either direction; its meaning is
# agreed between sender and receiver. From the MCB (~/byte_from_mcb),
# header.stamp is when the MCB started sending the frame, as in RobotPose.
# To the MCB (~/byte_to_mcb), the header is not sent.
std_msgs/Header header

uint8 data
