# header.stamp is simulation capture time of the state encoded by this UART frame.
std_msgs/Header header
# Strictly increasing, nonzero identity for this bridge session.
uint64 frame_id
# Complete original UART frame, including header and CRCs.
uint8[] frame
