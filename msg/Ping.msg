# PING (id=5), either direction. The MCB echoes every Ping it gets straight
# back, so a number sent on ~/ping_to_mcb returns on ~/ping_from_mcb.
# From the MCB, header.stamp is when the MCB started sending the frame, as in
# RobotPose. To the MCB, the header is not sent.
std_msgs/Header header

uint8 number
