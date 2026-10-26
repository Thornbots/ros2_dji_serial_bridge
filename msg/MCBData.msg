# Opaque firmware-owned channel data. stamp estimates when the MCB began
# sending this frame (last-byte read time minus UART wire time), like RobotPose.
std_msgs/Header header
uint8 channel
uint8[] data
