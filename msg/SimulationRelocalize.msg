# Simulator-only relocalization envelope; the embedded correction stamp is
# preserved for its future application tick, while header.stamp identifies the
# logical EKF decision tick carried by the UART simulator packet.
std_msgs/Header header
bool has_correction
geometry_msgs/PointStamped correction
