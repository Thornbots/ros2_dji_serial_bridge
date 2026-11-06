# header.stamp is ROS time when the bridge emitted this result, not capture time.
std_msgs/Header header
# Metadata identity; zero means failure before metadata was identified.
uint64 frame_id
# True only after the corresponding ROS publish call returned.
bool published
# Empty on success; failure permanently latches the coordinated session.
string reason
