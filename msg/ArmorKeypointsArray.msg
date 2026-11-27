# Every armor plate decoded from one network output, published once per
# frame including empty frames. ROS-internal only.
std_msgs/Header header  # stamp = capture time of the source image (same as each detection)

ArmorKeypoints[] detections
