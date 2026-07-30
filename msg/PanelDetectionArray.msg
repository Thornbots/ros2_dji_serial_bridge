# Every panel detection deprojected from one /detections_output frame.
# ROS-internal only, produced by roi_depth_query's roi_depth_node.
# header.stamp/frame_id match the driving Detection2DArray message (NOT
# the depth image) -- see roi_depth_node.cpp for why. Published once per
# driving frame, including empty (zero detections), so downstream rate
# and target-loss timeout logic stay correct.
std_msgs/Header header

PanelDetection[] detections
