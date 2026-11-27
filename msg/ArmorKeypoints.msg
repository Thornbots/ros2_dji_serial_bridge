# One armor plate from a YOLO26 detector, in network-input pixels
# (letterboxed, before mapping to colour space). ROS-internal only.
# Keypoints are the outer light-bar ends: top-left, top-right,
# bottom-right, bottom-left as seen in the image. See
# Realsense_ROI_Depth_Rectifier's README for the pose solve.
std_msgs/Header header  # stamp = capture time of the source image; frame_id = colour camera frame

float32 center_x          # bbox centre, network pixels
float32 center_y
float32 size_x            # bbox width/height, network pixels
float32 size_y
float32 confidence
int32 class_id
bool has_keypoints        # false for detect-only models
geometry_msgs/Point32[4] keypoints  # x, y network pixels; z = keypoint confidence 0-1
