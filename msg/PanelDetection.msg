# Bounding box + depth for one detected armor panel, bundled together so
# thornbots_pkg can reason about panel geometry instead of a bare point.
# ROS-internal only: produced by roi_depth_query's roi_depth_node (in the
# Realsense_ROI_Depth_Rectifier repo) and by sim's cv_target_emulator.py,
# consumed by thornbots_pkg's target_selector.py. Never crosses the UART wire.
std_msgs/Header header  # stamp = driving detection frame capture time; frame_id = camera frame

geometry_msgs/Point32[4] corners  # TL,TR,BR,BL - camera body frame (REP-103), metres;
                                   # light-bar ends from the pose solve when has_normal
geometry_msgs/Point32 center      # deprojected bbox-center point
float32 depth_m                   # mean sampled depth used for deprojection (planar assumption)
float32 confidence                # max hypothesis score, forwarded from Detection2D
int32 class_id                    # best hypothesis class_id, -1 if none
geometry_msgs/Vector3 normal      # unit plate normal pointing out of the robot, camera body frame
bool has_normal                   # false: normal unmeasured (box-only detector), ignore it
float32 normal_yaw_std            # rad, 1-sigma of the normal's yaw about the camera's up axis
uint32 robot_track_id             # 0 = ungrouped (raw roi_depth_query output);
                                   # set by thornbots_pkg/target_selector.py once grouped
