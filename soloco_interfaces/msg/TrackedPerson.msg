# Message defining a tracked person
std_msgs/Header header
# unique identifier of the target, consistent over time
int16 track_id        

# Tracking state
int8 tracking_state
# 0 -> OFF (object not valid)
# 1 -> OK
# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)

# The following fields are extracted from the Kalman state x and its covariance C
geometry_msgs/PoseWithCovariance  current_pose      # current pose of the track 

# Bounding box dimensions
float32 bbox_x_length
float32 bbox_y_length
float32 bbox_z_length

# interpolated pose history of the track
nav_msgs/Path track     