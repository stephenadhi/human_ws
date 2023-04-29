# Message defining a tracked person

# unique identifier of the target, consistent over time
int16 track_id        

# Tracking state
int8 tracking_state
# 0 -> OFF (object not valid)
# 1 -> OK
# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)

# Current pose of the person
geometry_msgs/PoseStamped  current_pose

# Bounding box dimensions
float32 bbox_x_length
float32 bbox_y_length
float32 bbox_z_length

# interpolated pose history of the track
nav_msgs/Path track