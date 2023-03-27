# Message defining a tracked person

# unique identifier of the target, consistent over time
int16                          track_id        

# Tracking state
int8 tracking_state
# 0 -> OFF (object not valid)
# 1 -> OK
# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)

# The following fields are extracted from the Kalman state x and its covariance C
geometry_msgs/PoseWithCovariance  pose      # current pose of the track 
geometry_msgs/TwistWithCovariance twist     # current velocity of the track 

nav_msgs/Path track     # interpolated pose history of the track