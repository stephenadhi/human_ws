# Message defining the future of a tracked person

# unique identifier of the target, consistent over time
int16 track_id

# interpolated future trajectory of the track
nav_msgs/Path track

# For evaluator node
float32 radius