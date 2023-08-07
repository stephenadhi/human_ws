# Message defining the future of a tracked person

# unique identifier of the target
int16 agent_id

# interpolated future trajectory of the track
nav_msgs/Path track

# For evaluator node
float32 radius