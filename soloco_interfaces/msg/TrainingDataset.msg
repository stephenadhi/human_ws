# Message defining the recorded data from pedsim to create a training dataset

# header to track timestamp and frame id
std_msgs/Header header

# Scene identifier
int8 scene_id

# ID of the ego agent
int8 ego_agent_id

# Current states of the ego agent (pose and twist)
nav_msgs/Odometry

# This is the local costmap. Only based on walls and/or static obstacles.
nav_msgs/OccupancyGrid local_occgrid

# This is the global costmap. Only based on walls and/or static obstacles.
nav_msgs/OccupancyGrid global_occgrid

# Only record current people position (x, y). Ego agent is excluded.
soloco_interfaces/TrackedPersons people
