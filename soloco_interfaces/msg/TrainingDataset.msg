# Message defining the recorded data from pedsim to create a training dataset
# Everything will be recorded in the 'map' frame

# header to track timestamp and frame id
std_msgs/Header header

# person_radius to define occupied space [in meters]
int8 person_radius

# Occupancy grid raw. This is the occupancy grid map based on the raw laserscan data.
nav_msgs/OccupancyGrid occgrid_raw

# Occupancy grid filtered. This is the occupancy grid map based on the filtered laserscan data.
# Points where it is located within the person radius will be removed
nav_msgs/OccupancyGrid occgrid_filtered

# people with their interpolated history
soloco_interfaces/TrackedPersons tracks