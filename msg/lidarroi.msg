## velodyne_cluster

# pointcloud2_to_xyz
float32 min_x
float32 max_x
float32 min_y
float32 max_y
float32 min_z
float32 max_z

# dbscan
float32 eps
int32 min_samples

## lattice_planner

float32 checkObject_dis         #checkObject
float32[] lane_weight           #collision_check
float32 collision_check_dis     #collision_check
float32[] lane_off_set          #latticePlanner
float32 x_interval              #latticePlanner
float32 a2                      #latticePlanner
float32 a3                      #latticePlanner