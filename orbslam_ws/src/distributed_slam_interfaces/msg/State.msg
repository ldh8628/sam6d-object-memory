# System UTC nanoseconds; capture stamp is the camera acquisition time.
std_msgs/Header header
uint64 sequence
string<=64 map_id
string<=64 session_id
uint64 tracking_epoch
bool tracking
geometry_msgs/Pose pose
int64 enqueue_ns
int64 track_start_ns
int64 track_end_ns
int64 publish_ns
uint64 dropped_inputs
