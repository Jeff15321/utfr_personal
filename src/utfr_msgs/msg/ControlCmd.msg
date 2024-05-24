# Input to actuators

# Header for timestamp
std_msgs/Header header

# TODO: find RPM limits
int32 thr_cmd # [min_rpm, max_rpm] RPM
int32 brk_cmd #[0, -30] Nm
int32 str_cmd #[-3200, 3200] Degrees

