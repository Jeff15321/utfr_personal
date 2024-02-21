# Input to actuators

# Header for timestamp
std_msgs/Header header

# TODO: find RPM limits
int32 thr_cmd # [min_rpm, max_rpm] RPM
float32 brk_cmd #[0, 255] PWM
int32 str_cmd #[-360, 360] Degrees

