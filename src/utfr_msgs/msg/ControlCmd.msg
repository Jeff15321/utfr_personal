# Input to actuators

# Header for timestamp
std_msgs/Header header

float64 thr_cmd #[0.0, 1.0] Accel in m/s2
float64 brk_cmd #[-1.0, 0.0] 
float64 str_cmd #[-1.0, 1.0] Rate in rad/s 

