# Header for timestamp
std_msgs/Header header

geometry_msgs/Vector3 pos #x, y, yaw [m, m, deg]

float64 velocity #[m/s]

uint8 type # Current Vehicle Target State

# True if vehicle at this point would collide with occupancy map
std_msgs/Bool collision

# target states:
uint8 UNINITIALIZED=0
uint8 START=1
uint8 ACTIVE=2
uint8 FINISH=3