# Individual definition for Cone attributes

# Header for timestamp
std_msgs/Header header

geometry_msgs/Point pos # x, y, null [m, m, n/a]
uint8 type # cone type based on enum below
float64[2] covariance # 2D covariences for x, y

# cone types:
uint8 UNKNOWN=0
uint8 BLUE=1
uint8 YELLOW=2
uint8 SMALL_ORANGE=3
uint8 LARGE_ORANGE=4
