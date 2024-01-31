#Bounding  boxes for perception 2d detection

# Header for timestamp
std_msgs/Header header

int64 x #(px)
int64 y #(px)
int64 width #(px)
int64 height #(px)
int64 type ##cone types
float64 score #confidence
# cone types:
uint8 UNKNOWN=0
uint8 BLUE=1
uint8 YELLOW=2
uint8 SMALL_ORANGE=3
uint8 LARGE_ORANGE=4