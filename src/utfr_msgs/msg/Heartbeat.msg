# Header for timestamp
std_msgs/Header header

# name of module
std_msgs/String module 

# configured update rate of module
float64 update_rate # ms

# heartbeat status based on enum below
uint8 status 

# lap counter
uint8 lap_count # TODO: Implement logic in car interface node

# module status
uint8 NOT_READY=1
uint8 READY=2
uint8 ACTIVE=3
uint8 ERROR=4
uint8 FINISH=5