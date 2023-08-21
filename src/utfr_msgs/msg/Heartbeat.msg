# Header for timestamp
std_msgs/Header header

# name of module
std_msgs/String module 

# configured update rate of module in milliseconds
float64 update_rate

# heartbeat status based on enum below
uint8 status 

# module status:
uint8 UNINITIALIZED=0
uint8 ACTIVE=1
uint8 FATAL=2