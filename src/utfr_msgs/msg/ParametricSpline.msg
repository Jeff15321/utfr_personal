# Header for timestamp
std_msgs/Header header

# Path params
# List of length 6. Index 0 corresponds to x^5, index 1 to x^4, etc.
float64[] x_params 
float64[] y_params

float64[] skidpad_params # [x0, y0, radius]

uint8 lap_count