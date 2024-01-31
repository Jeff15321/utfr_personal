import matplotlib.pyplot as plt
import re

file_path = 'gps.txt'

with open(file_path, 'r') as file:
    gps_data = file.readlines()


# Extracting x and y coordinates from each line
x_coords = []
y_coords = []
for line in gps_data:
    match = re.search(r'x:\s*([-\d.e]+)\s*y:\s*([-\d.e]+)', line)
    if match:
        x, y = match.groups()
        x_coords.append(float(x))
        y_coords.append(float(y))

# Remove the first 1000 points
index = 1
x_coords_before = x_coords[0:index]
y_coords_before = y_coords[0:index]
x_coords_after = x_coords[index:]
y_coords_after = y_coords[index:]

# Plotting the points
plt.scatter(x_coords_before, y_coords_before, c='red', marker='o')
plt.scatter(x_coords_after, y_coords_after, c='green', marker='o')
plt.title('IMU Coordinates Plot')
plt.xlabel('X Coordinates')
plt.ylabel('Y Coordinates')
plt.grid(True)
plt.show()