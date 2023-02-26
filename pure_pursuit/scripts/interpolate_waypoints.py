import csv
import numpy as np
from scipy.interpolate import splprep, splev

# Read in the data from the CSV file
with open('waypoints.csv', 'r') as f:
    reader = csv.reader(f)
    next(reader)  # Skip the header row
    data = np.array(list(reader), dtype=np.float32)

# Separate the x, y, and z columns into separate arrays
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

# Set the number of points for the interpolated spline
n_points = 200

# Generate the interpolated spline
tck, u = splprep([x, y, z], s=0)
u_new = np.linspace(u.min(), u.max(), n_points)
x_new, y_new, z_new = splev(u_new, tck)

# Write the interpolated data to a new CSV file
with open('/home/griffin/Documents/f1tenth_ws/src/lab-5-slam-and-pure-pursuit-team-10/pure_pursuit/src/interpolated_path.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y', 'z'])
    for i in range(n_points):
        writer.writerow([x_new[i], y_new[i], z_new[i]])
