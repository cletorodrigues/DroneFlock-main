import matplotlib.pyplot as plt
import csv
import ast  # For literal_eval
from itertools import cycle
import numpy as np 

# Create a list of different colors for plotting
colors = cycle(['b', 'g', 'r', 'c', 'm', 'y', 'k'])

# Initialize lists to store data for all files
all_position_values_x = []
all_position_values_y = []
all_position_values_z = []

c_sigma = np.array([8, 8, 8])
e_i = []

# Specify the range of file IDs (0 to 20 for 21 files)
file_ids = range(21)
print(file_ids)

# Loop through each file ID
for file_id in file_ids:
    # Read data from the CSV file for the current file
    position_values_x = []
    position_values_y = []
    position_values_z = []

    # Specify the file path for the current file
    file_path = f'/home/miguel/crazyflie_simulation/webots/controllers/crazyflie_controller_py_test1/drone_{file_id}_data_table.csv'

    '/home/miguel/crazyflie_simulation/webots/controllers/crazyflie_controller_py_test1/drone_8_data_table.csv'

    # Open the file
    with open(file_path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)

        # Skip the header row
        next(csvreader)

        for row in csvreader:
            # Parse the position string '[x y z]' into separate coordinates
            position_str = row[4]  # Assuming position is in the 4th column
            position_parts = position_str.strip('[]').split()
            position_values_x.append(float(position_parts[0]))
            position_values_y.append(float(position_parts[1]))
            position_values_z.append(float(position_parts[2]))

    # Append the position data for the current file to the cumulative lists
    all_position_values_x.append(position_values_x)
    all_position_values_y.append(position_values_y)
    all_position_values_z.append(position_values_z)

# Create a 3D line plot for all position values with different colors
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for x, y, z, color in zip(all_position_values_x, all_position_values_y, all_position_values_z, colors):
    ax.plot(x, y, z, color=color)

ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('3D Trajectory Plot FIRST FUNCTION')

# Show the plot
plt.show()
