import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import for 3D plotting
import csv
import numpy as np 

# Read data from CSV file
time = []
variable_values = []

position_values_x = []
position_values_y = []
position_values_z = []

c_sigma = np.array([8, 8, 8])
e_i = []

# Specify the full path to the CSV file
file_path = '/home/miguel/crazyflie_simulation/webots/controllers/crazyflie_controller_py_test1/drone_8_data_table.csv'

# Open the file
with open(file_path, 'r') as csvfile:
    csvreader = csv.reader(csvfile)

    # Skip the header row
    next(csvreader)

    for row in csvreader:
        time.append(float(row[1]))
        variable_values.append(float(row[2]))
        
        # Split the comma-separated string into values
        position_data_values = row[3].split(',')
        
        # Convert the string values to floats
        position_data = [float(value) for value in position_data_values]

        e = np.linalg.norm(position_data - c_sigma, axis=0)
        e_i.append(e)
        
        position_values_x.append(position_data[0])
        position_values_y.append(position_data[1])
        position_values_z.append(position_data[2])


# Calculate velocity along the X-axis
velocity_x = [0.0]  # Initialize with 0 for the first data point
velocity_y = [0.0]  # Initialize with 0 for the first data point
velocity_z = [0.0]  # Initialize with 0 for the first data point
for i in range(1, len(position_values_x)):
    delta_position_x = position_values_x[i] - position_values_x[i - 1]
    delta_position_y = position_values_y[i] - position_values_y[i - 1]
    delta_position_z = position_values_z[i] - position_values_z[i - 1]
    delta_time = time[i] - time[i - 1]
    velocity_x.append(delta_position_x / delta_time)
    velocity_y.append(delta_position_y / delta_time)
    velocity_z.append(delta_position_z / delta_time)

# Create a line plot for velocity along the X-axis
plt.figure(figsize=(10, 5))
plt.plot(time, velocity_x)
plt.xlabel('Time')
plt.ylabel('Velocity along X-axis')
plt.title('Velocity Along X-axis Over Time')

# Create a line plot for velocity along the X-axis
plt.figure(figsize=(10, 5))
plt.plot(time, e_i)
plt.xlabel('Time')
plt.ylabel('ERROR')
plt.title('Distance from the centroid to c_sigma')



# Create a line plot for variable_values
plt.figure(figsize=(10, 5))
plt.plot(time, variable_values)
plt.xlabel('Time')
plt.ylabel('Average Distance to Centroid')
plt.title('ALMOST CONSTANT ATTRACTION AND UNBOUNDED REPULSION CASE')


# Create a 3D plot for position over time
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Set the limits of the Y and Z axes based on the range of values in the X axis
x_min = min(position_values_x)
x_max = max(position_values_x)
y_min = min(position_values_y)
y_max = max(position_values_y)
z_min = min(position_values_z)
z_max = max(position_values_z)

ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, x_max)
ax.set_zlim(z_min, x_max)

ax.plot(position_values_x, position_values_y, position_values_z)
ax.set_xlabel('Position X')
ax.set_ylabel('Position Y')
ax.set_zlabel('Position Z')
ax.set_title('Position Over Time (3D Plot)')

# Show both plots
plt.show()
