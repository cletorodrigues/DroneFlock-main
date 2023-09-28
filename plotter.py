import matplotlib.pyplot as plt
import csv

import ast  # For literal_eval

# Read data from CSV file
time = []
variable_values = []

position_values_x = []
position_values_y = []
position_values_z = []


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



# Create a line plot for variable_values
plt.plot(time, variable_values)
plt.xlabel('Time')
plt.ylabel('Average Distance to Centroid')
plt.title('ALMOST CONSTANT ATTRACTION AND UNBOUNDED REPULSION CASE')

# Show both plots
plt.show()