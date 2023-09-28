import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
from statistics import mean
from scipy.spatial.distance import pdist, squareform


def calculate_control_forces(pos, a, b, c, drone_radius):
    n_drones = len(pos)
    dist_matrix = squareform(pdist(pos))
    u = np.zeros([n_drones, 3])

    for i in range(n_drones):
        u[i] = 0
        for j in range(n_drones):
            if i != j:
                y_d = pos[i] - pos[j]
                y_norm = dist_matrix[i, j]
                u[i] -= y_d * (a - b * np.exp(-(((y_norm**2) - 4*drone_radius**2)**2) / c))

        u[i] = u[i] + 5 

    return u


def update_positions(pos, u, elapsed_time):
    return pos + u * elapsed_time

def plot_positions(pos, avg_pos, xlim=None, ylim=None, zlim=None):
    x, y, z = pos.T

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    if xlim is not None:
        ax.set_xlim(xlim)
    if ylim is not None:
        ax.set_ylim(ylim)
    if zlim is not None:
        ax.set_zlim(zlim)

    plt.show()


def create_sphere(center, radius, resolution=100):
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    
    return x, y, z


def plot_figures(all_positions, avg_positions, sample_time, Title):

   # Assuming all_x1, all_y1, and all_z1 are as described earlier
    # Also, assuming you want to plot the trajectory for drone 0 (change as needed)

    num_drones = len(all_positions[0])  # Number of drones

    # Generate a list of unique colors for each drone
    colors = plt.cm.viridis(np.linspace(0, 1, num_drones))

    # Create a 3D plot for the trajectory of the selected drone
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(f'Trajectory of Drones Over Time')

    for i in range(num_drones):
        # Plot the trajectory of the selected drone
        ax.plot(all_positions[:][i][0], all_positions[:][i][1], all_positions[:][i][2], label=f'Drone {i}', color=colors[i])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

    # # Create a 3D plot for all positions
    # fig1 = plt.figure(figsize=(10, 6))
    # ax1 = fig1.add_subplot(111, projection='3d')
    # ax1.set_title('All Positions Over Time')

    # num_drones = len(all_x)
    # for i in range(num_drones):
    #     ax1.scatter(all_x[i], all_y[i], all_z[i], label=f'Drone {i+1}')

    # ax1.set_xlabel('X')
    # ax1.set_ylabel('Y')
    # ax1.set_zlabel('Z')
    # ax1.legend()

    centroid_velocity = []
    ts = 0.0032

    for i in range(len(avg_positions) - 1):
        aux = np.linalg.norm(avg_positions[i+1] - avg_positions[i])/sample_time[i]
        print(aux)
        centroid_velocity.append(aux)
    
    
    print("DIST ",len(avg_positions), len(centroid_velocity))

    # Create a figure for the velocity magnitude plot
    fig3, ax3 = plt.subplots(figsize=(8, 4))
    ax3.plot(sample_time, centroid_velocity, label='Velocity Magnitude', color='blue')
    ax3.set_title('Velocity Magnitude of Centroid Over Time')
    ax3.set_xlabel('Time')
    ax3.set_ylabel('Velocity Magnitude')
    ax3.legend()

    plt.show()




# Example usage:
# Replace 'pos' and 'avg_position' with your actual data
# Set 'sample_time' and 'Title' as desired
# plot_figures(pos, avg_position, sample_time, Title)


def control_loop(pos, coeff_vec, threshold_distance, drone_radius, Title):
    a, b, c = coeff_vec[0], coeff_vec[1], coeff_vec[2]

    init_pos = pos

    xlim = (0, pos[:, 0].max() * 1.1)
    ylim = (0, pos[:, 1].max() * 1.1)
    zlim = (0, pos[:, 2].max() * 1.1)

    iteration = 0
    centroid_dist = []
    collisions = 0
    sampling_time = 0.0032
    
    avg_pos = np.mean(pos, axis=0) # centroid position

    pos_list = []
    avg_pos_list = []
    time_list = []
    pos_list.append(pos)
    avg_pos_list.append(avg_pos)
    
    while True:
        start_time = time.time()
        iteration += 1

        control_forces = calculate_control_forces(pos, a, b, c, drone_radius)
        
        # Define the desired maximum magnitude 'x'
        x = 5  # You can change this value as needed


        # Find the indices where the magnitudes are not zero to avoid division by zero
        nonzero_indices = np.where(control_forces != 0)

        if iteration == 1:
            max_control_value = np.max(control_forces)
            min_control_value = np.min(control_forces)

            max_magnitude = max_control_value*(max_control_value >= min_control_value) - min_control_value*(max_control_value < min_control_value)

            print("MAX MAGNITUDE = ", max_magnitude)
        

        # Normalize the control forces while preserving directions
        control_forces[nonzero_indices] = control_forces[nonzero_indices] * x / max_magnitude


        prev_pos = pos
        pos = update_positions(pos, control_forces, sampling_time)
        avg_pos = np.mean(pos, axis=0) # centroid position

        dist_matrix = squareform(pdist(pos))
        
        avg_dist_to_centroid = np.mean(np.linalg.norm(pos - avg_pos, axis=1))
        dist_to_centroid = np.linalg.norm(pos - avg_pos, axis=1)
        
        centroid_dist.append(np.abs(avg_dist_to_centroid).max())

        if iteration % 10 == 0:
            pos_list.append(pos)
            print(avg_dist_to_centroid)
            avg_pos_list.append(avg_pos)
            time_list.append(iteration*sampling_time)

        aux_matrix = dist_matrix
        np.fill_diagonal(aux_matrix, np.inf)

        # Check for collisions
        if np.any(aux_matrix <= 2*drone_radius):
            #print("Collisions detected!", dist_matrix)
            indices = np.where(aux_matrix <= 2*drone_radius)
            print("INDEXES = ", indices)
            
            # zip row and column indices into coordinate pairs
            coordinate_pairs = list(zip(indices[0], indices[1]))

            for pair in coordinate_pairs:
                print("CONTROL FORCE DRONE ", pair[0], " = ", control_forces[pair[0]])
                print("POSITION Xi = ", pos[pair[0]], "PREV POSITION = ",prev_pos[pair[0]])
                print("DISTANCE BETWEEN THEM = ", np.linalg.norm(pos[pair[0]] - pos[pair[1]]))

            break
            collisions = collisions + 1 
            
        np.fill_diagonal(dist_matrix, 0)

        
        #print("MAX DISTANCE TO CENTROID = ", dist_to_centroid.max(), "\n")

        # if np.all(dist_to_centroid <= threshold_distance):
        #     print(dist_to_centroid)
        #     print("COLLISIONS =  ",collisions)
        #     print("DISTANCE IS ",threshold_distance)
        #     break

        
        #print("ITER = ", iteration, "  ---   " ,pos_list[0][1])

        if dist_to_centroid.max() <= 7:
            plot_figures(pos_list, avg_pos_list, time_list, Title)
            #print(len(pos_list))
            break


    # Compute the sphere's center
    sphere_center = np.mean(pos, axis=0)
    
    #plot_figures(pos_list, avg_pos_list, sampling_time,Title)

    return



def lin_attraction_bounded_repulsion(pos, threshold_distance, equi_dist, drone_radius):
    n_drones = len(pos)
    #a = threshold_distance/n_drones
    a = 0.1
    b = threshold_distance*a
    c = (equi_dist**2)/(math.log(b/a))

    if threshold_distance < drone_radius*np.power(n_drones, 1/3):
        print("The distance is too small to aggregate taking into consideration the number of drones and each drones' radius\n")
        return
    
    print("a = ", a, "b = ", b, "c = ", c, "\n")

    control_loop(pos, [a, b, c], threshold_distance, drone_radius, 'Linear Attraction and Bounded from Below Repulsion')
        
        


def main():
    plt.ion() # turns on interactive mode

    n_drones = 30
    pos = 40 * np.random.rand(n_drones, 3) - 5
    threshold_distance = 5
    equi_dist = 3
    
    drone_radius = 0.35
    lin_attraction_bounded_repulsion(pos, threshold_distance, equi_dist, drone_radius)

    plt.show(block=True)


if __name__ == "__main__":
    main()
