import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
from statistics import mean
from scipy.spatial.distance import pdist, squareform

a_sigma = np.array([0, 0, 0.3])
c_sigma = np.array([2, 2, 2]).T

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
                u[i] -= y_d * (a - b * np.exp(-((y_norm - 4*drone_radius)**2) / c))

        u[i][0] = u[i][0] - a_sigma[0]
        u[i][1] = u[i][1] - a_sigma[1]
        u[i][2] = u[i][2] - a_sigma[2]

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


def plot_figures(all_positions, avg_positions, sample_time, Title, centroid_velocity, min_distance, avg_dist_to_centroid):

   # Assuming all_x1, all_y1, and all_z1 are as described earlier
    # Also, assuming you want to plot the trajectory for drone 0 (change as needed)

    num_drones = len(all_positions[0])  # Number of drones

    # # Create a figure for the velocity magnitude plot
    # fig1, ax1 = plt.subplots(figsize=(8, 4))
    # ax1.plot(sample_time, avg_dist_to_centroid, label='Distance', color='blue')
    # ax1.set_title('Average distance to the centroid')
    # ax1.set_xlabel('Time')
    # ax1.set_ylabel('distance')
    # ax1.legend()

    # # Add grid lines
    # ax1.grid(True)

    # plt.show()

    # # Create a figure for the velocity magnitude plot
    # fig4, ax4 = plt.subplots(figsize=(8, 4))
    # ax4.plot(sample_time, min_distance, label='distance', color='blue')
    # ax4.set_title('Minimum distance between agents')
    # ax4.set_xlabel('Time')
    # ax4.set_ylabel('Distance')
    # ax4.legend()

    # # Add grid lines
    # ax1.grid(True)

    # plt.show()


    # Create a figure for the plots with subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)

    # First subplot
    ax1.plot(sample_time, avg_dist_to_centroid, label='Average Distance', color='blue')
    ax1.set_title('Average Distance to the Centroid')
    ax1.set_ylabel('Distance')
    ax1.legend()
    ax1.grid(True)

    # Second subplot
    ax2.plot(sample_time, min_distance, label='Minimum Distance', color='blue')
    ax2.set_title('Minimum Distance Between Agents')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Distance')
    ax2.legend()
    ax2.grid(True)

    # Adjust spacing between subplots
    plt.tight_layout()

    # Show the combined plot
    plt.show()
    

    # Create a 3D plot for average positions over time
    fig2 = plt.figure(figsize=(10, 6))
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.set_title('Centroid Position Over Time')

    # Assuming avg_positions is a list of lists where each inner list represents x, y, and z coordinates
    x_positions = [point[0] for point in avg_positions]  # Extract x-coordinates
    y_positions = [point[1] for point in avg_positions]  # Extract y-coordinates
    z_positions = [point[2] for point in avg_positions]  # Extract z-coordinates

    ax2.plot(x_positions, y_positions, z_positions, label='Average Position', color='blue')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.legend()

    plt.show()

    

    # Create a figure for the velocity magnitude plot
    fig3, ax3 = plt.subplots(figsize=(8, 4))
    ax3.plot(sample_time, centroid_velocity, label='Velocity Magnitude', color='blue')
    ax3.set_title('Velocity Magnitude of Centroid Over Time')
    ax3.set_xlabel('Time')
    ax3.set_ylabel('Velocity Magnitude')
    ax3.legend()

    # Set the y-axis limits to be between 1 and -1
    ax3.set_ylim(-1.25*np.abs(centroid_velocity[0]), 1.25*np.abs(centroid_velocity[0]))

    # Add grid lines
    ax3.grid(True)

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
    sampling_time = 0.032
    
    avg_pos = np.mean(pos, axis=0) # centroid position

    pos_list = []
    avg_pos_list = []
    time_list = []
    centroid_vel = []
    avg_dist_centr = []
    min_dist_agents = []
    pos_list.append(pos)
    avg_pos_list.append(avg_pos)
    
    while True:
        start_time = time.time()
        iteration += 1

        control_forces = calculate_control_forces(pos, a, b, c, drone_radius)
        
        # # Define the desired maximum magnitude 'x'
        # x = 5  # You can change this value as needed


        # # Find the indices where the magnitudes are not zero to avoid division by zero
        # nonzero_indices = np.where(control_forces != 0)

        # if iteration == 1:
        #     max_control_value = np.max(control_forces)
        #     min_control_value = np.min(control_forces)

        #     max_magnitude = max_control_value*(max_control_value >= min_control_value) - min_control_value*(max_control_value < min_control_value)
            
        #     real_centroid_velocity = np.linalg.norm(a_sigma)*x / max_magnitude
        #     print("\nMAX MAGNITUDE = ", max_magnitude, "CENTROID VELOCITY AT EQUILIBRIUM = ", real_centroid_velocity)
        

        # # Normalize the control forces while preserving directions
        # control_forces[nonzero_indices] = control_forces[nonzero_indices] * x / max_magnitude


        prev_pos = pos
        pos = update_positions(pos, control_forces, sampling_time)
        last_avg_pos = avg_pos
        avg_pos = np.mean(pos, axis=0) # centroid position
        centroid_velocity = np.linalg.norm(last_avg_pos - avg_pos)/sampling_time
        centroid_velocity1 = np.sqrt((last_avg_pos[0] - avg_pos[0])**2 + (last_avg_pos[1] - avg_pos[1])**2 + (last_avg_pos[2] - avg_pos[2])**2)/sampling_time



        dist_matrix = squareform(pdist(pos))
        
        avg_dist_to_centroid = np.mean(np.linalg.norm(pos - avg_pos, axis=1))
        dist_to_centroid = np.linalg.norm(pos - avg_pos, axis=1)
        
        centroid_dist.append(np.abs(avg_dist_to_centroid).max())

        aux_matrix = dist_matrix
        np.fill_diagonal(aux_matrix, np.inf)

        if iteration % 1 == 0:
            pos_list.append(pos)
            print("\nAVERAGE DISTANCE = ", avg_dist_to_centroid, "||  LEAST DISTANCE = ", aux_matrix.min(), "||  ITER = ", iteration)
            avg_dist_centr.append(avg_dist_to_centroid)
            min_dist_agents.append(aux_matrix.min())
            #print("\nEXPECTED VEL = ", centroid_velocity1, "VEL = ", centroid_velocity)

            avg_pos_list.append(avg_pos)
            time_list.append(iteration*sampling_time)
            centroid_vel.append(centroid_velocity)

        

        # Check for collisions
        if np.any(aux_matrix <= 2*drone_radius):
            plot_figures(pos_list, avg_pos_list, time_list, Title, centroid_vel, min_dist_agents, avg_dist_centr)
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

        if dist_to_centroid.max() <= 1 or iteration >= 8000:
            plot_figures(pos_list, avg_pos_list, time_list, Title, centroid_vel, min_dist_agents, avg_dist_centr)
            print("Centroid Velocity should be ", np.linalg.norm(a_sigma))
            print("Average distance to centroid = ", avg_dist_to_centroid)
            break


    # Compute the sphere's center
    sphere_center = np.mean(pos, axis=0)
    
    #plot_figures(pos_list, avg_pos_list, sampling_time,Title)

    return



def lin_attraction_bounded_repulsion(pos, threshold_distance, equi_dist, drone_radius):
    n_drones = len(pos)
    #a = threshold_distance/n_drones
    a = 0.01
    b = threshold_distance*a
    c = (equi_dist**2)/(math.log(b/a))

    equilib_dist = np.sqrt(c*(math.log(b/a)))

    if threshold_distance < drone_radius*np.power(n_drones, 1/3):
        print("The distance is too small to aggregate taking into consideration the number of drones and each drones' radius\n")
        return
    
    print("a = ", a, "b = ", b, "c = ", c, "\n")

    control_loop(pos, [a, b, c], threshold_distance, drone_radius, 'Linear Attraction and Bounded from Below Repulsion')
        
        


def main():
    plt.ion() # turns on interactive mode

    n_drones = 30
    pos = 100 * np.random.rand(n_drones, 3) - 50
    threshold_distance = 8
    equi_dist = 7
    
    drone_radius = 0.1
    lin_attraction_bounded_repulsion(pos, threshold_distance, equi_dist, drone_radius)

    plt.show(block=True)


if __name__ == "__main__":
    main()
