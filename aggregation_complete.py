import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
from statistics import mean
from scipy.spatial.distance import pdist, squareform


def calculate_control_forces(pos, a, b, c, mode, drone_radius):
    n_drones = len(pos)
    dist_matrix = squareform(pdist(pos))
    u = np.zeros([n_drones, 3])

    for i in range(n_drones):
        u[i] = 0
        for j in range(n_drones):
            if i != j:
                y_d = pos[i] - pos[j]
                y_norm = dist_matrix[i, j]
                if mode == 1:
                    u[i] -= y_d * (a - b * np.exp(-(y_norm**2) / c))

                elif mode == 2:
                    u[i] -= y_d * (a - b/(y_norm**2))

                elif mode == 3:
                    u[i] -= y_d * (a/y_norm - b/(y_norm**2))

                elif mode == 4:
                    u[i] -= y_d * (a/y_norm - b/(((y_norm**2) - 4*drone_radius**2)**2))

                else:
                    break

    return u


def update_positions(pos, u, elapsed_time):
    return pos + u * elapsed_time

def plot_positions(pos, xlim=None, ylim=None, zlim=None):
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

def plot_figures(figure_numb, init_pos, pos, xlim, ylim, zlim, sphere_center, threshold_distance, centroid_dist, drone_radius, Title):
    fig = plt.figure(figure_numb, figsize=(15, 4))  # adjust the size as necessary

    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    ax3 = fig.add_subplot(1, 3, 3)

    # Plot 1
    ax1.set_xlabel('X Label')
    ax1.set_ylabel('Y Label')
    ax1.set_zlabel('Z Label')
    ax1.set_xlim(xlim)
    ax1.set_ylim(ylim)
    ax1.set_zlim(zlim)
    ax1.scatter(init_pos[:, 0], init_pos[:, 1], init_pos[:, 2])
    ax1.set_title('Initial positions')

    # Plot 2
    ax2.set_xlabel('X Label')
    ax2.set_ylabel('Y Label')
    ax2.set_zlabel('Z Label')
    ax2.set_xlim(xlim)
    ax2.set_ylim(ylim)
    ax2.set_zlim(zlim)
    ax2.scatter(pos[:, 0], pos[:, 1], pos[:, 2])
    ax2.set_title('Final Position')

    # Create the sphere and plot it
    x, y, z = create_sphere(sphere_center, threshold_distance)
    ax2.plot_surface(x, y, z, color='b', alpha=0.2)

    if figure_numb == 4:
        # Create and plot a sphere for each drone
        for drone_pos in pos:
            x, y, z = create_sphere(drone_pos, drone_radius)
            ax2.plot_surface(x, y, z, color='r', alpha=0.1)
    

    # Plot 3
    ax3.plot(centroid_dist)
    ax3.set_title("Average distance to centroid over iterations")
    ax3.set_xlabel("Iteration")
    ax3.set_ylabel("Average distance to centroid")

    # Title for the whole figure
    fig.suptitle(Title, fontsize=16)
     
    plt.show()

    return

def control_loop(pos, mode, coeff_vec, threshold_distance, drone_radius, Title):
    a, b, c = coeff_vec[0], coeff_vec[1], coeff_vec[2]

    avg_pos = np.mean(pos, axis=0) # centroid position

    init_pos = pos

    xlim = (0, pos[:, 0].max() * 1.1)
    ylim = (0, pos[:, 1].max() * 1.1)
    zlim = (0, pos[:, 2].max() * 1.1)

    iteration = 0
    centroid_dist = []
    collisions = 0
    sampling_time = 0.001


    while True:
        start_time = time.time()
        iteration += 1

        control_forces = calculate_control_forces(pos, a, b, c, mode, (mode == 4)*drone_radius)
        # if np.abs(control_forces).max() >= 50:
        #     control_max_indexes = np.where(np.abs(control_forces) >= 50)
        #     print(control_max_indexes)

        prev_pos = pos
        pos = update_positions(pos, control_forces, time.time() - start_time)

        dist_matrix = squareform(pdist(pos))
        
        avg_dist_to_centroid = np.mean(np.linalg.norm(pos - avg_pos, axis=1))
        dist_to_centroid = np.linalg.norm(pos - avg_pos, axis=1)
        
        centroid_dist.append(np.abs(avg_dist_to_centroid).max())

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

        print(dist_to_centroid.max())

        if np.all(dist_to_centroid <= threshold_distance):
            print(dist_to_centroid)
            print("COLLISIONS =  ",collisions)
            print("DISTANCE IS ",threshold_distance)
            break

        prev_pos = pos
        pos = update_positions(pos, control_forces, sampling_time)


    # Compute the sphere's center
    sphere_center = np.mean(pos, axis=0)

    plot_figures(mode, init_pos, pos, xlim, ylim, zlim, sphere_center, threshold_distance, centroid_dist, (mode == 4)*drone_radius,Title)

    return



def lin_attraction_bounded_repulsion(pos, threshold_distance, equi_dist):
    mode = 1
    n_drones = len(pos)
    a = threshold_distance/n_drones
    b = threshold_distance*a
    c = (equi_dist**2)/(math.log(b/a))

    control_loop(pos, mode, [a, b, c], threshold_distance, 0, 'Linear Attraction and Bounded from Below Repulsion')



def Lin_BoundBelowAttandUnboundRep(pos, threshold_distance):
    mode = 2
    n_drones = len(pos)

    epsilon = threshold_distance/np.sqrt(n_drones)

    #epsilon = sqrt(b/2a)

    a = threshold_distance/(2*n_drones)
    b = (epsilon**2)*2*a

    control_loop(pos, mode, [a, b, 0], threshold_distance, 0, 'Linearly Bounded from Below Attraction and Unbounded Repulsion')


def AlConstAttUnboundRep(pos, threshold_distance):
    mode = 3
    n_drones = len(pos)

    epsillon = threshold_distance/n_drones

    a = (np.linalg.norm(pos - np.mean(pos, axis=0), axis=1).max())/n_drones
    b = epsillon*a

    control_loop(pos, mode, [a, b, 0], threshold_distance, 0, 'Almost Constant Attraction and Unbounded Repulsion')


def FiniteBodySize(pos, threshold_distance, drone_radius):
    mode = 4
    n_drones = len(pos)

    if threshold_distance < drone_radius*np.power(n_drones, 1/3):
        print("The distance is too small to aggregate taking into consideration the number of drones and each drones' radius\n")
        return
    
    else:
        epsillon = threshold_distance/n_drones

        a = 0.2
        b = a * (2*threshold_distance**2)/n_drones

        control_loop(pos, mode, [a, b, 0], threshold_distance, drone_radius, 'Rigid Body Case')
        print("Min radius = ", drone_radius*np.power(n_drones, 1/3))
        
        


def main():
    plt.ion() # turns on interactive mode

    n_drones = 8
    pos = 10 * np.random.rand(n_drones, 3) - 5
    threshold_distance = 7
    equi_dist = 3
    
    print(pos)
    #lin_attraction_bounded_repulsion(pos, threshold_distance, equi_dist)

    #Lin_BoundBelowAttandUnboundRep(pos, threshold_distance)

    #AlConstAttUnboundRep(pos, threshold_distance)

    drone_radius = 1.5
    #FiniteBodySize(pos, threshold_distance, drone_radius)

    plt.show(block=True)


if __name__ == "__main__":
    main()
