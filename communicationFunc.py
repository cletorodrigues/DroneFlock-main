#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""


from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import DistanceSensor

from math import cos, sin

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
from statistics import mean
from scipy.spatial.distance import pdist, squareform


import sys
sys.path.append('../../../controllers/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1


def init():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    ## Initialize motors
    m1_motor = robot.getDevice("m1_motor");
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor");
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor");
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor");
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    ## Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)

    ## Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    #get device name
    robot_name = robot.getName()
    #print(robot_name)	

    # Get emitter
    emitter = robot.getDevice("emitter")
    
    #Get receiver
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

    return robot, timestep, m1_motor, m2_motor, m3_motor, m4_motor, imu, gps, gyro, range_front, range_left, range_back, range_right, keyboard, emitter, receiver




def send_message(emitter, my_id, message, is_operat):
    message_to_send = f"{my_id}:{message}:{is_operat}"
    emitter.send(message_to_send.encode('utf-8'))


def get_positions(pos, robot, receiver, timestep, my_id):
    TIMEOUT = 50  # This can be adjusted based on your needs.

    all_op = np.zeros(8)
    timeout_counter = 0
    while receiver.getQueueLength() < 7:
        if timeout_counter > TIMEOUT:
            print(f"Timeout occurred for Drone {my_id} waiting for messages. Proceeding with {receiver.getQueueLength()} messages.")
            break
            
        #robot.step(timestep)
        timeout_counter += 1
    

    while receiver.getQueueLength() > 0:
        received_message = receiver.getString()
        sender_name, message_content, is_operat = received_message.split(":")

        sender_id = int(sender_name)

        #store the operational flag
        all_op[sender_id] = int(is_operat)
        

        # Removing parenthesis and splitting the string into list
        message_content = message_content.strip('()').split(',')

        # Converting each element in the list to float
        message_content = [float(i) for i in message_content]

        pos[sender_id] = message_content
        receiver.nextPacket() # move to the next message in the queue

    return pos, all_op

def aggregate(pos, avg_pos, my_id, coeff_vec, drone_radius, u):
    n_drones = len(pos)
    dist_matrix = squareform(pdist(pos))

    a, b = coeff_vec[0], coeff_vec[1]

    #calculate distances between agents
    dist_matrix = squareform(pdist(pos))

    # calculate avg distances to centroid and the distance of each individual to the centroid
    avg_dist_to_centroid = np.mean(np.linalg.norm(pos - avg_pos, axis=1))

    dist_to_centroid = np.linalg.norm(pos - avg_pos, axis=1)
    #print(avg_dist_to_centroid)

    if np.any(dist_to_centroid >= threshold_distance):
            i = my_id
            u[i] = 0
            for j in range(n_drones):
                if i != j:
                    y_d = pos[i] - pos[j]
                    y_norm = dist_matrix[i, j]
                    u[i] -= y_d * (a/y_norm - b/(((y_norm**2) - 4*drone_radius**2)**2))
                    

            vx_p = u[my_id][0]
            vy_p = u[my_id][1]
            vz_p = u[my_id][2]

    else:
        print("DRONE ", my_id, "CONFIRMS AGGREGATION IS COMPLETE")

        vx_p = 0
        vy_p = 0
        vz_p = 0

    
    return vx_p, vy_p, vz_p, u

def moving_average(data, window_size):
    """Compute the moving average of a list. If window_size > len(data), returns the average of the whole list."""
    if window_size > len(data):
        return sum(data) / len(data)
    else:
        return sum(data[-window_size:]) / window_size

    

if __name__ == '__main__':
    robot, timestep, m1_motor, m2_motor, m3_motor, m4_motor, imu, gps, gyro, range_front, range_left, range_back, range_right, keyboard, emitter, receiver = init()

    ## Initialize variables

    past_x_global = gps.getValues()[0]
    past_y_global = gps.getValues()[1]
    altitude_rate = 0
    past_altitude = gps.getValues()[2]
    past_time = robot.getTime()
    

    # Crazyflie velocity PID controller
    PID_CF = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()
    

    while np.isnan(past_x_global) or np.isnan(past_y_global) or np.isnan(past_altitude):
        robot.step(timestep)
        past_x_global = gps.getValues()[0]
        past_y_global = gps.getValues()[1]
        past_altitude = gps.getValues()[2]

    
    height_desired = FLYING_ATTITUDE


    #drone radius 
    drone_radius = 0.05
    
    pos = np.zeros((8, 3))

    #initualize control variables
    n_drones = len(pos)
    u = np.zeros([n_drones, 3])

    my_id = int(robot.getName())

    #print("INIT POS DRONE " , my_id, " = ", past_x_global, past_y_global, height_desired)


    #define the goal distance
    threshold_distance = 1
    equi_dist = 1.3

    #initialize attraction/repulsion function's parameters
    a, b, c = 0.8, 0.1, 2.885

    coeff_vec = [a, b, c]

    avg_pos = np.zeros(3)

    OPERATIONAL = 0
    Delta_H_LIST = []
    Vz_LIST = []

    MAX_LIST_SIZE = 100  
    DELTA_H_THRESHOLD = 0.01
    Vz_THRESHOLD = 0.05

    all_operat = np.zeros(8)

    # Main loop:
    while robot.step(timestep) != -1:

        dt = robot.getTime() - past_time
        actual_state = {}

        ## Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        altitude = gps.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global)/dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global)/dt
        altitude_rate = (altitude - past_altitude)/dt

        #print("x_global = ", x_global, " past_x_global = ", past_x_global, "VX = ", v_x_global)
        
        ## Get body fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = - v_x_global * sinyaw + v_y_global * cosyaw

        ## Initialize values
        desired_state = [0, 0, 0, 0]
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        pos[my_id] = x_global, y_global, altitude
        message = x_global, y_global, altitude
        send_message(emitter, my_id, message, OPERATIONAL)

        pos, all_operat = get_positions(pos, robot, receiver, timestep, my_id)
        all_operat[my_id] = OPERATIONAL
        # #print("DRONE ", my_id, "\n ALL OPERATIONAL ≃ ", all_operat)


        # #calculate average position
        # avg_pos += np.mean(pos, axis=0)*np.all(avg_pos == 0) # centroid position
        # #print("DRONE ", my_id, "\n CENTROID = ", avg_pos, "\n DRONE POSITION = ", pos[my_id])



        #calculate control forces
        if np.all(all_operat) == 1:
            # if forward_desired == 0:
            #     print("ALL DRONES ARE OPERATIONAL, STARTING AGGREGATION")

            forward_desired, sideways_desired, height_diff_desired, u = aggregate(pos, avg_pos, my_id, coeff_vec, drone_radius, u)


        height_desired += 0.1*height_diff_desired * dt

        print("MY ID = ", my_id, "VX = ", forward_desired, "VY = ", sideways_desired, "VZ = ", height_diff_desired, "\n \n")    

        # Limit the size of lists
        if len(Delta_H_LIST) > MAX_LIST_SIZE:
            Delta_H_LIST.pop(0)
        
        if len(Vz_LIST) > MAX_LIST_SIZE:
            Vz_LIST.pop(0)


        Delta_H_LIST.append(np.abs(altitude - height_desired))
        Vz_LIST.append(np.abs(altitude_rate))

        Delta_H_ma = moving_average(Delta_H_LIST, 50)
        Vz_ma = moving_average(Vz_LIST, 50)

    
        if Delta_H_ma <= DELTA_H_THRESHOLD and Vz_ma <= Vz_THRESHOLD:
            OPERATIONAL = 1

        # Example how to get sensor data
        # range_front_value = range_front.getValue();
        # cameraData = camera.getImage()

        ## PID velocity controller with fixed height
        motor_power = PID_CF.pid(dt, forward_desired/10, sideways_desired/10,
                                yaw_desired, height_desired,
                                roll, pitch, yaw_rate,
                                altitude, v_x, v_y)
        
        # if my_id == 2:
        #     print("dt = ", dt, "roll = ", roll, "pitch = ", pitch, "yaw_rate =", yaw_rate, "altitude = ", altitude,"\n motor_power = ", motor_power)
        
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
        past_altitude = altitude