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


DRONE_RADIUS = "0.05"
FLYING_ATTITUDE = "1"


class CrazyflieDrone:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.init_devices()
        # Other initializations...
    
    def init_devices(self):
        ## Initialize motors
        self.m1_motor = self.robot.getDevice("m1_motor");
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor");
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor");
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor");
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        ## Initialize Sensors
        self.imu = self.robot.getDevice("inertial_unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        self.range_front = self.robot.getDevice("range_front")
        self.range_front.enable(self.timestep)
        self.range_left = self.robot.getDevice("range_left")
        self.range_left.enable(self.timestep)
        self.range_back = self.robot.getDevice("range_back")
        self.range_back.enable(self.timestep)
        self.range_right = self.robot.getDevice("range_right")
        self.range_right.enable(self.timestep)

        ## Get keyboard
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        #get device name
        robot_name = self.robot.getName()
        #print(robot_name)	

        self.my_id = int(robot_name)

        # Get emitter
        self.emitter = self.robot.getDevice("emitter")
        
        #Get receiver
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.timestep)

        # Crazyflie velocity PID controller
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = self.robot.getTime()
        self.sensor_read_last_time = self.robot.getTime()


    def communicate(self):
        def send_message(self):
            my_id = self.my_id
            emitter = self.emitter

            message = self.pos[my_id][0], self.pos[my_id][1], self.pos[my_id][2]
            message_to_send = f"{my_id}:{message}:{self.OPERATIONAL}"
            emitter.send(message_to_send.encode('utf-8'))

        def receive_message(self):
            receiver = self.receiver
            my_id = self.my_id
            
            TIMEOUT = 50  # This can be adjusted based on your needs.

            timeout_counter = 0
            while receiver.getQueueLength() < 7:
                if timeout_counter > TIMEOUT:
                    print(f"Timeout occurred for Drone {my_id} waiting for messages. Proceeding with {receiver.getQueueLength()} messages.")
                    break
                
                timeout_counter += 1

            while receiver.getQueueLength() > 0:
                received_message = receiver.getString()
                sender_name, message_content, is_operat = received_message.split(":")

                sender_id = int(sender_name)

                #store the operational flag
                self.all_op[sender_id] = int(is_operat)
                
                # Removing parenthesis and splitting the string into list
                message_content = message_content.strip('()').split(',')

                # Converting each element in the list to float
                message_content = [float(i) for i in message_content]

                self.pos[sender_id] = message_content
                receiver.nextPacket() # move to the next message in the queue  

        send_message()
        receive_message()


    def get_init_sensor_values(self):
        gps = self.gps
        robot = self.robot
        
        self.past_x_global = gps.getValues()[0]
        self.past_y_global = gps.getValues()[1]
        self.altitude_rate = 0
        self.past_altitude = gps.getValues()[2]
        self.past_time = robot.getTime()

        self.height_desired = FLYING_ATTITUDE

        self.pos = np.zeros((8, 3))

        #initualize control variables
        n_drones = len(self.pos)
        self.u = np.zeros([n_drones, 3])
        self.avg_pos = np.zeros(3)

        self.OPERATIONAL = 0
        self.all_op = np.zeros(8)

        while np.isnan(self.past_x_global) or np.isnan(self.past_y_global) or np.isnan(self.past_altitude):
            robot.step(self.timestep)
            self.past_x_global = gps.getValues()[0]
            self.past_y_global = gps.getValues()[1]
            self.past_altitude = gps.getValues()[2]
        
        return
    
    def get_sensor_values(self):
        self.dt = self.robot.getTime() - self.past_time
        
        ## Get sensor data
        self.roll = self.imu.getRollPitchYaw()[0]
        self.pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        self.yaw_rate = self.gyro.getValues()[2]
        self.altitude = self.gps.getValues()[2]
        self.x_global = self.gps.getValues()[0]
        v_x_global = (self.x_global - self.past_x_global)/self.dt
        self.y_global = self.gps.getValues()[1]
        v_y_global = (self.y_global - self.past_y_global)/self.dt
        self.altitude_rate = (self.altitude - self.past_altitude)/self.dt

        
        ## Get body fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        self.v_x = v_x_global * cosyaw + v_y_global * sinyaw
        self.v_y = - v_x_global * sinyaw + v_y_global * cosyaw

    
    def stabilize(self):
        MAX_LIST_SIZE = 100  
        DELTA_H_THRESHOLD = 0.01
        Vz_THRESHOLD = 0.05

        # Limit the size of lists
        if len(self.Delta_H_LIST) > MAX_LIST_SIZE:
            self.Delta_H_LIST.pop(0)
        
        if len(self.Vz_LIST) > MAX_LIST_SIZE:
            self.Vz_LIST.pop(0)


        self.Delta_H_LIST.append(np.abs(self.altitude - self.height_desired))
        self.Vz_LIST.append(np.abs(self.altitude_rate))

        Delta_H_ma = self.moving_average(self.Delta_H_LIST, 50)
        Vz_ma = self.moving_average(self.Vz_LIST, 50)

        if Delta_H_ma <= DELTA_H_THRESHOLD and Vz_ma <= Vz_THRESHOLD and self.OPERATIONAL == 0:
            self.OPERATIONAL = 1




    def send_output(self):
        ## PID velocity controller with fixed height
        motor_power = self.PID_CF.pid(self.dt, self.forward_desired, self.sideways_desired,
                                self.yaw_desired, self.height_desired,
                                self.roll, self.pitch, self.yaw_rate,
                                self.altitude, self.v_x, self.v_y)
        
        # if my_id == 2:
        #     print("dt = ", dt, "roll = ", roll, "pitch = ", pitch, "yaw_rate =", yaw_rate, "altitude = ", altitude,"\n motor_power = ", motor_power)
        
        self.m1_motor.setVelocity(-motor_power[0])
        self.m2_motor.setVelocity(motor_power[1])
        self.m3_motor.setVelocity(-motor_power[2])
        self.m4_motor.setVelocity(motor_power[3])    

    
    def aggregate(self, coeff_vec, threshold_distance):
        pos = self.pos

        n_drones = len(pos)
        dist_matrix = squareform(pdist(pos))
        my_id = self.my_id

        a, b = coeff_vec[0], coeff_vec[1]

        u = np.zeros(3)

        avg_pos = self.avg_pos

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
                        u[i] -= y_d * (a/y_norm - b/(((y_norm**2) - 4*DRONE_RADIUS**2)**2))
                        

                vx_p = u[my_id][0]
                vy_p = u[my_id][1]
                vz_p = u[my_id][2]

        else:
            print("DRONE ", my_id, "CONFIRMS AGGREGATION IS COMPLETE")

            vx_p = 0
            vy_p = 0
            vz_p = 0

        
        return vx_p, vy_p, vz_p

    def moving_average(self, data, window_size):
        """Compute the moving average of a list. If window_size > len(data), returns the average of the whole list."""
        
        if window_size > len(data):
            return sum(data) / len(data)
        else:
            return sum(data[-window_size:]) / window_size
    
    def control_loop(self):
        self.get_init_sensor_values()
        
        #initialize attraction/repulsion function's parameters
        a, b, c = 0.16, 0.01, 2.885
        coeff_vec = [a, b, c]

        #define convergence distance
        threshold_distance = 1

        #create lists for the stabilizing process    
        self.Delta_H_LIST = []
        self.Vz_LIST = []

        while self.robot.step(self.timestep) != -1:
            # Main loop logic...
            
            #get sensor data
            self.get_sensor_values()
            
            ## Initialize values
            self.forward_desired = 0
            self.sideways_desired = 0
            self.yaw_desired = 0
            height_diff_desired = 0
            
            #assign position
            self.pos[self.my_id] = self.x_global, self.y_global, self.altitude
            
            #check if the drone is already operational
            if self.OPERATIONAL == 0:
                self.stabilize() 
            
            #send_position and operational flag to the other drones and receive their positions and operational flags 
            self.communicate()

            #store the operational flag on the operational array    
            self.all_op[self.my_id] = self.OPERATIONAL
            
            #aggregation process when all drones are operational
            if np.all(self.all_op) == 1:    
                self.forward_desired, self.sideways_desired, height_diff_desired = self.aggregate(coeff_vec, threshold_distance)

            #calculate the desired altitute    
            self.height_desired += height_diff_desired * self.dt

            #send desired velocities and desired altitude (vx, vy, xz)    
            self.send_output()


            self.past_time = self.robot.getTime()
            self.past_x_global = self.x_global
            self.past_y_global = self.y_global
            self.past_altitude = self.altitude


if __name__ == '__main__':
    drone = CrazyflieDrone()
    drone.control_loop()


