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

import csv

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



DRONE_RADIUS = 0.05
FLYING_ATTITUDE = 3*np.random.rand()
TYPE1 = 'LINEAR ATRTACTION AND BOUNDED REPULSION CASE'
TYPE2 = 'LINEARLY BOUNDED FROM BELOW ATTRACTION AND UNBOUNDED REPULSION CASE'
TYPE3 = 'ALMOST CONSTANT ATTRACTION AND UNBOUNDED REPULSION CASE'

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

        #Initialize social foraging state
        self.social_state = 0
        self.Centroid_Velocity = 0.1

        self.c_sigma = np.array([8, 8, 8]).T
        #A_sigma = np.array([Centroid_Velocity, Centroid_Velocity, -Centroid_Velocity])

        self.A_sigma = self.Centroid_Velocity

        # Modify the file name based on self.my_id
        self.file_name = f'drone_{self.my_id}_data_table.csv'

        with open(self.file_name, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['DRONE', 'Timestamp', 'Average Distance to Centroid', 'CENTROID_POS', 'POSITION'])  # Write header


    def send_message(self):
        my_id = self.my_id
        emitter = self.emitter

        message = self.pos[my_id][0], self.pos[my_id][1], self.pos[my_id][2]
        message_to_send = f"{my_id}:{message}:{self.OPERATIONAL}:{self.is_aggregated}"
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
            sender_name, message_content, is_operat, is_aggregated = received_message.split(":")

            sender_id = int(sender_name)

            #store the operational flag
            self.all_op[sender_id] = int(is_operat)

            #store the aggregation flag
            self.all_aggreg[sender_id] = int(is_aggregated)
            
            # Removing parenthesis and splitting the string into list
            message_content = message_content.strip('()').split(',')

            # Converting each element in the list to float
            message_content = [float(i) for i in message_content]

            self.pos[sender_id] = message_content
            receiver.nextPacket() # move to the next message in the queue  

        


    def get_init_sensor_values(self):
        gps = self.gps
        robot = self.robot
        
        self.past_x_global = gps.getValues()[0]
        self.past_y_global = gps.getValues()[1]
        self.altitude_rate = 0
        self.past_altitude = gps.getValues()[2]
        self.past_time = robot.getTime()

        self.height_desired = FLYING_ATTITUDE

        self.pos = np.zeros((21, 3))

        #initualize control variables
        n_drones = len(self.pos)
        self.u = np.zeros([n_drones, 3])
        self.avg_pos = np.zeros(3)

        self.OPERATIONAL = 0
        self.all_op = np.zeros(n_drones)
        
        self.all_aggreg = np.zeros(n_drones)
        self.is_aggregated = 0

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
        DELTA_H_THRESHOLD = 0.05
        Vz_THRESHOLD = 0.03

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
        
        
        self.m1_motor.setVelocity(-motor_power[0])
        self.m2_motor.setVelocity(motor_power[1])
        self.m3_motor.setVelocity(-motor_power[2])
        self.m4_motor.setVelocity(motor_power[3])    

    
    def aggregate(self):
        pos = self.pos
        #dist_matrix = squareform(pdist(pos))
        n_drones = len(pos)
        my_id = self.my_id


        threshold_distance = 0.7

        if self.motion_mode == 0:
            self.aggreg_mode = 0
            if self.my_id == 0:
                print("SELECT RESOURCE PROFILE: \n '1' PLANE \n '2' QUADRATIC \n '3' FOR ALM CONST ATT AND UNBOUND REP")
            
            key = self.keyboard.getKey()  
            
            while key <= 0:
                key = self.keyboard.getKey() 
                self.robot.step(self.timestep)
            

            self.aggreg_mode = int(key) - 48


            if self.aggreg_mode == 1:
                self.motion_mode = 1
                equilib_dist = threshold_distance*0.6
                self.a = 0.015
                self.b = self.a*threshold_distance
                self.c = (equilib_dist**2)/math.log(self.b/self.a)

                if self.c <= 0:
                    self.c = 0.001

                print("LINEAR ATRTACTION AND BOUNDED REPULSION CASE \n PARAMETERS: a = ", self.a, " || b = ", self.b, " || c = ", self.c)
            
            elif self.aggreg_mode == 2:
                self.motion_mode = 1
                self.a = 0.005
                self.b = (threshold_distance**2)*2*self.a/n_drones
                self.c = 0

                print("LINEARLY BOUNDED FROM BELOW ATTRACTION AND UNBOUNDED REPULSION CASE \n PARAMETERS: a = ", self.a, " || b = ", self.b)

            elif self.aggreg_mode == 3:
                self.motion_mode = 1
                self.a = 0.015
                self.b = threshold_distance*self.a/n_drones
                self.c = 0

                print("ALMOST CONSTANT ATTRACTION AND UNBOUNDED REPULSION CASE \n PARAMETERS: a = ", self.a, " || b = ", self.b)

            else:
                print("WRONG KEYBOARD INPUT \n")
                return

            key = self.keyboard.getKey()

        
        u = np.zeros(3)

        avg_pos = self.avg_pos
        a, b, c = self.a, self.b, self.c

        #calculate distances between agents
        dist_matrix = squareform(pdist(pos))

        # calculate avg distances to centroid and the distance of each individual to the centroid
        avg_dist_to_centroid = np.mean(np.linalg.norm(pos - avg_pos, axis=1))

        self.write_in_file(self.robot.getTime(), avg_dist_to_centroid, self.pos[self.my_id])

        dist_to_centroid = np.linalg.norm(pos - avg_pos, axis=1)
        #print(avg_dist_to_centroid)

        # Find and print the largest distance
        largest_distance = np.max(dist_to_centroid)   

        # if self.social_state == 0 and self.aggreg_mode > 0:
        #     self.social_state = 1
        # elif self.social_state == 1 and self.avg_pos[0] <= -6:
        #     self.social_state = 2
        # elif self.social_state == 2 and self.avg_pos[1] <= -6:
        #     self.social_state = 3
        # elif self.social_state == 3 and self.avg_pos[0] >= 6:
        #     self.social_state = 4
        # elif self.social_state == 4 and self.avg_pos[1] >= 6:
        #     self.social_state = 1
    
    
        #A_sigma = [0, 0, 0]*(self.social_state == 0) + [self.Centroid_Velocity, 0, 0]*(self.social_state == 1) + [0, self.Centroid_Velocity, 0]*(self.social_state == 2) + [-self.Centroid_Velocity, 0, 0]*(self.social_state == 3) + [0, -self.Centroid_Velocity, 0]*(self.social_state == 4)

        
        #print("\n SOCIAL STATE = ", self.social_state, "A = ", A_sigma, "\n centroid = ", self.avg_pos)
	

        if np.any(dist_to_centroid >= threshold_distance) or True:
                i = my_id
                
                u = 0
                for j in range(n_drones):
                    if i != j:
                        y_d = pos[i] - pos[j]
                        y_norm = dist_matrix[i, j]
                        if self.aggreg_mode == 2:
                            u -= y_d * (a - b/(((y_norm**2) - 4*DRONE_RADIUS**2)**2))
                            resource_profile = self.A_sigma

                        elif self.aggreg_mode == 3:
                            u -= y_d * (a - b/(((y_norm**2) - 4*DRONE_RADIUS**2)**2))
                            resource_profile = self.A_sigma*(pos[i] - self.c_sigma)

                        elif self.aggreg_mode == 1:
                            u -= y_d * (a - b * np.exp(-(((y_norm**2) - 4*DRONE_RADIUS**2)**2) / c))
                            resource_profile = self.A_sigma*(pos[i] - self.c_sigma)

                        else:
                            return  

                vx_p = u[0] - resource_profile[0]
                vy_p = u[1] - resource_profile[1]
                vz_p = u[2] - resource_profile[2]


        else:
            vx_p = 0
            vy_p = 0
            vz_p = 0
        
        if np.linalg.norm(self.avg_pos - self.c_sigma, axis=0) <= 1.5 and self.A_sigma >= 0:
            self.A_sigma = -2*self.Centroid_Velocity
            self.c_sigma = self.avg_pos.T + ((self.c_sigma - self.avg_pos)/4).T
            print("\n \n \n ENTREI AQUI \n \n \n")
            print("NEW C_SIGMA = ", self.c_sigma)
        else:
            print("\n centroid = ", self.avg_pos, "\n ERROR = ", np.linalg.norm(self.avg_pos - self.c_sigma, axis=0))

        # if self.my_id == 0:
        #     print(u)

        return vx_p, vy_p, vz_p

    def moving_average(self, data, window_size):
        """Compute the moving average of a list. If window_size > len(data), returns the average of the whole list."""
        
        if window_size > len(data):
            return sum(data) / len(data)
        else:
            return sum(data[-window_size:]) / window_size
    
    def write_in_file(self, timestamp, avg_dist_2_centroid, position):
        # Convert the array to a comma-separated string
        avg_pos_str = ','.join(map(str, self.avg_pos))
        # Write timestamp and avg_dist_to_centroid to CSV
        with open(self.file_name, 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([self.my_id, timestamp, avg_dist_2_centroid, avg_pos_str, position])
        

    def control_loop(self):
        self.get_init_sensor_values()

        self.motion_mode = 0  # 1 for aggregation; 2 for social foraging; 3 for formation control 

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
            
            #store the operational flag on the operational array    
            self.all_op[self.my_id] = self.OPERATIONAL

            #store aggregation flag 
            self.all_aggreg[self.my_id] = self.is_aggregated

            #send_position and operational flag to the other drones and receive their positions and operational flags 
            self.send_message()
            self.receive_message()

            # if self.my_id == 3:
            #     print("\nTIMESTAMP = ", format(self.robot.getTime(), ".3f"), "OPERATIONAL = ", self.OPERATIONAL, "\n")

            #aggregation process when all drones are operational
            if np.all(self.all_op) == 1 and np.any(self.all_aggreg) == 0:
                self.avg_pos = np.mean(self.pos, axis = 0) 
                self.forward_desired, self.sideways_desired, height_diff_desired = self.aggregate()

            
            if np.all(self.all_aggreg) == 1:
                self.motion_mode = 0
                print("a = ", self.a, "b = ", self.b)
            
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

