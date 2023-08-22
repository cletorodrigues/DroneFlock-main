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




class CrazyflieDrone:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.init_devices()
        self.init_sensors()
        # Other initializations...
    
    def init_devices(self):
        ## Initialize motors
        m1_motor = self.robot.getDevice("m1_motor");
        m1_motor.setPosition(float('inf'))
        m1_motor.setVelocity(-1)
        m2_motor = self.robot.getDevice("m2_motor");
        m2_motor.setPosition(float('inf'))
        m2_motor.setVelocity(1)
        m3_motor = self.robot.getDevice("m3_motor");
        m3_motor.setPosition(float('inf'))
        m3_motor.setVelocity(-1)
        m4_motor = self.robot.getDevice("m4_motor");
        m4_motor.setPosition(float('inf'))
        m4_motor.setVelocity(1)

        ## Initialize Sensors
        imu = self.robot.getDevice("inertial_unit")
        imu.enable(self.timestep)
        gps = self.robot.getDevice("gps")
        gps.enable(self.timestep)
        gyro = self.robot.getDevice("gyro")
        gyro.enable(self.timestep)
        range_front = self.robot.getDevice("range_front")
        range_front.enable(self.timestep)
        range_left = self.robot.getDevice("range_left")
        range_left.enable(self.timestep)
        range_back = self.robot.getDevice("range_back")
        range_back.enable(self.timestep)
        range_right = self.robot.getDevice("range_right")
        range_right.enable(self.timestep)

        ## Get keyboard
        keyboard = Keyboard()
        keyboard.enable(self.timestep)

        #get device name
        robot_name = self.robot.getName()
        #print(robot_name)	

        my_id = int(robot_name)

        # Get emitter
        emitter = self.robot.getDevice("emitter")
        
        #Get receiver
        receiver = self.robot.getDevice("receiver")
        receiver.enable(self.timestep)


    def communicate(self):
        def send_message(message, is_operat):
            message_to_send = f"{self.my_id}:{message}:{is_operat}"
            self.emitter.send(message_to_send.encode('utf-8'))

        def receive_message():
            TIMEOUT = 50  # This can be adjusted based on your needs.

            all_op = np.zeros(8)
            timeout_counter = 0
            while self.receiver.getQueueLength() < 7:
                if timeout_counter > TIMEOUT:
                    print(f"Timeout occurred for Drone {self.my_id} waiting for messages. Proceeding with {self.receiver.getQueueLength()} messages.")
                    break
            

            
        while True:
            with self.lock:
                send_message()
                receive_message()
            self.robot.step(self.timestep)


    def aggregate(self, pos, avg_pos, coeff_vec, drone_radius):
        # Logic to compute aggregate...

    def moving_average(self, data, window_size):
        # Logic for moving average...
    
    def control_loop(self):
        while self.robot.step(self.timestep) != -1:
            # Main loop logic...
            
if __name__ == '__main__':
    drone = CrazyflieDrone()
    drone.control_loop()


