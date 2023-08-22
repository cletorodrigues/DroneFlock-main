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
        # Code to initialize motors, receiver, emitter, etc.

    def init_sensors(self):
        # Code to initialize IMU, GPS, gyro, etc.

    def send_message(self, message, is_operat):
        # The message sending logic here

    def get_positions(self):
        # Logic to fetch positions...

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


