"""crazyflie_controller_py controller."""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
from statistics import mean
from scipy.spatial.distance import pdist, squareform

from controller import Robot
from controller import Supervisor
from controller import Node
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import Camera
from controller import DistanceSensor

from math import cos, sin

import sys
sys.path.append('../../../controllers/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1


class DroneController:
    def __init__(self, robot, drone_name):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())


        ## Initialize motors
        self.m1_motor = robot.getDevice("m1_motor");
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = robot.getDevice("m2_motor");
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = robot.getDevice("m3_motor");
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = robot.getDevice("m4_motor");
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)
        
        ## Initialize Sensors
        self.imu = robot.getDevice("inertial_unit")
        self.imu.enable(self.timestep)
        self.gps = robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        self.camera = robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.range_front = robot.getDevice("range_front")
        self.range_front.enable(self.timestep)
        self.range_left = robot.getDevice("range_left")
        self.range_left.enable(self.timestep)
        self.range_back = robot.getDevice("range_back")
        self.range_back.enable(self.timestep)
        self.range_right = robot.getDevice("range_right")
        self.range_right.enable(self.timestep)

        ## Initialize variables

        self.past_x_global = self.gps.getValues()[0]
        self.past_y_global = self.gps.getValues()[1]
        self.past_time = robot.getTime()

        # Crazyflie velocity PID controller
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = robot.getTime()
        self.sensor_read_last_time = robot.getTime()

        self.height_desired = FLYING_ATTITUDE
        
        # ... (any other initialization you need here)

    def step(self, desired_position):
        dt = robot.getTime() - self.past_time
        
        print("DT = ", dt, "desired_position = ", desired_position, "\n")

        ## Get sensor data
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        yaw_rate = self.gyro.getValues()[2]
        altitude = self.gps.getValues()[2]
        x_global = self.gps.getValues()[0]
        v_x_global = (x_global - self.past_x_global)/dt
        y_global = self.gps.getValues()[1]
        v_y_global = (y_global - self.past_y_global)/dt

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

        forward_desired = (desired_position[0] - x_global)/dt
        sideways_desired = (desired_position[1] - y_global)/dt


        print("FOR_DES = ", forward_desired, "SIDE_DES = ", sideways_desired)


        self.height_desired += height_diff_desired * dt

        ## Example how to get sensor data
        ## range_front_value = range_front.getValue();
        ## cameraData = camera.getImage()


        ## PID velocity controller with fixed height
        motor_power = self.PID_CF.pid(dt, forward_desired, sideways_desired,
                                yaw_desired, self.height_desired,
                                roll, pitch, yaw_rate,
                                altitude, v_x, v_y)

        self.m1_motor.setVelocity(-motor_power[0])
        self.m2_motor.setVelocity(motor_power[1])
        self.m3_motor.setVelocity(-motor_power[2])
        self.m4_motor.setVelocity(motor_power[3])

        self.past_time = robot.getTime()

        self.past_x_global = x_global
        self.past_y_global = y_global




if __name__ == '__main__':
    #robot = Robot()
    #drones = [DroneController(robot, 'Crazyflie'), DroneController(robot, 'Crazyflie(1)')]  # list of drones
    
    desired_position = [[3 , 0],
                        [0 , 3]]


    #print("number of robots = ", robot.getNumberOfDevices())
    #print("robot_name = ", robot.getName())

    supervisor = Supervisor()

    # get root node of the scene tree
    root = supervisor.getRoot()

    # get the 'children' field of the root node
    children_field = root.getField('children')

    for i in range(2):  # N is the number of Crazyflie robots you want to add
        # create a new VRML description of the Crazyflie robot
        new_robot_description = f"""
        Crazyflie {{
        translation {i * 2} 0 0  # for example, set x-coordinate to 'i * 2' to avoid overlap
        rotation 0 0 1 0
        name "crazyflie{i}"  # each robot needs a unique name
        controller "crazyflie_controller"  # replace with the name of your controller
        }}
        """
    
        # add the new robot to the 'children' field of the root node
        children_field.importMFNodeFromString(-1, new_robot_description)


    # # get the 'children' field of the root node
    # children_field = root_node.getField("children")

    # # create a new node using a VRML string
    # new_node = supervisor.getFromDef("Crazyflie")

    # # add the new node to the 'children' field of the root node
    # children_field.importMFNode(-1, new_node)

    # print(new_node.getDef())

    # while robot.step(drones[0].timestep) != -1:
    #     i = 0
    #     for drone in drones:
    #         drone.step(desired_position[i])
    #         i += 1