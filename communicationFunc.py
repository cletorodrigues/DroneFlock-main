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
from controller import Camera
from controller import DistanceSensor

from math import cos, sin
import numpy as np


import sys
sys.path.append('../../../controllers/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1



def send_message(my_id, message):
    message_to_send = f"{my_id}:{message}"
    emitter.send(message_to_send.encode('utf-8'))

def receive_message():
    received_message = receiver.getString()
    sender_name, message_content = received_message.split(":")

    sender_id = int(sender_name)
            
    # Removing parenthesis and splitting the string into list
    message_content = message_content.strip('()').split(',')

    # Converting each element in the list to float
    message_content = [float(i) for i in message_content]

    return sender_id, message_content



if __name__ == '__main__':

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
    camera = robot.getDevice("camera")
    camera.enable(timestep)
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
    print(robot_name)	

    # Get emitter
    emitter = robot.getDevice("emitter")
    
    #Get receiver
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

    ## Initialize variables

    past_x_global = gps.getValues()[0]
    past_y_global = gps.getValues()[1]
    past_time = robot.getTime()

    # Crazyflie velocity PID controller
    PID_CF = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()

    height_desired = FLYING_ATTITUDE
    
    pos = np.zeros((8, 3))

    my_id = int(robot.getName())
    input_read = 0

    #define the goal distance
    threshold_distance = 3
    equi_dist = 2

    # Main loop:
    while robot.step(timestep) != -1:
        input_read = 0 

        dt = robot.getTime() - past_time
        actual_state = {}
    
        
        print(dt)

        ## Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        altitude = gps.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global != past_x_global)*(x_global - past_x_global)/dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global != past_y_global)*(y_global - past_y_global)/dt

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


        height_desired += height_diff_desired * dt

        pos[my_id] = x_global, y_global, altitude
        message = x_global, y_global, altitude
        send_message(my_id, message)

        
        while receiver.getQueueLength() > 0 or input_read == 0:
            sender_id, message_content = receive_message()
            
            pos[sender_id] = message_content
            receiver.nextPacket() # move to the next message in the queue

            if sender_id == 7:
                input_read = 1
                

        


        ## Example how to get sensor data
        ## range_front_value = range_front.getValue();
        ## cameraData = camera.getImage()


        ## PID velocity controller with fixed height
        motor_power = PID_CF.pid(dt, forward_desired, sideways_desired,
                                yaw_desired, height_desired,
                                roll, pitch, yaw_rate,
                                altitude, v_x, v_y)

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global