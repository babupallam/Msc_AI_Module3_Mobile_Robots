# -*- coding: utf-8 -*-
"""
Created on 30th November 2020

@author: Robert B.
"""

"""
Make sure to have the server side running in V-REP by having the following command executed just once, at simulation start:

simRemoteApi.start(19999)

then start simulation, and run this program.

IMPORTANT: for each successful call to simxStart, there should be a corresponding call to simxFinish at the end!
"""

try:
    import sim as vrep  # import as vrep for older versions;
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import time
import math
import sys

class Robot():

    def __init__(self):
        # Setup Motors
        res, self.leftMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
        res, self.rightMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)

        # Setup Sonars
        res, self.frontLeftSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor6', vrep.simx_opmode_blocking)
        res, self.frontRightSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', vrep.simx_opmode_blocking)

        # Start Sonars
        vrep.simxReadProximitySensor(clientID, self.frontLeftSonar, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(clientID, self.frontRightSonar, vrep.simx_opmode_streaming)
        time.sleep(2)

    def getDistanceReading(self, objectHandle):
        res, detectionState, detectedPoint, _, _ = vrep.simxReadProximitySensor(clientID, objectHandle, vrep.simx_opmode_buffer)
        if detectionState == 1:
            return math.sqrt(sum(i ** 2 for i in detectedPoint))
        return 9999

    def stop(self):
        vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, vrep.simx_opmode_blocking)

    def move(self, velocity):
        vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, velocity, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, velocity, vrep.simx_opmode_blocking)

    def turnArc(self, leftMotorVelocity, rightMotorVelocity):
        vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, leftMotorVelocity, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, rightMotorVelocity, vrep.simx_opmode_blocking)


print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to simulator

if clientID != -1:
    print('Connected to remote API server')

    robot = Robot()


    # Variables initialization;
    """
    Original Code
        # PID Controller Constants
        Kp = 0.5   # Proportional gain
        Kd = 0.01   # Derivative gain
        Ki = 0.01  # Integral gain
        dt = 0.1   # Time step
    """
    Kp = 0.001  # Proportional gain;
    Kd = 0.0002  # Derivative gain;
    Ki = 0.0  # Integral gain
    dt = 0.2  # Time constant;

    # PID State Variables
    previous_error = 0
    integral = 0
    desired_distance = 0.5  # Desired distance from the wall (meters)

    while True:
        # Read distance from front left and front right sonar sensors
        front_left_dist = robot.getDistanceReading(robot.frontLeftSonar)
        front_right_dist = robot.getDistanceReading(robot.frontRightSonar)

        # Calculate average distance and error
        current_distance = (front_left_dist + front_right_dist) / 2
        error = desired_distance - current_distance

        # Calculate integral and derivative terms
        integral += error * dt
        derivative = (error - previous_error) / dt

        # Compute PID output
        control_signal = Kp * error + Ki * integral + Kd * derivative

        # Adjust robot motion based on control signal
        left_motor_velocity = 1 - control_signal
        right_motor_velocity = 1 + control_signal

        robot.turnArc(left_motor_velocity, right_motor_velocity)

        # Print debug information
        print(f"Error: {error:.4f}, Control Signal: {control_signal:.4f}, Dist: {current_distance:.4f}")

        # Update previous error
        previous_error = error

        time.sleep(dt)

    robot.stop()
    vrep.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
sys.exit('Could not connect')
