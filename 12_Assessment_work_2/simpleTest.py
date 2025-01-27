# Make sure to have the server side running in V-REP (CoppeliaSim): 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim as vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math


class Robot():

    def __init__(self):

        # Setup Motors
        res, self.leftMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
        res, self.rightMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)

        # Setup Sonars
        res, self.frontSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking)
        res, self.leftSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1',vrep.simx_opmode_blocking)
        res, self.rightSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor8',vrep.simx_opmode_blocking)

        # Start Sonars
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,self.frontSonar,vrep.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,self.leftSonar,vrep.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,self.rightSonar,vrep.simx_opmode_streaming)

        time.sleep(2)


    def getDistanceReading(self, objectHandle):
        # Get reading from sensor
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,objectHandle,vrep.simx_opmode_buffer)

        if detectionState == 1:
            # return magnitude of detectedPoint
            return math.sqrt(sum(i**2 for i in detectedPoint))
        else:
            # return another value that we know cannot be true and handle it (use a large number so that if you do 'distance < reading' it will work)
            return 9999



    def stop(self):
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, vrep.simx_opmode_blocking)



    def turn(self, turnVelocity):
        # turnVelocity < 0 = turn left
        # turnVelocity > 0 = turn right
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, turnVelocity, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, -turnVelocity, vrep.simx_opmode_blocking)



    def move(self, velocity):
        # velocity < 0 = reverse
        # velocity > 0 = forward
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, velocity, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, velocity, vrep.simx_opmode_blocking)


    def turnArc(self, leftMotorVelocity, rightMotorVelocity):
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, leftMotorVelocity, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, rightMotorVelocity, vrep.simx_opmode_blocking)



print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')




    robot = Robot()

    for i in range(5):
        print(robot.getDistanceReading(robot.frontSonar))
        time.sleep(0.2)

    #code here

    robot.move(1)

    time.sleep(5)

    robot.turn(0.5)
    time.sleep(3)

    robot.turn(-0.5)
    time.sleep(3)

    robot.turnArc(0.1, 0.5)
    time.sleep(3)


    robot.stop()

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')












