# Make sure to have the server side running in CoppeliaSim (V-REP):
# in a child script of the scene, add the following command at simulation start:
#
#   simRemoteApi.start(19999)
#
# then start the simulation, and run this program.
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
        res, self.leftMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                                       vrep.simx_opmode_blocking)
        res, self.rightMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                                        vrep.simx_opmode_blocking)

        # Setup Sonars
        # Note: The sensor numbering might differ depending on your scene setup.
        # Make sure these names match the ones in your scene.
        res, self.frontSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5',
                                                        vrep.simx_opmode_blocking)
        res, self.leftSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1',
                                                       vrep.simx_opmode_blocking)
        res, self.rightSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor8',
                                                        vrep.simx_opmode_blocking)

        # Start streaming sonar data
        vrep.simxReadProximitySensor(clientID, self.frontSonar, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(clientID, self.leftSonar, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(clientID, self.rightSonar, vrep.simx_opmode_streaming)

        # Let the streaming data initialize
        time.sleep(2)

    def getDistanceReading(self, objectHandle):
        # Return magnitude of detected point if sensor sees something,
        # or a large number (9999) if nothing is detected
        res, detectionState, detectedPoint, _, _ = vrep.simxReadProximitySensor(clientID,
                                                                                objectHandle,
                                                                                vrep.simx_opmode_buffer)
        if detectionState == 1:
            return math.sqrt(sum(i**2 for i in detectedPoint))
        else:
            return 9999

    def stop(self):
        vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, vrep.simx_opmode_blocking)

    def turn(self, turnVelocity):
        # turnVelocity < 0 => turn left
        # turnVelocity > 0 => turn right
        vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, turnVelocity, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, -turnVelocity, vrep.simx_opmode_blocking)

    def move(self, velocity):
        # velocity < 0 => reverse
        # velocity > 0 => forward
        vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, velocity, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, velocity, vrep.simx_opmode_blocking)

    def turnArc(self, leftMotorVelocity, rightMotorVelocity):
        vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, leftMotorVelocity, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, rightMotorVelocity, vrep.simx_opmode_blocking)

print ('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print ('Connected to remote API server')

    # Create Robot instance
    robot = Robot()

    # -------------------------------
    # PID Controller Setup
    # -------------------------------
    # Gains for PID
    Kp = 0.001   # Proportional gain
    Ki = 0.0   # Integral gain
    Kd = 0.0002   # Derivative gain

    # For storing errors over time
    error_integral = 0.0
    last_error = 0.0
    last_time = time.time()

    # Target distance from left wall (tune as needed)
    target_distance = 0.5

    # Movement parameters
    base_speed = 3     # forward speed
    max_turn = 0.3        # clamp for steering correction

    # Example sensor checks (just 5 reads of front sonar)
    for _ in range(5):
        print("Front sonar dist:", robot.getDistanceReading(robot.frontSonar))
        time.sleep(0.2)

    print("Starting PID wall-following... Press Ctrl+C to stop.")
    try:
        while True:
            # Step 1: Get current time and compute dt
            current_time = time.time()
            dt = current_time - last_time
            if dt <= 0:
                dt = 0.01  # safety to avoid division by zero

            # Step 2: Read sensors
            front_dist = robot.getDistanceReading(robot.frontSonar)
            left_dist = robot.getDistanceReading(robot.leftSonar)

            # Step 3: Obstacle avoidance (simple check front)
            if front_dist < 0.3:
                # If front obstacle is detected, turn in place briefly
                print("Front obstacle detected. Turning right in place.")
                robot.turn(0.3)  # turn right
                time.sleep(0.5)
                continue

            # Step 4: Compute PID error => difference from target
            error = target_distance - left_dist

            # Step 5: Accumulate integral
            error_integral += error * dt

            # Step 6: Derivative
            error_derivative = (error - last_error) / dt

            # Step 7: PID output
            output = (Kp * error) + (Ki * error_integral) + (Kd * error_derivative)

            # Optional clamp to avoid excessive turning
            if output > max_turn:
                output = max_turn
            if output < -max_turn:
                output = -max_turn

            # Step 8: Set motor speeds
            # If output is positive => we want to turn right slightly
            # (meaning leftDist is bigger, so we get closer to wall)
            # Conversely, if output is negative => turn left
            left_speed = base_speed - output
            right_speed = base_speed + output
            robot.turnArc(left_speed, right_speed)

            # Step 9: Save current error/time for next iteration
            last_error = error
            last_time = current_time

            # small delay to allow consistent loop timing
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping robot...")

    # Stopping the robot
    robot.stop()

    # Ensure the last command arrives
    vrep.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')

print ('Program ended')
