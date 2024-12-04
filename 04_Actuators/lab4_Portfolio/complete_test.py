# Step 1: Initialize the Pioneer P3DX robot and connect to CoppeliaSim
import sim  # Import the CoppeliaSim Python API
import time  # For controlling loop timings
import math  # For calculations

# Step 2: Establish connection to CoppeliaSim
print("Connecting to CoppeliaSim...")
sim.simxFinish(-1)  # Close all previous connections
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print("Connected to CoppeliaSim!")
else:
    print("Failed to connect to CoppeliaSim. Please check the connection.")
    exit()

# Step 3: Start simulation in a reset state
sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)  # Ensure simulation is stopped
time.sleep(1)  # Wait to ensure the simulation stops
sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)  # Start the simulation

# Step 4: Get handles for left and right motors, and ultrasonic sensors
_, left_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
_, right_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

# Handles for ultrasonic sensors (left: 1 and 16, right: 8 and 9)
_, left_sensor_1 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', sim.simx_opmode_blocking)
_, left_sensor_16 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor16', sim.simx_opmode_blocking)
_, right_sensor_8 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor8', sim.simx_opmode_blocking)
_, right_sensor_9 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor9', sim.simx_opmode_blocking)

# Step 5: Initialize sensor readings (start streaming)
sensors = [left_sensor_1, left_sensor_16, right_sensor_8, right_sensor_9]
for sensor in sensors:
    sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_streaming)

time.sleep(0.5)  # Allow some time for the sensors to start streaming data


# Step 6: Define utility functions for sensor readings and motor control
def get_sensor_distance(sensor_handle):
    """
    Fetch the distance reading from a specific ultrasonic sensor.
    Returns the distance if detected, else a high value (e.g., 1.0m for no detection).
    """
    _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, sensor_handle,
                                                                         sim.simx_opmode_buffer)
    if detectionState:
        return math.sqrt(detectedPoint[0] ** 2 + detectedPoint[1] ** 2 + detectedPoint[2] ** 2)
    else:
        return 1.0  # Default high value when no object is detected


def set_motor_speed(left_speed, right_speed):
    """
    Set the speed of the robot's motors.
    """
    sim.simxSetJointTargetVelocity(clientID, left_motor, left_speed, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_motor, right_speed, sim.simx_opmode_streaming)


# Step 7: Wandering logic
def wander_until_wall_detected():
    """
    Move the robot around randomly until a wall is detected by its sensors.
    """
    print("Wandering around...")
    wall_detected = False
    while not wall_detected:
        # Move forward
        set_motor_speed(2.0, 2.0)

        # Check distances from all sensors
        left_distance_1 = get_sensor_distance(left_sensor_1)
        left_distance_16 = get_sensor_distance(left_sensor_16)
        right_distance_8 = get_sensor_distance(right_sensor_8)
        right_distance_9 = get_sensor_distance(right_sensor_9)

        # Check if any wall is detected
        if (left_distance_1 < 0.5 or left_distance_16 < 0.5 or
                right_distance_8 < 0.5 or right_distance_9 < 0.5):
            wall_detected = True
            print("Wall detected! Switching to alignment.")
        time.sleep(0.1)  # Small delay to prevent excessive looping

    # Stop motors once a wall is detected
    set_motor_speed(0, 0)


# Step 8: Alignment logic and stopping after alignment
def align_with_wall():
    """
    Rotate the robot to ensure sensors are parallel to the wall.
    After alignment, stop completely before starting wall-following.
    """
    print("Starting alignment...")
    aligned = False
    timeout = time.time() + 10  # Timeout after 10 seconds to prevent indefinite loops
    side_aligned = None  # Track which side is aligned

    while not aligned and time.time() < timeout:
        # Get distances from sensors
        left_distance_1 = get_sensor_distance(left_sensor_1)
        left_distance_16 = get_sensor_distance(left_sensor_16)
        right_distance_8 = get_sensor_distance(right_sensor_8)
        right_distance_9 = get_sensor_distance(right_sensor_9)

        # Determine which side to align with
        if left_distance_1 < 0.5 and left_distance_16 < 0.5:
            # Align with the left side
            if abs(left_distance_1 - left_distance_16) > 0.02:
                if left_distance_1 > left_distance_16:
                    set_motor_speed(-0.1, 0.1)  # Rotate clockwise
                else:
                    set_motor_speed(0.1, -0.1)  # Rotate counter-clockwise
            else:
                aligned = True
                side_aligned = "left"
                print("Left-side alignment achieved!")
        elif right_distance_8 < 0.5 and right_distance_9 < 0.5:
            # Align with the right side
            if abs(right_distance_8 - right_distance_9) > 0.02:
                if right_distance_8 > right_distance_9:
                    set_motor_speed(-0.1, 0.1)  # Rotate clockwise
                else:
                    set_motor_speed(0.1, -0.1)  # Rotate counter-clockwise
            else:
                aligned = True
                side_aligned = "right"
                print("Right-side alignment achieved!")
        else:
            # No wall detected, rotate in place to search for wall
            set_motor_speed(0.1, -0.1)  # Rotate in place to search for wall

        time.sleep(0.1)

    # Stop motors after alignment to ensure stability
    set_motor_speed(0, 0)
    if aligned:
        print("Alignment completed. Stopping movement to prepare for wall-following.")

    return side_aligned


# Step 9: Wall-following logic with sensors from aligned side
def follow_wall(side_aligned):
    """
    Follow the wall on the aligned side while maintaining a constant distance.
    """
    print("Starting wall-following...")
    target_distance = 0.5  # Desired distance from the wall

    while True:
        if side_aligned == "left":
            print("Following wall on the left side.")
            # Use left-side sensors for wall-following
            left_distance_1 = get_sensor_distance(left_sensor_1)
            left_distance_16 = get_sensor_distance(left_sensor_16)

            if left_distance_1 < 1.0 and left_distance_16 < 1.0:
                # Adjust motor speeds based on individual sensor distances
                if left_distance_1 < target_distance:
                    # If too close to the wall, slow down left motor to steer away
                    set_motor_speed(0.5, 0.2)
                    print("Adjusting: Left too close, slowing left motor.")
                elif left_distance_16 < target_distance:
                    # If too close to the wall, slow down left motor to steer away
                    set_motor_speed(0.2, 0.5)
                    print("Adjusting: Left too close, slowing left motor.")
                elif left_distance_1 > target_distance:
                    # If too far from the wall, speed up left motor to steer closer
                    set_motor_speed(0.2, 0.5)
                    print("Adjusting: Left too far, speeding left motor.")
                elif left_distance_16 > target_distance:
                    # If too far from the wall, speed up left motor to steer closer
                    set_motor_speed(0.5, 0.2)
                    print("Adjusting: Left too far, speeding left motor.")
                else:
                    # If at the desired distance, move forward
                    set_motor_speed(0.5, 0.5)
                    print("Maintaining: Correct distance, moving forward.")

            else:
                # No wall detected, stop motors
                set_motor_speed(0, 0)
                print("No wall detected on the left side. Stopping wall-following.")
                break

        elif side_aligned == "right":
            print("Following wall on the right side.")
            # Use right-side sensors for wall-following
            right_distance_8 = get_sensor_distance(right_sensor_8)
            right_distance_9 = get_sensor_distance(right_sensor_9)

            if right_distance_8 < 1.0 and right_distance_9 < 1.0:
                # Adjust motor speeds based on individual sensor distances
                if right_distance_8 < target_distance:
                    # If too close to the wall, slow down right motor to steer away
                    set_motor_speed(0.5, 0.2)
                    print("Adjusting: Right too close, slowing right motor.")
                elif right_distance_9 < target_distance:
                    # If too close to the wall, slow down right motor to steer away
                    set_motor_speed(0.2, 0.5)
                    print("Adjusting: Right too close, slowing right motor.")
                elif right_distance_8 > target_distance:
                    # If too far from the wall, speed up right motor to steer closer
                    set_motor_speed(0.2, 0.5)
                    print("Adjusting: Right too far, speeding right motor.")
                elif right_distance_9 > target_distance:
                    # If too far from the wall, speed up right motor to steer closer
                    set_motor_speed(0.5, 0.2)
                    print("Adjusting: Right too far, speeding right motor.")
                else:
                    # If at the desired distance, move forward
                    set_motor_speed(0.5, 0.5)
                    print("Maintaining: Correct distance, moving forward.")

            else:
                # No wall detected, stop motors
                set_motor_speed(0, 0)
                print("No wall detected on the right side. Stopping wall-following.")
                break

        time.sleep(0.1)


# Step 10: Execute wandering, alignment, and wall-following tasks
try:
    wander_until_wall_detected()  # Wander until a wall is detected
    aligned_side = align_with_wall()  # Align the robot with the wall and stop
    if aligned_side:
        follow_wall(aligned_side)  # Start following the wall after stopping
except KeyboardInterrupt:
    print("Stopping robot...")
    set_motor_speed(0, 0)
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)  # Stop the simulation on keyboard interrupt
    time.sleep(1)  # Wait to ensure the simulation stops
finally:
    sim.simxFinish(clientID)
    print("Disconnected from CoppeliaSim.")
