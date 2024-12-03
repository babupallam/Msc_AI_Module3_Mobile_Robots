# Import necessary libraries
import sim  # Import the CoppeliaSim remote API to control the simulator
import time  # Import time library for delays
import math  # Import math library for calculations


# Function to make the robot follow a specific path and draw it using dummy objects
def follow_path():
    # Step 1: Establish a connection to CoppeliaSim
    print("Connecting to CoppeliaSim...")
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if client_id != -1:
        print("Connected to CoppeliaSim successfully!")
    else:
        print("Failed to connect to CoppeliaSim.")
        return

    # Step 2: Ensure simulation is running and stop any previous instance
    return_code, sim_state = sim.simxGetInMessageInfo(client_id, sim.simx_headeroffset_server_state)
    if return_code == sim.simx_return_ok and sim_state & 1 != 0:
        print("Simulation already running. Stopping current simulation.")
        sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
        time.sleep(1)

    # Start simulation
    print("Starting simulation...")
    sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)
    time.sleep(1)

    # Step 3: Get the robot handle and motor handles
    _, robot_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx', sim.simx_opmode_blocking)
    _, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

    if robot_handle == -1 or left_motor_handle == -1 or right_motor_handle == -1:
        print("Error: Could not retrieve all required handles. Exiting...")
        sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        return
    else:
        print("Robot and motor handles retrieved successfully.")

    # Step 4: Define movement functions for motor control
    def move_motors(left_speed, right_speed, distance):
        """Control both motors to move a given distance."""
        # Calculate duration using distance and speed
        if left_speed == right_speed:
            duration = distance / abs(left_speed)  # Time = Distance / Speed
        else:
            duration = 0  # Duration is zero if we are not moving forward/backward directly

        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, left_speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, right_speed, sim.simx_opmode_oneshot)

        # Start tracking the robot's position
        sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_streaming)
        time.sleep(0.1)

        # Loop until the duration has elapsed
        start_time = time.time()
        while time.time() - start_time < duration:
            _, current_position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
            add_path_marker(client_id, current_position)  # Add marker to visualize the path
            time.sleep(0.05)  # Reduce delay for more frequent marker placement

        # Stop the motors after moving the required distance
        stop_motors()

    def stop_motors():
        """Stop both motors."""
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_oneshot)

    def turn_robot(angle):
        """Turn the robot by a specific angle using built-in set orientation function."""
        # Start streaming the current orientation of the robot
        sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_streaming)
        time.sleep(0.1)

        # Get the current orientation of the robot
        _, initial_orientation = sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_buffer)
        initial_yaw = initial_orientation[2]  # Get the current yaw (z-axis rotation)

        # Calculate the target yaw and normalize it to be within -pi to pi
        target_yaw = normalize_yaw(initial_yaw + math.radians(angle))

        # Set the target orientation with the updated yaw
        target_orientation = [initial_orientation[0], initial_orientation[1], target_yaw]
        sim.simxSetObjectOrientation(client_id, robot_handle, -1, target_orientation, sim.simx_opmode_blocking)

        # Wait for a short period to ensure the orientation is updated
        time.sleep(1)

    # Step 5: Function to add a marker at the current robot position
    def add_path_marker(client_id, position):
        """Add a dummy object at the given position to visualize the path."""
        _, dummy_handle = sim.simxCreateDummy(client_id, 0.05, None, sim.simx_opmode_blocking)
        if dummy_handle != -1:
            sim.simxSetObjectPosition(client_id, dummy_handle, -1, position, sim.simx_opmode_blocking)

    # Step 6: Move along the given path using loops for repeated pattern
    try:
        speed = 1  # Set speed to a lower value for better precision

        for i in range(3):
            print(f"Starting iteration {i + 1} of the movement pattern.")

            # Move forward by 1 meter
            move_motors(speed, speed, 1.0)  # Move forward by 1 meter

            # Turn left (90 degrees)
            turn_robot(90)  # Turn left 90 degrees

            # Move forward by 3 meters
            move_motors(speed, speed, 3.0)  # Move forward by 3 meters

            # Turn 180 degrees before moving backward
            turn_robot(180)  # Turn left 180 degrees

            # Move backward by 3 meters (using the same move_motors but robot is now facing backward)
            move_motors(speed, speed, 3.0)  # Move forward by 3 meters

            # Turn left again (90 degrees)
            turn_robot(90)  # Turn left 90 degrees

            # Stop the motors after completing the movement pattern
        stop_motors()

    except KeyboardInterrupt:
        print("Movement interrupted by user. Stopping motors.")
        stop_motors()

    finally:
        # Step 7: Stop the simulation and close the connection
        print("Stopping simulation.")
        sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
        time.sleep(1)
        print("Disconnecting from CoppeliaSim.")
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")


# Helper Function
def normalize_yaw(yaw):
    """Normalize yaw to be within -pi to pi."""
    if yaw > math.pi:
        yaw -= 2 * math.pi
    elif yaw < -math.pi:
        yaw += 2 * math.pi
    return yaw


# Run the program
if __name__ == "__main__":
    follow_path()
