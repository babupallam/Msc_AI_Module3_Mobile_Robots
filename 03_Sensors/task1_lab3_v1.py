import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays

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
        time.sleep(1)  # Wait to ensure the simulation is stopped

    # Start simulation
    print("Starting simulation...")
    sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)
    time.sleep(1)  # Allow time for the simulation to initialize

    # Step 3: Get the robot handle and motor handles
    _, robot_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx', sim.simx_opmode_blocking)
    _, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

    # Check if handles are retrieved successfully
    if robot_handle == -1 or left_motor_handle == -1 or right_motor_handle == -1:
        print("Error: Could not retrieve all required handles. Exiting...")
        sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)  # Stop simulation before exiting
        sim.simxFinish(client_id)
        return
    else:
        print("Robot and motor handles retrieved successfully.")

    # Step 4: Define movement functions for motor control
    def move_motors(left_speed, right_speed, duration):
        """Control both motors at given speeds for a specific duration."""
        print(f"Setting left motor to {left_speed} and right motor to {right_speed} for {duration} seconds.")
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, left_speed, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, right_speed, sim.simx_opmode_blocking)
        total_time = 0
        while total_time < duration:
            time.sleep(0.1)
            total_time += 0.1

    def stop_motors():
        """Stop both motors."""
        print("Stopping motors.")
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)

    def turn_left_90():
        """Turn left approximately 90 degrees."""
        print("Turning left 90 degrees.")
        # Tune the speed and duration for an approximate 90-degree turn
        turn_speed = 0.5
        turn_duration = 1.35  # Adjust this value for a more accurate 90-degree turn
        move_motors(-turn_speed, turn_speed, turn_duration)

    def turn_left_180():
        """Turn left approximately 180 degrees."""
        print("Turning left 180 degrees.")
        # Tune the speed and duration for an approximate 180-degree turn
        turn_speed = 0.5
        turn_duration = 2.9  # Adjust this value for a more accurate 180-degree turn
        move_motors(-turn_speed, turn_speed, turn_duration)

    # Step 5: Move along the given path using explicit function calls
    try:
        speed = 1.0  # Set speed to an appropriate value

        # Movement sequence without loop
        print("Starting movement pattern.")

        # Step 1: Move forward by 1 meter
        move_motors(speed, speed, 1.5)  # Adjust duration based on the desired distance

        # Step 2: Turn left (90 degrees)
        turn_left_90()

        # Step 3: Move forward by 3 meters
        move_motors(speed, speed, 4.5)  # Adjust duration based on the desired distance

        # Step 4: Turn 180 degrees before moving backward
        turn_left_180()

        # Step 5: Move backward by 3 meters (using the same move_motors but robot is now facing backward)
        move_motors(speed, speed, 4.5)  # Adjust duration based on the desired distance

        # Step 6: Turn left again (90 degrees)
        turn_left_90()

        # Step 7: Move forward by 1 meter (second iteration of path without loop)
        move_motors(speed, speed, 1.5)

        # Step 8: Turn left (90 degrees)
        turn_left_90()

        # Step 9: Move forward by 3 meters
        move_motors(speed, speed, 4.5)

        # Step 10: Turn 180 degrees before moving backward
        turn_left_180()

        # Step 11: Move backward by 3 meters
        move_motors(speed, speed, 4.5)

        # Step 12: Turn left again (90 degrees)
        turn_left_90()

        # Step 13: Move forward by 1 meter (third iteration of path without loop)
        move_motors(speed, speed, 1.5)

        # Step 14: Turn left (90 degrees)
        turn_left_90()

        # Step 15: Move forward by 3 meters
        move_motors(speed, speed, 4.5)

        # Step 16: Turn 180 degrees before moving backward
        turn_left_180()

        # Step 17: Move backward by 3 meters
        move_motors(speed, speed, 4.5)

        # Step 18: Turn left again (90 degrees)
        turn_left_90()

        # Stop the motors after completing the movement pattern
        stop_motors()

    except KeyboardInterrupt:
        print("Movement interrupted by user. Stopping motors.")
        stop_motors()

    finally:
        # Step 6: Stop the simulation and close the connection
        print("Stopping simulation.")
        sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
        time.sleep(1)  # Allow time for the simulation to stop
        print("Disconnecting from CoppeliaSim.")
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the program
if __name__ == "__main__":
    follow_path()
