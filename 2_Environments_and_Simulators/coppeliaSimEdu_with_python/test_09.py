import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import random  # Import random for random obstacle avoidance

def path_planning_and_memory():
    # Step 1: Establish a connection to CoppeliaSim
    print("Connecting to CoppeliaSim...")
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    
    if client_id != -1:
        print("Connected to CoppeliaSim successfully!")  # Connection successful
    else:
        print("Failed to connect to CoppeliaSim.")
        return  # Exit if the connection fails

    # Step 2: Get motor handles for the Pioneer P3-DX robot
    _, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

    # Step 3: Get the handle for the proximity sensor
    sensor_retrieval_code, front_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)
    
    # Test 17: Verify sensor handle retrieval
    if sensor_retrieval_code == sim.simx_return_ok:
        print("Sensor handle retrieved successfully.")
    else:
        print("Failed to retrieve sensor handle.")
        return  # Exit if the handle cannot be retrieved

    # Step 4: Start streaming data from the proximity sensor
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for streaming to start

    # Step 5: Define motion parameters
    move_speed = 1.0           # Speed of the robot in meters per second for forward movement
    turn_speed = 0.5           # Speed of the robot during turning
    safety_distance = 0.5      # Safety distance in meters to stop the robot
    turn_min_duration = 1      # Minimum duration of turn in seconds
    turn_max_duration = 3      # Maximum duration of turn in seconds
    memory_length = 3          # Number of recent moves to remember

    # Step 6: Initialize memory for path planning
    memory = []  # Keeps track of recent directions (-1 for left, 1 for right)

    # Step 7: Continuous reactive behavior loop with path planning
    try:
        while True:
            # Move the robot forward initially or resume after a turn
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
            print("Robot moving forward...")

            while True:
                # Step 7.1: Continuously read sensor data
                return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)

                if return_code == sim.simx_return_ok and detection_state:
                    # Calculate distance to the detected object
                    distance = (detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)**0.5
                    print(f"Object detected at distance: {distance:.2f} meters.")

                    # Step 7.2: Stop the robot if within safety distance threshold
                    if distance < safety_distance:
                        print("Object within safety distance. Stopping the robot to avoid collision.")
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                        break  # Exit the inner loop to execute a turn

                time.sleep(0.1)  # Delay for smooth sensor updates

            # Step 7.3: Use memory to decide which direction to turn
            # Check memory to avoid repeating recently taken directions
            recent_turns = set(memory[-memory_length:])
            if recent_turns == {-1}:
                # If the recent memory only contains left turns, prefer turning right
                turn_direction = 1
                print("Avoiding repeated left turns, turning right instead.")
            elif recent_turns == {1}:
                # If the recent memory only contains right turns, prefer turning left
                turn_direction = -1
                print("Avoiding repeated right turns, turning left instead.")
            else:
                # Otherwise, choose randomly
                turn_direction = random.choice([-1, 1])
                print(f"Choosing a random direction: {'left' if turn_direction == -1 else 'right'}.")  # Test 18: Random turn based on memory

            # Add the chosen direction to memory
            memory.append(turn_direction)

            # Step 7.4: Execute the turn
            turn_duration = random.uniform(turn_min_duration, turn_max_duration)  # Random duration between min and max
            print(f"Turning for {turn_duration:.2f} seconds in direction: {'left' if turn_direction == -1 else 'right'}.")

            # Set velocities for turning
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, turn_direction * turn_speed, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -turn_direction * turn_speed, sim.simx_opmode_blocking)
            time.sleep(turn_duration)  # Wait for the turn to complete

            # Stop turning
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
            print("Resuming forward movement...")  # Test 18: Resume motion after turn

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 8: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the path planning and memory function
if __name__ == "__main__":
    path_planning_and_memory()
