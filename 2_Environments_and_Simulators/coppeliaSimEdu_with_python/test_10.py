import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import random  # Import random for random obstacle avoidance

def advanced_obstacle_avoidance_navigation():
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

    # Step 3: Get handles for multiple ultrasonic sensors
    sensor_names = [
        'Pioneer_p3dx_ultrasonicSensor1',  # Front-left sensor
        'Pioneer_p3dx_ultrasonicSensor2',  # Front-center-left sensor
        'Pioneer_p3dx_ultrasonicSensor3',  # Front-center-right sensor
        'Pioneer_p3dx_ultrasonicSensor4',  # Front-right sensor
    ]

    sensors = {}
    for sensor_name in sensor_names:
        return_code, sensor_handle = sim.simxGetObjectHandle(client_id, sensor_name, sim.simx_opmode_blocking)
        if return_code == sim.simx_return_ok:
            sensors[sensor_name] = sensor_handle
            print(f"Sensor handle for {sensor_name} retrieved successfully.")
        else:
            print(f"Failed to retrieve sensor handle for {sensor_name}.")
            return  # Exit if any sensor handle cannot be retrieved

    # Step 4: Start streaming data from all sensors
    for sensor_name, sensor_handle in sensors.items():
        sim.simxReadProximitySensor(client_id, sensor_handle, sim.simx_opmode_streaming)
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

    # Step 7: Continuous reactive navigation loop
    try:
        while True:
            # Move the robot forward initially or resume after a turn
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
            print("Robot moving forward...")

            while True:
                # Step 7.1: Continuously read data from all sensors
                obstacle_detected = False
                detected_distances = {}

                for sensor_name, sensor_handle in sensors.items():
                    return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, sensor_handle, sim.simx_opmode_buffer)

                    if return_code == sim.simx_return_ok and detection_state:
                        # Calculate distance to the detected object
                        distance = (detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)**0.5
                        detected_distances[sensor_name] = distance
                        if distance < safety_distance:
                            print(f"Obstacle detected by {sensor_name} at distance: {distance:.2f} meters.")
                            obstacle_detected = True

                # Step 7.2: Stop the robot if any sensor detects an obstacle within safety distance
                if obstacle_detected:
                    print("Stopping the robot to avoid collision.")
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                    break  # Exit the inner loop to execute a turn

                time.sleep(0.1)  # Delay for smooth sensor updates

            # Step 7.3: Use memory and sensor data for intelligent navigation
            # Check memory to decide the direction of the turn
            recent_turns = set(memory[-memory_length:])
            if recent_turns == {-1}:
                turn_direction = 1
                print("Memory suggests avoiding repeated left turns, choosing right.")
            elif recent_turns == {1}:
                turn_direction = -1
                print("Memory suggests avoiding repeated right turns, choosing left.")
            else:
                # Use sensor data to make a more informed decision
                # If the front-left sensors have smaller distances, turn right, and vice versa
                left_distances = [detected_distances.get(sensor, float('inf')) for sensor in ['Pioneer_p3dx_ultrasonicSensor1', 'Pioneer_p3dx_ultrasonicSensor2']]
                right_distances = [detected_distances.get(sensor, float('inf')) for sensor in ['Pioneer_p3dx_ultrasonicSensor3', 'Pioneer_p3dx_ultrasonicSensor4']]

                if min(left_distances) < min(right_distances):
                    turn_direction = 1  # Turn right
                    print("Obstacle detected more on the left side, turning right.")
                elif min(right_distances) < min(left_distances):
                    turn_direction = -1  # Turn left
                    print("Obstacle detected more on the right side, turning left.")
                else:
                    turn_direction = random.choice([-1, 1])
                    print("Distances are similar, choosing a random direction.")

            # Add the chosen direction to memory
            memory.append(turn_direction)

            # Step 7.4: Execute the turn
            turn_duration = random.uniform(turn_min_duration, turn_max_duration)  # Random duration between min and max
            print(f"Turning for {turn_duration:.2f} seconds in direction: {'left' if turn_direction == -1 else 'right'}.")  # Test 20

            # Set velocities for turning
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, turn_direction * turn_speed, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -turn_direction * turn_speed, sim.simx_opmode_blocking)
            time.sleep(turn_duration)  # Wait for the turn to complete

            # Stop turning
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
            print("Resuming forward movement...")  # Test 20: Resume motion after turn

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 8: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the advanced obstacle avoidance and navigation function
if __name__ == "__main__":
    advanced_obstacle_avoidance_navigation()
