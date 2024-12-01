import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import random  # Import random for random obstacle avoidance

def adaptive_obstacle_avoidance():
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
    max_speed = 1.0             # Maximum speed in open environments
    reduced_speed = 0.5         # Reduced speed in cluttered environments
    turn_speed = 0.5            # Speed of the robot during turning
    safety_distance = 0.5       # Safety distance in meters to stop the robot
    turn_short_duration = 1     # Shorter turn duration in cluttered environments
    turn_long_duration = 3      # Longer turn duration in open environments
    cluttered_threshold = 3     # Number of obstacles detected within a short time to classify as cluttered

    # Step 6: Continuous adaptive behavior loop
    cluttered_count = 0  # Counter for detecting the cluttered environment

    try:
        while True:
            # Move the robot forward initially or resume after a turn
            current_speed = max_speed if cluttered_count < cluttered_threshold else reduced_speed
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, current_speed, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, current_speed, sim.simx_opmode_blocking)
            print(f"Robot moving forward at speed {current_speed:.2f} m/s...")

            while True:
                # Step 6.1: Continuously read data from all sensors
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

                # Step 6.2: Stop the robot if any sensor detects an obstacle within safety distance
                if obstacle_detected:
                    print("Stopping the robot to avoid collision.")
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                    cluttered_count += 1  # Increase clutter count if obstacles are detected frequently
                    break  # Exit the inner loop to execute a turn

                cluttered_count = max(0, cluttered_count - 1)  # Decrease clutter count over time if no obstacles are detected
                time.sleep(0.1)  # Delay for smooth sensor updates

            # Step 6.3: Execute a turn
            turn_direction = random.choice([-1, 1])  # Randomly choose between -1 (left) and 1 (right)
            turn_duration = turn_short_duration if cluttered_count >= cluttered_threshold else turn_long_duration
            print(f"Turning for {turn_duration:.2f} seconds in direction: {'left' if turn_direction == -1 else 'right'}.")

            # Set velocities for turning
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, turn_direction * turn_speed, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -turn_direction * turn_speed, sim.simx_opmode_blocking)
            time.sleep(turn_duration)  # Wait for the turn to complete

            # Stop turning
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
            print("Resuming forward movement...")  # Test 22: Confirm dynamic switch in behavior

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 7: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the adaptive obstacle avoidance function
if __name__ == "__main__":
    adaptive_obstacle_avoidance()
