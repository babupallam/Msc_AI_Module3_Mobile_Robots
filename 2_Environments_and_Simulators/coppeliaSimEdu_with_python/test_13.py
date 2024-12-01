import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import math  # Import math for calculating distances and angles

def path_smoothing_navigation():
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
    
    # Step 3: Get handle for robot base
    _, robot_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx', sim.simx_opmode_blocking)

    # Step 4: Start streaming data to retrieve robot's position
    sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for streaming to start

    # Step 5: Navigation parameters
    move_speed = 1.0           # Normal speed in meters per second for forward movement
    turn_speed = 0.3           # Reduced speed during turning to make the movement smoother
    smooth_turn_factor = 0.1   # Gradual turning increment factor
    threshold_distance = 0.3   # Distance threshold to consider a waypoint reached
    obstacle_safety_distance = 0.5  # Safety distance to stop for obstacles

    # Step 6: Get handle for the proximity sensor
    _, front_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)

    # Step 7: Continuous path smoothing navigation
    try:
        while True:
            # Step 7.1: Read robot's current position and orientation
            _, robot_position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
            _, robot_orientation = sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_buffer)

            # Step 7.2: Obstacle detection
            return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)
            if return_code == sim.simx_return_ok and detection_state:
                # Calculate distance to the detected object
                obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                if obstacle_distance < obstacle_safety_distance:
                    print(f"Obstacle detected at distance: {obstacle_distance:.2f} meters. Adjusting path to avoid collision.")

                    # Gradually reduce the left motor speed to create a smooth curve-like turn
                    left_motor_velocity = move_speed
                    right_motor_velocity = move_speed - (smooth_turn_factor * move_speed)

                    # Set a reduced and differential speed to create a gradual curve
                    while obstacle_distance < obstacle_safety_distance:
                        left_motor_velocity = max(left_motor_velocity - smooth_turn_factor, turn_speed)
                        right_motor_velocity = max(right_motor_velocity, turn_speed * 0.5)

                        # Apply the new velocities to create a smooth turning motion
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, left_motor_velocity, sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, right_motor_velocity, sim.simx_opmode_blocking)

                        # Update sensor data
                        return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)
                        if detection_state:
                            obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                        else:
                            break  # No obstacle detected anymore

                        time.sleep(0.1)  # Delay for smooth sensor updates

                    # Resume normal speed after avoiding the obstacle
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
                    print("Resuming forward movement after avoiding obstacle.")  # Test 26

            else:
                # If no obstacle detected, keep moving forward at normal speed
                sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
                print("Moving forward smoothly...")  # Test 25

            time.sleep(0.1)  # Update frequency for smooth navigation

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 8: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the path smoothing navigation function
if __name__ == "__main__":
    path_smoothing_navigation()
