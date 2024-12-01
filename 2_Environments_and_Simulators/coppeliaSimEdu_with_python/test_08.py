import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays

def speed_control_based_on_proximity():
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
    
    # Test 15: Verify sensor handle retrieval
    if sensor_retrieval_code == sim.simx_return_ok:
        print("Sensor handle retrieved successfully.")
    else:
        print("Failed to retrieve sensor handle.")
        return  # Exit if the handle cannot be retrieved

    # Step 4: Start streaming data from the proximity sensor
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for streaming to start

    # Step 5: Define motion parameters
    max_speed = 10               # Maximum speed of the robot in meters per second
    min_speed = 0.2               # Minimum speed for approaching an obstacle in meters per second
    safety_distance = 0.5         # Safety distance in meters to stop the robot
    slow_down_distance = 1.5      # Distance in meters to start slowing down

    # Step 6: Continuous speed control loop based on proximity
    try:
        while True:
            # Step 6.1: Read sensor data
            return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)

            if return_code == sim.simx_return_ok and detection_state:
                # Calculate distance to the detected object
                distance = (detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)**0.5
                print(f"Object detected at distance: {distance:.2f} meters.")

                # Step 6.2: Adjust speed based on distance
                if distance < safety_distance:
                    # Stop the robot if too close to the obstacle
                    print("Object too close. Stopping the robot.")  # Test 16: Smooth stop
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                elif distance < slow_down_distance:
                    # Reduce speed proportionally as distance decreases
                    reduced_speed = max(min_speed, max_speed * (distance / slow_down_distance))
                    print(f"Reducing speed to {reduced_speed:.2f} m/s due to approaching obstacle.")  # Test 15: Speed reduction
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, reduced_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, reduced_speed, sim.simx_opmode_blocking)
                else:
                    # Set maximum speed if no obstacles are close
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, max_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, max_speed, sim.simx_opmode_blocking)
                    print("No nearby obstacles. Moving at max speed.")

            else:
                # No object detected, move at maximum speed
                sim.simxSetJointTargetVelocity(client_id, left_motor_handle, max_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, right_motor_handle, max_speed, sim.simx_opmode_blocking)
                print("No object detected. Moving forward.")

            time.sleep(0.1)  # Delay for sensor updates and control adjustments

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 7: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the speed control based on proximity function
if __name__ == "__main__":
    speed_control_based_on_proximity()
