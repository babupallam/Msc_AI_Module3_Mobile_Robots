import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays

def stop_on_object_detection():
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
    
    # Test 5: Verify sensor handle retrieval
    if sensor_retrieval_code == sim.simx_return_ok:
        print("Sensor handle retrieved successfully.")
    else:
        print("Failed to retrieve sensor handle.")
        return  # Exit if the handle cannot be retrieved

    # Step 4: Start streaming data from the proximity sensor
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for the streaming to start

    # Step 5: Set the robot to move forward initially
    move_speed = 10  # Speed of the robot in meters per second
    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
    print("Robot moving forward...")

    # Step 6: Continuously check the sensor to stop on object detection
    detection_distance_threshold = 1.0  # Distance threshold in meters to stop the robot

    while True:
        # Step 6.1: Read sensor data
        return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)

        if return_code == sim.simx_return_ok:
            if detection_state:
                # Calculate distance to the detected object
                distance = (detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)**0.5
                print(f"Object detected at distance: {distance:.2f} meters.")

                # Step 6.2: Stop the robot if within detection distance threshold
                if distance < detection_distance_threshold:
                    print("Object within threshold distance. Stopping the robot.")  # Test 6: Check stopping
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                    break  # Stop further movement since object has been detected and robot is stopped
            else:
                print("No object detected.")
        
        time.sleep(0.1)  # Delay for smooth sensor updates

    # Step 7: Close the connection to CoppeliaSim
    sim.simxFinish(client_id)
    print("Disconnected from CoppeliaSim.")

# Run the stop on object detection function
if __name__ == "__main__":
    stop_on_object_detection()
