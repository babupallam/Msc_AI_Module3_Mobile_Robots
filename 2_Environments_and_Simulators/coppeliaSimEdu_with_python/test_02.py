import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays

def proximity_sensor_integration():
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
    # simxGetObjectHandle() retrieves a handle that can be used to interact with the sensor
    # 'Pioneer_p3dx_ultrasonicSensor5' is the sensor name in the CoppeliaSim scene
    sensor_retrieval_code, front_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)
    
    # Test 3: Verify sensor handle retrieval
    if sensor_retrieval_code == sim.simx_return_ok:
        print("Sensor handle retrieved successfully.")
    else:
        print("Failed to retrieve sensor handle.")
        return  # Exit if the handle cannot be retrieved

    # Step 4: Start streaming data from the proximity sensor
    # simxReadProximitySensor() starts streaming data for the given sensor
    # We use simx_opmode_streaming to indicate that we want continuous data
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for the streaming to start

    # Step 5: Move the robot forward while reading sensor data
    # Set the initial velocity of the motors to move forward
    move_speed = 5  # Speed of the robot in meters per second
    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)

    # Step 6: Read and print proximity sensor values for a defined period
    duration = 5  # Duration for which we read the sensor values (seconds)
    print("Reading sensor values for {} seconds...".format(duration))

    start_time = time.time()
    while (time.time() - start_time) < duration:
        # Step 6.1: Read sensor data
        # simx_opmode_buffer is used after streaming has started to get the latest value
        return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)

        # Test 4: Confirm that sensor data is being streamed and read
        if return_code == sim.simx_return_ok:
            if detection_state:
                distance = (detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)**0.5
                print(f"Object detected at distance: {distance:.2f} meters.")
            else:
                print("No object detected.")
        else:
            print("Failed to read sensor data.")

        time.sleep(0.2)  # Delay to prevent overloading the connection with requests

    # Step 7: Stop the robot after the reading period
    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
    print("Robot stopped.")

    # Step 8: Close the connection to CoppeliaSim
    sim.simxFinish(client_id)
    print("Disconnected from CoppeliaSim.")

# Run the proximity sensor integration function
if __name__ == "__main__":
    proximity_sensor_integration()
