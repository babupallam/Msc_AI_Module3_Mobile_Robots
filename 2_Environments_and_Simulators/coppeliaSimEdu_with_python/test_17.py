import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import math  # Import math for calculations
import cv2   # Import OpenCV for image processing
import numpy as np  # Import numpy for handling image data

def real_time_obstacle_prediction():
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
    
    # Step 3: Get handle for the robot base
    _, robot_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx', sim.simx_opmode_blocking)

    # Step 4: Get handles for proximity sensors
    sensor_names = [
        'Pioneer_p3dx_ultrasonicSensor1', 
        'Pioneer_p3dx_ultrasonicSensor2',
        'Pioneer_p3dx_ultrasonicSensor3',
        'Pioneer_p3dx_ultrasonicSensor4',
    ]
    sensors = {}
    for sensor_name in sensor_names:
        return_code, sensor_handle = sim.simxGetObjectHandle(client_id, sensor_name, sim.simx_opmode_blocking)
        if return_code == sim.simx_return_ok:
            sensors[sensor_name] = sensor_handle
            sim.simxReadProximitySensor(client_id, sensor_handle, sim.simx_opmode_streaming)

    # Step 5: Get handle for the robot's camera (if available)
    _, camera_handle = sim.simxGetObjectHandle(client_id, 'Vision_sensor', sim.simx_opmode_blocking)
    sim.simxGetVisionSensorImage(client_id, camera_handle, 0, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for sensor and camera data to start streaming

    # Step 6: Navigation parameters
    move_speed = 1.0  # Speed for moving forward
    turn_speed = 0.5  # Speed for turning
    obstacle_safety_distance = 0.5  # Safety distance for obstacle avoidance

    try:
        while True:
            obstacle_detected = False
            obstacle_distances = []

            # Step 7: Read data from proximity sensors
            for sensor_name, sensor_handle in sensors.items():
                return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, sensor_handle, sim.simx_opmode_buffer)

                if return_code == sim.simx_return_ok and detection_state:
                    # Calculate distance to the detected object
                    obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                    obstacle_distances.append(obstacle_distance)

                    if obstacle_distance < obstacle_safety_distance:
                        obstacle_detected = True
                        print(f"Obstacle detected by {sensor_name} at distance: {obstacle_distance:.2f} meters.")

            # Step 8: Read and process camera data for obstacle prediction
            return_code, resolution, image = sim.simxGetVisionSensorImage(client_id, camera_handle, 0, sim.simx_opmode_buffer)
            if return_code == sim.simx_return_ok:
                # Convert the image to a numpy array and reshape it
                image = np.array(image, dtype=np.uint8)
                image = image.reshape((resolution[1], resolution[0], 3))
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert from RGB to BGR for OpenCV

                # Step 8.1: Use basic image processing to detect obstacles
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                _, thresh_image = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # If contours are found, assume potential obstacles are present
                if len(contours) > 0:
                    obstacle_detected = True
                    print("Potential obstacle detected by camera.")

                    # Draw contours for visualization (optional)
                    cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

                # Display the processed image (optional)
                cv2.imshow("Camera View", image)
                cv2.waitKey(1)

            # Step 9: Decision-making based on sensor fusion
            if obstacle_detected:
                # If an obstacle is detected, turn to avoid it
                sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -turn_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, right_motor_handle, turn_speed, sim.simx_opmode_blocking)
                print("Obstacle detected! Turning to avoid.")  # Test 34
            else:
                # If no obstacles are detected, move forward
                sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
                print("Moving forward...")  # Test 33

            time.sleep(0.1)  # Delay to update sensor readings and movement

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 10: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        cv2.destroyAllWindows()
        print("Disconnected from CoppeliaSim.")

# Run the real-time obstacle prediction function
if __name__ == "__main__":
    real_time_obstacle_prediction()
