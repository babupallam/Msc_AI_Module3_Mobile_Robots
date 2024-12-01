import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import cv2  # Import OpenCV for image processing
import numpy as np  # Import numpy for handling image data

def vision_based_object_recognition():
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
    
    # Step 3: Get handle for the robot's camera (Vision sensor)
    _, camera_handle = sim.simxGetObjectHandle(client_id, 'Vision_sensor', sim.simx_opmode_blocking)
    sim.simxGetVisionSensorImage(client_id, camera_handle, 0, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for the camera feed to start streaming

    # Step 4: Navigation parameters
    move_speed = 1.0  # Speed for moving forward
    turn_speed = 0.5  # Speed for turning

    try:
        while True:
            # Step 5: Capture image from camera
            return_code, resolution, image = sim.simxGetVisionSensorImage(client_id, camera_handle, 0, sim.simx_opmode_buffer)
            if return_code == sim.simx_return_ok:
                # Convert image to a numpy array
                image = np.array(image, dtype=np.uint8)
                image = image.reshape((resolution[1], resolution[0], 3))
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert from RGB to BGR

                # Step 6: Process image to detect objects
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                _, thresh_image = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                object_detected = False
                obstacle_type = None

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 500:  # Filter out small contours
                        # Find bounding box
                        x, y, w, h = cv2.boundingRect(contour)

                        # Identify the type of object based on size or shape (as an example)
                        if w > 50 and h > 50:  # Example threshold for a human-like object
                            obstacle_type = "human"
                            object_detected = True
                            cv2.putText(image, "Human Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        else:
                            obstacle_type = "wall"
                            object_detected = True
                            cv2.putText(image, "Wall Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        # Draw the bounding box for visualization
                        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # Step 7: Make navigation decisions based on detected objects
                if object_detected:
                    if obstacle_type == "human":
                        # Stop the robot if a human is detected
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                        print("Human detected! Stopping the robot.")  # Test 40
                    elif obstacle_type == "wall":
                        # Turn to avoid the wall
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -turn_speed, sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, turn_speed, sim.simx_opmode_blocking)
                        print("Wall detected! Turning to avoid.")  # Test 40
                else:
                    # Move forward if no obstacles are detected
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
                    print("No obstacles detected. Moving forward.")  # Test 39

                # Display the processed image (optional)
                cv2.imshow("Camera View", image)
                cv2.waitKey(1)

            time.sleep(0.1)  # Delay to update camera feed and movement

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 8: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        cv2.destroyAllWindows()
        print("Disconnected from CoppeliaSim.")

# Run the vision-based object recognition function
if __name__ == "__main__":
    vision_based_object_recognition()
