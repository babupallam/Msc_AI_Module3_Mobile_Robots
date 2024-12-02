import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import random  # Import random for random obstacle avoidance


def simple_obstacle_avoidance():
    # Step 1: Establish a connection to CoppeliaSim
    print("Connecting to CoppeliaSim...")
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if client_id != -1:
        print("Connected to CoppeliaSim successfully!")
    else:
        print("Failed to connect to CoppeliaSim.")
        return

    # Step 2: Get motor handles
    _, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

    # Step 3: Get proximity sensor handle
    _, proximity_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5',
                                                         sim.simx_opmode_blocking)
    if proximity_sensor_handle == -1:
        print("Error: Could not retrieve sensor handle. Exiting...")
        return
    else:
        print("Proximity sensor handle retrieved successfully.")

    # Step 4: Start streaming sensor data
    sim.simxReadProximitySensor(client_id, proximity_sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow streaming to initialize

    # Step 5: Define parameters
    move_speed = 1.0  # Forward speed (m/s)
    turn_speed = 0.5  # Turning speed (m/s)
    detection_distance = 2.0  # X: Object detection distance (meters)
    stop_distance = 0.5  # Y: Stop distance (meters)
    turn_duration = 2  # Turning duration (seconds)

    # Step 6: Continuous navigation
    try:
        while True:
            # Move forward
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
            print("Robot moving forward...")

            # Check for obstacles
            while True:
                return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id,
                                                                                                 proximity_sensor_handle,
                                                                                                 sim.simx_opmode_buffer)
                if return_code == sim.simx_return_ok and detection_state:
                    # Calculate distance to the detected object
                    distance = (detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2) ** 0.5
                    if distance < detection_distance:
                        print(f"Object detected at {distance:.2f} meters.")
                        # Stop the robot if within stop distance
                        if distance < stop_distance:
                            print("Stopping robot to avoid collision.")
                            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                            break  # Exit the inner loop to execute a turn
                time.sleep(0.1)

            # Randomly turn left or right
            turn_direction = random.choice([-1, 1])  # -1 for left, 1 for right
            print(f"Turning {'left' if turn_direction == -1 else 'right'} for {turn_duration} seconds.")
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, turn_direction * turn_speed,
                                           sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -turn_direction * turn_speed,
                                           sim.simx_opmode_blocking)
            time.sleep(turn_duration)

            # Stop turning
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
            print("Resuming forward movement...")

    except KeyboardInterrupt:
        # Stop the robot on user interrupt
        print("Stopping robot and disconnecting...")
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)

    finally:
        # Close the connection
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")


# Run the program
if __name__ == "__main__":
    simple_obstacle_avoidance()
