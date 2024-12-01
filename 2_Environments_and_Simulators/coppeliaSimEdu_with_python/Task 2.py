import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import random  # Import random library for random direction


def object_avoidance_robot():
    # Step 1: Establish a connection to CoppeliaSim
    print("Connecting to CoppeliaSim...")
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if client_id != -1:
        print("Connected to CoppeliaSim successfully!")
    else:
        print("Failed to connect to CoppeliaSim.")
        return

    # Step 2: Get handles for the motors and proximity sensor
    _, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
    _, proximity_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5',
                                                         sim.simx_opmode_blocking)

    # Step 3: Initialize the proximity sensor in streaming mode
    sim.simxReadProximitySensor(client_id, proximity_sensor_handle, sim.simx_opmode_streaming)

    move_speed = 1  # Speed of the robot in meters per second
    detect_distance_x = 2.0  # Object detection distance (X meters)
    stop_distance_y = 0.5  # Stop distance (Y meters)

    try:
        while True:
            # Step 4: Move forward at constant speed
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)

            # Step 5: Check for objects within detection range
            detection_state, detected_point, _, _, _ = sim.simxReadProximitySensor(client_id, proximity_sensor_handle,
                                                                                   sim.simx_opmode_buffer)
            if detection_state and detected_point[2] < detect_distance_x:
                print(f"Object detected within {detect_distance_x} meters.")

                # Step 6: Stop if the object is closer than the stop distance
                if detected_point[2] < stop_distance_y:
                    print(f"Stopping! Object is less than {stop_distance_y} meters away.")
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                    time.sleep(1)  # Pause for a moment

                    # Step 7: Turn randomly in a certain direction
                    turn_direction = random.choice(['left', 'right'])
                    if turn_direction == 'left':
                        print("Turning left...")
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -move_speed,
                                                       sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed,
                                                       sim.simx_opmode_blocking)
                    else:
                        print("Turning right...")
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed,
                                                       sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -move_speed,
                                                       sim.simx_opmode_blocking)

                    time.sleep(2)  # Turn duration
            time.sleep(0.1)  # Add a small delay for better control

    except KeyboardInterrupt:
        # Stop the robot on program interruption
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        print("Program interrupted and robot stopped.")

    # Step 8: Close the connection to CoppeliaSim
    sim.simxFinish(client_id)
    print("Disconnected from CoppeliaSim.")


# Run the object avoidance robot program
if __name__ == "__main__":
    object_avoidance_robot()
