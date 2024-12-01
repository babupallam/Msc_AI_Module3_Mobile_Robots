import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays

def basic_connection_and_movement():
    # Step 1: Establish a connection to CoppeliaSim
    # Connect to CoppeliaSim on localhost (127.0.0.1) and port 19999
    # simxStart() returns a client ID which should be > -1 if the connection is successful
    print("Connecting to CoppeliaSim...")
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    
    if client_id != -1:
        print("Connected to CoppeliaSim successfully!")  # Test 1: Successful connection
    else:
        print("Failed to connect to CoppeliaSim.")
        return  # Exit if the connection fails
    
    # Step 2: Get handles for the motors
    # simxGetObjectHandle() retrieves an object handle that can be used to interact with the robot components
    # 'Pioneer_p3dx_leftMotor' and 'Pioneer_p3dx_rightMotor' are the names of the motors in the CoppeliaSim scene
    _, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

    # Step 3: Set the joint target velocities for the motors
    # simxSetJointTargetVelocity() sets the target velocity for the given motor
    # Here, we set both the left and right motors to move forward at 1 m/s
    move_speed = 1  # Speed of the robot in meters per second
    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)

    # Step 4: Move forward for a few seconds
    # Use time.sleep() to wait while the robot is moving
    time_duration = 5  # Duration in seconds for the robot to move forward
    print(f"Robot moving forward for {time_duration} seconds...")  # Test 2: Ensure robot moves
    time.sleep(time_duration)

    # Step 5: Stop the robot
    # Set the velocity of both motors to 0 to stop the robot
    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
    print("Robot stopped.")

    # Step 6: Close the connection to CoppeliaSim
    # simxFinish() closes the connection to CoppeliaSim
    sim.simxFinish(client_id)
    print("Disconnected from CoppeliaSim.")

# Run the basic connection and movement function
if __name__ == "__main__":
    basic_connection_and_movement()
