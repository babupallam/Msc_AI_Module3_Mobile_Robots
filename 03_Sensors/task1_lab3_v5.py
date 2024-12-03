# Import necessary libraries
import sim  # Import the CoppeliaSim remote API to control the simulator
import time  # Import time library for delays
import math  # Import math library for calculations

# Define some constants for easy adjustment
WHEEL_SEPARATION = 0.381  # Wheel separation of Pioneer P3DX in meters
YAW_TOLERANCE = 0.02  # Tolerance for reaching the desired yaw, in radians
STREAMING_DELAY = 0.1  # Delay for starting streaming, in seconds
CONTROL_LOOP_DELAY = 0.01  # Small delay for control loop, in seconds
DEFAULT_SPEED = 1.0  # Default movement speed, in m/s
TURN_SPEED = 0.5  # Default turning speed, in rad/s

# Function to make the robot follow a specific path and draw it using dummy objects
def follow_path():
    # Step 1: Establish a connection to CoppeliaSim
    client_id = connect_to_coppeliasim()
    if client_id == -1:
        return

    # Step 2: Get handles for robot and motors
    robot_handle, left_motor_handle, right_motor_handle = get_robot_handles(client_id)
    if robot_handle == -1 or left_motor_handle == -1 or right_motor_handle == -1:
        disconnect_from_coppeliasim(client_id)
        return

    # Step 3: Define movement control functions
    def set_motor_speeds(left_speed, right_speed):
        """Set the speeds for both left and right motors."""
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, left_speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, right_speed, sim.simx_opmode_oneshot)

    def stop_motors():
        """Stop both motors."""
        set_motor_speeds(0, 0)

    def move_distance(speed, distance):
        """Move a specific distance at the given speed."""
        duration = distance / abs(speed)
        set_motor_speeds(speed, speed)
        timed_loop(duration, stop_motors, client_id, robot_handle)

    def turn_robot(angle):
        """Turn the robot by a specific angle (in degrees)."""
        sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_streaming)
        time.sleep(STREAMING_DELAY)

        _, initial_orientation = sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_buffer)
        target_yaw = normalize_yaw(initial_orientation[2] + math.radians(angle))

        # Set turning speeds for left and right motors
        left_speed = -TURN_SPEED if angle > 0 else TURN_SPEED
        right_speed = TURN_SPEED if angle > 0 else -TURN_SPEED
        set_motor_speeds(left_speed, right_speed)

        # Continuously adjust until reaching the target yaw
        while True:
            _, current_orientation = sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_buffer)
            if abs(target_yaw - current_orientation[2]) < YAW_TOLERANCE:
                break
            time.sleep(CONTROL_LOOP_DELAY)

        # Stop the motors after turning
        stop_motors()

    # Step 4: Move along the given path using loops for repeated pattern
    try:
        for i in range(3):
            print(f"Starting iteration {i + 1} of the movement pattern.")
            move_distance(DEFAULT_SPEED, 1.0)  # Move forward by 1 meter
            turn_robot(90)  # Turn left 90 degrees
            move_distance(DEFAULT_SPEED, 3.0)  # Move forward by 3 meters
            turn_robot(180)  # Turn left 180 degrees
            move_distance(DEFAULT_SPEED, 3.0)  # Move backward by 3 meters
            turn_robot(90)  # Turn left 90 degrees
        stop_motors()

    except KeyboardInterrupt:
        print("Movement interrupted by user. Stopping motors.")
        stop_motors()

    finally:
        # Step 5: Stop the simulation and close the connection
        disconnect_from_coppeliasim(client_id)


# Helper Functions
def connect_to_coppeliasim():
    """Connect to CoppeliaSim and return the client ID."""
    print("Connecting to CoppeliaSim...")
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:
        print("Connected to CoppeliaSim successfully!")
        sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)
    else:
        print("Failed to connect to CoppeliaSim.")
    return client_id

def disconnect_from_coppeliasim(client_id):
    """Stop the simulation and disconnect from CoppeliaSim."""
    print("Stopping simulation.")
    sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
    time.sleep(1)
    print("Disconnecting from CoppeliaSim.")
    sim.simxFinish(client_id)
    print("Disconnected from CoppeliaSim.")

def get_robot_handles(client_id):
    """Retrieve handles for the robot, left motor, and right motor."""
    _, robot_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx', sim.simx_opmode_blocking)
    _, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
    return robot_handle, left_motor_handle, right_motor_handle

def timed_loop(duration, end_function, client_id, robot_handle):
    """Execute a loop for a specific duration, adding markers along the way."""
    start_time = time.time()
    while time.time() - start_time < duration:
        add_path_marker(client_id, robot_handle)
        time.sleep(0.1)
    end_function()

def add_path_marker(client_id, robot_handle):
    """Add a dummy object at the current robot position to visualize the path."""
    _, position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_blocking)
    _, dummy_handle = sim.simxCreateDummy(client_id, 0.05, None, sim.simx_opmode_blocking)
    if dummy_handle != -1:
        sim.simxSetObjectPosition(client_id, dummy_handle, -1, position, sim.simx_opmode_blocking)

def normalize_yaw(yaw):
    """Normalize yaw to be within -pi to pi."""
    if yaw > math.pi:
        yaw -= 2 * math.pi
    elif yaw < -math.pi:
        yaw += 2 * math.pi
    return yaw


# Run the program
if __name__ == "__main__":
    follow_path()
