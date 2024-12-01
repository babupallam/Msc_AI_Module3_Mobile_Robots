import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import math  # Import math for distance and angle calculations
import numpy as np  # Use numpy to help with map representation and handling

class SimpleGridMap:
    def __init__(self, size=100, resolution=0.1):
        """Initialize the grid map for SLAM."""
        self.size = size  # Size of the grid map (e.g., 100x100 cells)
        self.resolution = resolution  # Resolution in meters per cell
        self.grid = np.zeros((size, size), dtype=np.float32)  # Grid initialized to zero
        
    def update_map(self, robot_x, robot_y, obstacle_distance, sensor_angle):
        """Update the map based on sensor readings."""
        # Calculate obstacle position in grid coordinates
        obstacle_x = robot_x + obstacle_distance * math.cos(sensor_angle)
        obstacle_y = robot_y + obstacle_distance * math.sin(sensor_angle)

        # Convert to grid coordinates
        grid_x = int(obstacle_x / self.resolution) + self.size // 2
        grid_y = int(obstacle_y / self.resolution) + self.size // 2

        # Update the grid if the coordinates are within bounds
        if 0 <= grid_x < self.size and 0 <= grid_y < self.size:
            self.grid[grid_x][grid_y] = 1  # Mark the cell as an obstacle

    def display_map(self):
        """Display the map in the console (for simplicity)."""
        print(np.flipud(self.grid))

def slam_navigation():
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

    # Step 4: Get handles for multiple proximity sensors
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

    time.sleep(0.5)  # Allow some time for sensor data to start streaming

    # Step 5: Create an empty grid map for SLAM
    grid_map = SimpleGridMap(size=100, resolution=0.1)

    # Step 6: Navigation parameters
    move_speed = 1.0  # Speed for moving forward
    turn_speed = 0.5  # Speed for turning
    obstacle_safety_distance = 0.5  # Safety distance for obstacle avoidance

    try:
        while True:
            # Step 7: Get the robot's position and orientation
            _, robot_position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
            _, robot_orientation = sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_buffer)

            robot_x, robot_y = robot_position[0], robot_position[1]
            robot_angle = robot_orientation[2]  # Orientation in radians (about the z-axis)

            # Step 8: Update map with sensor data
            for sensor_name, sensor_handle in sensors.items():
                return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, sensor_handle, sim.simx_opmode_buffer)

                if return_code == sim.simx_return_ok and detection_state:
                    # Calculate distance to the detected object
                    obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                    # Calculate sensor's relative angle based on robot's orientation
                    sensor_angle_offset = {"Pioneer_p3dx_ultrasonicSensor1": -math.pi/4,
                                           "Pioneer_p3dx_ultrasonicSensor2": 0,
                                           "Pioneer_p3dx_ultrasonicSensor3": 0,
                                           "Pioneer_p3dx_ultrasonicSensor4": math.pi/4}
                    sensor_angle = robot_angle + sensor_angle_offset[sensor_name]
                    # Update the map
                    grid_map.update_map(robot_x, robot_y, obstacle_distance, sensor_angle)

            # Step 9: Obstacle avoidance and movement
            obstacle_nearby = False
            for sensor_name, sensor_handle in sensors.items():
                return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, sensor_handle, sim.simx_opmode_buffer)
                if return_code == sim.simx_return_ok and detection_state:
                    obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                    if obstacle_distance < obstacle_safety_distance:
                        obstacle_nearby = True
                        break

            if obstacle_nearby:
                # Turn to avoid obstacle
                sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -turn_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, right_motor_handle, turn_speed, sim.simx_opmode_blocking)
                print("Obstacle detected! Turning to avoid.")
            else:
                # Move forward
                sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
                print("Moving forward...")

            # Display the map for visualization purposes (could be replaced by a more sophisticated display)
            grid_map.display_map()

            time.sleep(0.1)  # Delay to update sensor readings and movement

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 10: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the SLAM navigation function
if __name__ == "__main__":
    slam_navigation()
