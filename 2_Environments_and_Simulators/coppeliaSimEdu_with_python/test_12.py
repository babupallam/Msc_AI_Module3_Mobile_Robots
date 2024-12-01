import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import math  # Import math for calculating distances and angles

def dynamic_goal_navigation():
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
    
    # Step 3: Get handle for robot base
    _, robot_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx', sim.simx_opmode_blocking)

    # Step 4: Define waypoints (target points) in the environment
    waypoints = [
        [2.0, 2.0],  # Goal 1: X=2.0, Y=2.0
        [-2.0, 3.0], # Goal 2: X=-2.0, Y=3.0
        [1.5, -1.5]  # Goal 3: X=1.5, Y=-1.5
    ]

    # Step 5: Start streaming data to retrieve robot's position
    sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for streaming to start

    # Step 6: Navigation parameters
    move_speed = 1.0           # Speed of the robot in meters per second for forward movement
    turn_speed = 0.5           # Speed of the robot during turning
    threshold_distance = 0.3   # Distance threshold to consider a waypoint reached
    obstacle_safety_distance = 0.5  # Safety distance to stop for obstacles

    # Step 7: Get handle for the proximity sensor
    _, front_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)

    # Step 8: Navigate to each waypoint
    try:
        for waypoint in waypoints:
            goal_x, goal_y = waypoint
            print(f"Navigating to target at X={goal_x}, Y={goal_y}...")  # Test 23

            while True:
                # Step 8.1: Get current position and orientation of the robot
                _, robot_position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
                _, robot_orientation = sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_buffer)

                # Step 8.2: Calculate distance to target
                distance_to_goal = math.sqrt((goal_x - robot_position[0])**2 + (goal_y - robot_position[1])**2)
                print(f"Distance to target: {distance_to_goal:.2f} meters.")

                # Step 8.3: Check if goal is reached
                if distance_to_goal < threshold_distance:
                    print(f"Reached target at X={goal_x}, Y={goal_y}.")  # Test 23
                    break

                # Step 8.4: Calculate direction to target
                goal_angle = math.atan2(goal_y - robot_position[1], goal_x - robot_position[0])
                current_angle = robot_orientation[2]
                angle_diff = goal_angle - current_angle

                # Normalize angle difference to the range [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                # Step 8.5: Obstacle detection
                return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)
                if return_code == sim.simx_return_ok and detection_state:
                    # Calculate distance to the detected object
                    obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                    if obstacle_distance < obstacle_safety_distance:
                        print(f"Obstacle detected at distance: {obstacle_distance:.2f} meters. Stopping to avoid collision.")
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
                        time.sleep(0.5)
                        continue

                # Step 8.6: Adjust orientation if needed
                if abs(angle_diff) > 0.1:
                    # If the robot is not facing the target, turn
                    turn_direction = 1 if angle_diff > 0 else -1
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, turn_direction * turn_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -turn_direction * turn_speed, sim.simx_opmode_blocking)
                else:
                    # If the robot is facing the target, move forward
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)

                time.sleep(0.1)

            # Stop the robot after reaching the goal
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 9: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the dynamic goal navigation function
if __name__ == "__main__":
    dynamic_goal_navigation()
