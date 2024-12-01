import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import math  # Import math for distance and angle calculations
from itertools import permutations  # Import permutations to find optimal path

def calculate_distance(point1, point2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def find_optimal_path(goals, start):
    """Find the optimal path using a simplified TSP solver."""
    min_distance = float('inf')
    best_path = None

    # Check all permutations to find the shortest path
    for perm in permutations(goals):
        current_distance = 0
        current_point = start

        # Calculate the total distance for this permutation
        for goal in perm:
            current_distance += calculate_distance(current_point, goal)
            current_point = goal

        # Update the best path if this one is shorter
        if current_distance < min_distance:
            min_distance = current_distance
            best_path = perm

    return best_path

def multiple_goal_navigation():
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

    # Step 4: Define multiple goals (waypoints) in the environment
    waypoints = [
        [2.0, 2.0],   # Goal 1
        [-2.0, 3.0],  # Goal 2
        [1.5, -1.5],  # Goal 3
        [0.0, -3.0]   # Goal 4
    ]

    # Step 5: Start streaming data to retrieve robot's position
    sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for streaming to start

    # Step 6: Navigation parameters
    move_speed = 1.0          # Speed for moving forward
    turn_speed = 0.5          # Speed for turning
    threshold_distance = 0.3  # Distance threshold to consider a waypoint reached

    # Step 7: Find the optimal path to visit each goal
    _, robot_position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
    start_position = [robot_position[0], robot_position[1]]
    optimal_path = find_optimal_path(waypoints, start_position)

    print(f"Optimal path calculated: {optimal_path}")  # Test 32

    # Step 8: Navigate through each goal in the optimal order
    try:
        for waypoint in optimal_path:
            goal_x, goal_y = waypoint
            print(f"Navigating to goal at X={goal_x}, Y={goal_y}...")  # Test 31

            while True:
                # Step 8.1: Get current position and orientation of the robot
                _, robot_position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
                _, robot_orientation = sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_buffer)

                robot_x, robot_y = robot_position[0], robot_position[1]
                robot_angle = robot_orientation[2]

                # Step 8.2: Calculate distance to the current goal
                distance_to_goal = calculate_distance((robot_x, robot_y), (goal_x, goal_y))
                print(f"Distance to goal: {distance_to_goal:.2f} meters.")

                # Step 8.3: Check if the goal is reached
                if distance_to_goal < threshold_distance:
                    print(f"Reached goal at X={goal_x}, Y={goal_y}.")
                    break

                # Step 8.4: Calculate direction to the goal
                goal_angle = math.atan2(goal_y - robot_y, goal_x - robot_x)
                angle_diff = goal_angle - robot_angle

                # Normalize angle difference to the range [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                # Step 8.5: Adjust orientation if needed
                if abs(angle_diff) > 0.1:
                    # If the robot is not facing the goal, turn
                    turn_direction = 1 if angle_diff > 0 else -1
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, turn_direction * turn_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -turn_direction * turn_speed, sim.simx_opmode_blocking)
                else:
                    # If the robot is facing the goal, move forward
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)

                time.sleep(0.1)

            # Stop the robot after reaching each goal
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

# Run the multiple goal navigation function
if __name__ == "__main__":
    multiple_goal_navigation()
