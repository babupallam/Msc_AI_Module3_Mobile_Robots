import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import math  # Import math for calculations
import heapq  # Import heapq for priority queue used in A* algorithm

class AStarPlanner:
    def __init__(self, grid_size=100, resolution=0.1):
        """Initialize the A* planner."""
        self.grid_size = grid_size
        self.resolution = resolution
        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

    def set_obstacle(self, x, y):
        """Mark grid cell as an obstacle."""
        grid_x = int(x / self.resolution) + self.grid_size // 2
        grid_y = int(y / self.resolution) + self.grid_size // 2
        if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
            self.grid[grid_x][grid_y] = 1

    def heuristic(self, start, goal):
        """Heuristic for A* (Euclidean distance)."""
        return math.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)

    def get_neighbors(self, node):
        """Get neighbors of the current node."""
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        neighbors = []
        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            if 0 <= neighbor[0] < self.grid_size and 0 <= neighbor[1] < self.grid_size and self.grid[neighbor[0]][neighbor[1]] == 0:
                neighbors.append(neighbor)
        return neighbors

    def plan_path(self, start, goal):
        """Plan a path from start to goal using A*."""
        start_grid = (int(start[0] / self.resolution) + self.grid_size // 2,
                      int(start[1] / self.resolution) + self.grid_size // 2)
        goal_grid = (int(goal[0] / self.resolution) + self.grid_size // 2,
                     int(goal[1] / self.resolution) + self.grid_size // 2)

        open_list = []
        heapq.heappush(open_list, (0, start_grid))
        came_from = {}
        cost_so_far = {start_grid: 0}

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal_grid:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return [(node[0] - self.grid_size // 2) * self.resolution,
                        (node[1] - self.grid_size // 2) * self.resolution] for node in path]

            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current

        return []  # Return empty path if no path found

def dynamic_replanning_navigation():
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

    # Step 4: Get handle for proximity sensors
    _, front_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for sensor data to start streaming

    # Step 5: Define target goal and planner
    goal_position = [5.0, 5.0]  # Target goal
    planner = AStarPlanner()

    # Step 6: Navigation parameters
    move_speed = 1.0
    turn_speed = 0.5
    obstacle_safety_distance = 0.5

    try:
        while True:
            # Step 7: Get current robot position
            _, robot_position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
            robot_x, robot_y = robot_position[0], robot_position[1]

            # Step 8: Read sensor data
            return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)

            # If an obstacle is detected, update the map and replan the path
            if return_code == sim.simx_return_ok and detection_state:
                obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                if obstacle_distance < obstacle_safety_distance:
                    obstacle_x = robot_x + obstacle_distance * math.cos(0)  # Assuming front direction for simplicity
                    obstacle_y = robot_y + obstacle_distance * math.sin(0)
                    planner.set_obstacle(obstacle_x, obstacle_y)
                    print("Obstacle detected. Replanning path...")
                    path = planner.plan_path((robot_x, robot_y), goal_position)
                else:
                    # If no obstacles detected, continue on current path
                    path = planner.plan_path((robot_x, robot_y), goal_position)

            if not path:
                print("No valid path to the goal.")
                break

            # Step 9: Follow the planned path
            for waypoint in path:
                goal_x, goal_y = waypoint
                while True:
                    # Get the current position of the robot
                    _, robot_position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
                    robot_x, robot_y = robot_position[0], robot_position[1]

                    # Calculate the distance to the goal
                    distance_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

                    if distance_to_goal < 0.3:
                        print(f"Reached waypoint at X={goal_x}, Y={goal_y}")
                        break

                    # Calculate direction to the goal
                    goal_angle = math.atan2(goal_y - robot_y, goal_x - robot_x)
                    _, robot_orientation = sim.simxGetObjectOrientation(client_id, robot_handle, -1, sim.simx_opmode_buffer)
                    robot_angle = robot_orientation[2]
                    angle_diff = goal_angle - robot_angle

                    # Normalize angle difference to the range [-pi, pi]
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi

                    # Adjust orientation if needed
                    if abs(angle_diff) > 0.1:
                        turn_direction = 1 if angle_diff > 0 else -1
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, turn_direction * turn_speed, sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -turn_direction * turn_speed, sim.simx_opmode_blocking)
                    else:
                        # Move forward if aligned
                        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)

                    time.sleep(0.1)

    except KeyboardInterrupt:
        print("User interrupted. Stopping the robot...")

    finally:
        # Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

if __name__ == "__main__":
    dynamic_replanning_navigation()
