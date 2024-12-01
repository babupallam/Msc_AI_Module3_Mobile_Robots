import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import math  # Import math for distance calculation
import random  # Import random for exploration during learning

# Q-learning parameters
ALPHA = 0.1     # Learning rate
GAMMA = 0.9     # Discount factor
EPSILON = 0.2   # Exploration rate
ACTIONS = ["move_forward", "turn_left", "turn_right"]

# Initialize Q-table (state-action pair values)
Q_TABLE = {}

def get_state(obstacle_distance):
    """ Get the state based on obstacle distance. """
    if obstacle_distance < 0.3:
        return "very_close"
    elif 0.3 <= obstacle_distance < 0.7:
        return "close"
    else:
        return "far"

def choose_action(state):
    """ Choose action based on the epsilon-greedy strategy. """
    if state not in Q_TABLE:
        Q_TABLE[state] = {action: 0 for action in ACTIONS}
    if random.uniform(0, 1) < EPSILON:
        # Explore: choose a random action
        return random.choice(ACTIONS)
    else:
        # Exploit: choose the action with the highest Q-value
        return max(Q_TABLE[state], key=Q_TABLE[state].get)

def update_q_value(state, action, reward, next_state):
    """ Update Q-table with new information. """
    if next_state not in Q_TABLE:
        Q_TABLE[next_state] = {action: 0 for action in ACTIONS}
    max_future_q = max(Q_TABLE[next_state].values())
    current_q = Q_TABLE[state][action]
    Q_TABLE[state][action] = (1 - ALPHA) * current_q + ALPHA * (reward + GAMMA * max_future_q)

def reinforcement_learning_navigation():
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
    
    # Step 3: Get handle for the proximity sensor
    _, front_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for streaming to start

    # Step 4: Navigation parameters
    move_speed = 1.0  # Speed for moving forward
    turn_speed = 0.5  # Speed for turning
    exploration_episodes = 100  # Number of exploration episodes to train the robot

    # Step 5: Reinforcement learning loop
    try:
        for episode in range(exploration_episodes):
            print(f"Starting episode {episode + 1}/{exploration_episodes}")
            state = "far"  # Initialize state as "far"

            while True:
                # Step 5.1: Read sensor data
                return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)
                obstacle_distance = float('inf')  # Default to infinity if no obstacle is detected

                if return_code == sim.simx_return_ok and detection_state:
                    obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)

                # Get current state
                state = get_state(obstacle_distance)

                # Choose action using epsilon-greedy strategy
                action = choose_action(state)

                # Execute the chosen action
                if action == "move_forward":
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
                elif action == "turn_left":
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -turn_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, turn_speed, sim.simx_opmode_blocking)
                elif action == "turn_right":
                    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, turn_speed, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, -turn_speed, sim.simx_opmode_blocking)

                # Step 5.2: Reward or penalize the action taken
                reward = 0
                if action == "move_forward" and state == "far":
                    reward = 1  # Reward for moving forward without obstacles
                elif action == "move_forward" and state in ["close", "very_close"]:
                    reward = -1  # Penalize for moving forward when too close
                elif action in ["turn_left", "turn_right"] and state == "very_close":
                    reward = 1  # Reward for turning when an obstacle is very close

                # Step 5.3: Get next state
                time.sleep(0.1)  # Delay to allow movement
                return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)
                next_state = get_state(float('inf'))
                if return_code == sim.simx_return_ok and detection_state:
                    obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                    next_state = get_state(obstacle_distance)

                # Step 5.4: Update Q-value based on the action taken
                update_q_value(state, action, reward, next_state)

                # Stop if a collision is likely
                if state == "very_close":
                    print("Too close to an obstacle. Episode ends.")
                    break

            # Stop motors at the end of each episode
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print("User interrupted. Stopping the robot...")

    finally:
        # Step 6: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print("Disconnected from CoppeliaSim.")

# Run the reinforcement learning navigation function
if __name__ == "__main__":
    reinforcement_learning_navigation()
