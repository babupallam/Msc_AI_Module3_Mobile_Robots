import sim  # Import the CoppeliaSim remote API
import time  # Import time library for delays
import math  # Import math for calculations
import socket  # Import socket for TCP/IP communication
import threading  # Import threading for handling multiple connections

# Define communication parameters
HOST = '127.0.0.1'
PORT = 65432

# Dictionary to store obstacle data shared by other robots
shared_obstacle_data = {}

def handle_incoming_communications(server_socket):
    """Handle incoming connections from other robots."""
    while True:
        conn, addr = server_socket.accept()
        with conn:
            data = conn.recv(1024).decode('utf-8')
            if data:
                # Update the shared obstacle data with received information
                robot_id, obstacle_info = data.split(":")
                shared_obstacle_data[robot_id] = obstacle_info
                print(f"Received obstacle data from {robot_id}: {obstacle_info}")

def send_data_to_other_robots(message):
    """Send obstacle data to other robots."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(message.encode('utf-8'))
    except Exception as e:
        print(f"Error sending data: {e}")

def swarm_navigation(robot_id):
    # Step 1: Establish a connection to CoppeliaSim
    print(f"Robot {robot_id} connecting to CoppeliaSim...")
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    
    if client_id != -1:
        print(f"Robot {robot_id} connected to CoppeliaSim successfully!")  # Connection successful
    else:
        print(f"Robot {robot_id} failed to connect to CoppeliaSim.")
        return  # Exit if the connection fails

    # Step 2: Get motor handles for the Pioneer P3-DX robot
    _, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
    
    # Step 3: Get handle for the proximity sensor
    _, front_sensor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)
    sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.5)  # Allow some time for sensor data to start streaming

    # Step 4: Navigation parameters
    move_speed = 1.0  # Speed for moving forward
    turn_speed = 0.5  # Speed for turning
    obstacle_safety_distance = 0.5  # Safety distance for obstacle avoidance

    try:
        while True:
            # Step 5: Read sensor data
            return_code, detection_state, detected_point, _, _ = sim.simxReadProximitySensor(client_id, front_sensor_handle, sim.simx_opmode_buffer)
            obstacle_detected = False
            obstacle_distance = float('inf')

            if return_code == sim.simx_return_ok and detection_state:
                obstacle_distance = math.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
                if obstacle_distance < obstacle_safety_distance:
                    obstacle_detected = True
                    print(f"Robot {robot_id} detected obstacle at distance: {obstacle_distance:.2f} meters.")
                    # Step 5.1: Share obstacle data with other robots
                    message = f"{robot_id}:{obstacle_distance}"
                    send_data_to_other_robots(message)

            # Step 6: Use shared obstacle data for navigation decision-making
            for other_robot_id, shared_distance in shared_obstacle_data.items():
                shared_distance = float(shared_distance)
                if shared_distance < obstacle_safety_distance:
                    print(f"Robot {robot_id} is aware of obstacle detected by {other_robot_id}.")
                    obstacle_detected = True

            # Step 7: Make navigation decisions based on obstacle detection
            if obstacle_detected:
                # If an obstacle is detected, turn to avoid it
                sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -turn_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, right_motor_handle, turn_speed, sim.simx_opmode_blocking)
                print(f"Robot {robot_id} is turning to avoid obstacle.")  # Test 36
            else:
                # If no obstacles are detected, move forward
                sim.simxSetJointTargetVelocity(client_id, left_motor_handle, move_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client_id, right_motor_handle, move_speed, sim.simx_opmode_blocking)
                print(f"Robot {robot_id} is moving forward.")  # Test 35

            time.sleep(0.1)  # Delay to update sensor readings and movement

    except KeyboardInterrupt:
        # Allow for clean exit when user interrupts with Ctrl+C
        print(f"Robot {robot_id} stopping...")

    finally:
        # Step 8: Stop the robot and close the connection to CoppeliaSim
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_blocking)
        sim.simxFinish(client_id)
        print(f"Robot {robot_id} disconnected from CoppeliaSim.")

if __name__ == "__main__":
    # Step 9: Start a server socket to listen for incoming data from other robots
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen()

    # Step 10: Run the server in a separate thread to handle incoming communications
    communication_thread = threading.Thread(target=handle_incoming_communications, args=(server_socket,))
    communication_thread.daemon = True
    communication_thread.start()

    # Start navigation for a specific robot (e.g., Robot A)
    swarm_navigation("Robot_A")
