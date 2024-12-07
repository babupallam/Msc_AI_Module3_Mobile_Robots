import sim
import time
import math
import random
import sys
import matplotlib.pyplot as plt
from datetime import datetime

class PioneerRobot:
    def __init__(self, client_id):
        self.client_id = client_id

        # Fetch motor handles
        self.left_motor, self.right_motor = self.fetch_motor_handles()

        # Fetch sensor handles
        self.sensors = self.fetch_sensor_handles()

        # Start sensor data streaming
        self.start_sensor_stream()

        # Data for plotting
        self.movement_data = []

    def fetch_motor_handles(self):
        # Get handles for motors
        _, left_motor = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
        _, right_motor = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
        return left_motor, right_motor

    def fetch_sensor_handles(self):
        sensor_map = {
            'front_left': 'Pioneer_p3dx_ultrasonicSensor5',
            'front_right': 'Pioneer_p3dx_ultrasonicSensor2',
            'left_side': 'Pioneer_p3dx_ultrasonicSensor16', #back_left
            'right_side': 'Pioneer_p3dx_ultrasonicSensor8', #front_right
            'back_left': 'Pioneer_p3dx_ultrasonicSensor14',
            'back_right': 'Pioneer_p3dx_ultrasonicSensor9'
        }
        sensor_handles = {}
        for key, sensor in sensor_map.items():
            _, sensor_handle = sim.simxGetObjectHandle(self.client_id, sensor, sim.simx_opmode_blocking)
            sensor_handles[key] = sensor_handle
        return sensor_handles

    def start_sensor_stream(self):
        # Initiate data streaming for all sensors
        for sensor_handle in self.sensors.values():
            sim.simxReadProximitySensor(self.client_id, sensor_handle, sim.simx_opmode_streaming)
        time.sleep(2)  # Allow streaming time to initiate

    def get_sensor_distance(self, sensor_name):
        # Retrieve distance measurement from a specified sensor
        sensor_handle = self.sensors[sensor_name]
        _, detected, detected_point, _, _ = sim.simxReadProximitySensor(self.client_id, sensor_handle,
                                                                        sim.simx_opmode_buffer)
        if detected:
            return math.sqrt(sum(coord ** 2 for coord in detected_point))  # Calculate distance
        else:
            return float('inf')  # Large value if no object detected

    def halt(self):
        # Stop both motors
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, 0, sim.simx_opmode_blocking)

    def drive(self, speed):
        # Drive forward or backward
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, speed, sim.simx_opmode_streaming)

    def steer(self, left_speed, right_speed):
        # Set distinct velocities for each motor to steer
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, left_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, right_speed, sim.simx_opmode_streaming)

    def wander(self):
        # Random speed for moving forward or backward
        random_move_speed = random.uniform(-0.5, 4.0)

        # Random turn speed for each motor
        random_turn_speed = random.uniform(-1.0, 1.0)

        # Combine random movement and random turn for left and right motor speeds
        left_motor_speed = random_move_speed + random_turn_speed
        right_motor_speed = random_move_speed - random_turn_speed

        # Apply the speeds to the motors
        self.steer(left_motor_speed, right_motor_speed)
        time.sleep(0.3)
        console_log("Action Taken", {
            "Message": "Wandering with random movement and turn.",
            "Random Move Speed": random_move_speed,
            "Random Turn Speed": random_turn_speed,
            "Left Motor Speed": left_motor_speed,
            "Right Motor Speed": right_motor_speed
        })

    def wander_and_wall_follow_logic(self):
        # For wandering and wall-following
        try:
            while True:
                # Obtain sensor data
                front_left = self.get_sensor_distance('front_left')
                front_right = self.get_sensor_distance('front_right')
                left_side = self.get_sensor_distance('left_side')
                right_side = self.get_sensor_distance('right_side')
                back_left = self.get_sensor_distance('back_left')
                back_right = self.get_sensor_distance('back_right')

                # Get the current position of the robot for logging purposes
                _, position = sim.simxGetObjectPosition(self.client_id, self.left_motor, -1, sim.simx_opmode_blocking)
                x, y, _ = position

                # Default action: Move forward at a constant speed
                action_message = "Moving forward at constant speed."
                self.drive(0.3)
                left_motor_speed = 0.3
                right_motor_speed = 0.3

                # Decision-making based on sensor inputs
                if front_left <= 1:
                    # Obstacle detected at the front left, turning right
                    self.steer(-2.0, 2.0)

                    console_log("Action Taken", {
                        "Message": "Obstacle detected at front left; turning right.",
                        "Front Left Distance": front_left,
                        "Left Motor Speed": -2.0,
                        "Right Motor Speed": 2.0
                    })
                elif front_right <= 1:
                    # Obstacle detected at the front right, turning right
                    self.steer(-2.0, 2.0)
                    console_log("Action Taken", {
                        "Message": "Obstacle detected at front right; turning right.",
                        "Front Right Distance": front_right,
                        "Left Motor Speed": -2.0,
                        "Right Motor Speed": 2.0
                    })
                elif left_side <= 0.4:
                    # Too close on the left, adjusting to the right
                    self.steer(0.3, 0.1)
                    console_log("Action Taken", {
                        "Message": "Too close on the left; adjusting to the right.",
                        "Left Side Distance": left_side,
                        "Left Motor Speed": 0.3,
                        "Right Motor Speed": 0.1
                    })
                elif left_side <= 1:
                    # Maintaining distance from left wall
                    self.steer(0.1, 0.4)
                    console_log("Action Taken", {
                        "Message": "Maintaining distance from left wall.",
                        "Left Side Distance": left_side,
                        "Left Motor Speed": 0.1,
                        "Right Motor Speed": 0.4
                    })
                elif right_side <= 0.4:
                    # Too close on the right, adjusting to the left
                    self.steer(0.1, 0.3)
                    console_log("Action Taken", {
                        "Message": "Too close on the right; adjusting to the left.",
                        "Right Side Distance": right_side,
                        "Left Motor Speed": 0.1,
                        "Right Motor Speed": 0.3
                    })
                elif right_side <= 1:
                    # Maintaining distance from right wall
                    self.steer(0.4, 0.1)
                    console_log("Action Taken", {
                        "Message": "Maintaining distance from right wall.",
                        "Right Side Distance": right_side,
                        "Left Motor Speed": 0.4,
                        "Right Motor Speed": 0.1
                    })
                elif back_left <= 2:
                    # Reversing left for better alignment
                    self.steer(0.1, 2.0)
                    console_log("Action Taken", {
                        "Message": "Reversing left for better alignment.",
                        "Back Left Distance": back_left,
                        "Left Motor Speed": 0.1,
                        "Right Motor Speed": 2.0
                    })
                elif back_right <= 2:
                    # Reversing right for better alignment
                    self.steer(2.0, 0.1)
                    console_log("Action Taken", {
                        "Message": "Reversing right for better alignment.",
                        "Back Right Distance": back_right,
                        "Left Motor Speed": 2.0,
                        "Right Motor Speed": 0.1
                    })
                elif (
                        front_left == float('inf') and front_right == float('inf') and
                        left_side == float('inf') and right_side == float('inf') and
                        back_left == float('inf') and back_right == float('inf')
                ):
                    # No walls detected, start wandering
                    self.wander()
                    action_message = "No walls detected; wandering."

                # Collect data for analysis
                self.collect_data(
                    x, y,
                    front_left, front_right, left_side, right_side, back_left, back_right,
                    action_message,
                    left_motor_speed, right_motor_speed
                )

        except KeyboardInterrupt:
            console_log("Terminating simulation", "Stopping robot.")
            # Stop the robot and generate plots when stopping the simulation
            self.halt()
            self.generate_plots()

    def collect_data(self, x, y, front_left, front_right, left_side, right_side, back_left, back_right, action_message,
                     left_motor_speed, right_motor_speed):

        # Store data related to the current time step
        self.movement_data.append({
            'time': time.time(),  # Timestamp of the data collection
            'x': x,  # X position of the robot
            'y': y,  # Y position of the robot
            'front_left_distance': front_left,  # Front left sensor distance
            'front_right_distance': front_right,  # Front right sensor distance
            'left_side_distance': left_side,  # Left side sensor distance
            'right_side_distance': right_side,  # Right side sensor distance
            'back_left_distance': back_left,  # Back left sensor distance
            'back_right_distance': back_right,  # Back right sensor distance
            'action_message': action_message,  # Message indicating what action the robot took
            'left_motor_speed': left_motor_speed,  # Speed of the left motor
            'right_motor_speed': right_motor_speed  # Speed of the right motor
        })

    def generate_plots(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Create directory for plots if it doesn't exist
        import os
        if not os.path.exists('plots'):
            os.makedirs('plots')

        # Position Over Time Plot (Trajectory)
        plt.figure()
        x_coords = [data['x'] for data in self.movement_data]
        y_coords = [data['y'] for data in self.movement_data]
        plt.plot(x_coords, y_coords, label='Trajectory')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Robot Position Over Time')
        plt.legend()
        plt.grid()
        plt.savefig(f'plots/{timestamp}_robot_trajectory.png')

        # Sensor Distances Over Time Plot
        plt.figure()
        times = [data['time'] - self.movement_data[0]['time'] for data in self.movement_data]
        front_left_distances = [data['front_left_distance'] for data in self.movement_data]
        front_right_distances = [data['front_right_distance'] for data in self.movement_data]
        left_side_distances = [data['left_side_distance'] for data in self.movement_data]
        right_side_distances = [data['right_side_distance'] for data in self.movement_data]
        back_left_distances = [data['back_left_distance'] for data in self.movement_data]
        back_right_distances = [data['back_right_distance'] for data in self.movement_data]
        plt.plot(times, front_left_distances, label='Front Left Distance')
        plt.plot(times, front_right_distances, label='Front Right Distance')
        plt.plot(times, left_side_distances, label='Left Side Distance')
        plt.plot(times, right_side_distances, label='Right Side Distance')
        plt.plot(times, back_left_distances, label='Back Left Distance')
        plt.plot(times, back_right_distances, label='Back Right Distance')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.title('Sensor Distances Over Time')
        plt.legend()
        plt.grid()
        plt.savefig(f'plots/{timestamp}_sensor_distances.png')

        # Motor Speed Over Time Plot
        plt.figure()
        left_motor_speeds = [data['left_motor_speed'] for data in self.movement_data]
        right_motor_speeds = [data['right_motor_speed'] for data in self.movement_data]
        plt.plot(times, left_motor_speeds, label='Left Motor Speed')
        plt.plot(times, right_motor_speeds, label='Right Motor Speed')
        plt.xlabel('Time (s)')
        plt.ylabel('Motor Speed (m/s)')
        plt.title('Motor Speed Over Time')
        plt.legend()
        plt.grid()
        plt.savefig(f'plots/{timestamp}_motor_speeds.png')

        # Distance to Nearest Obstacle Over Time Plot
        plt.figure()
        nearest_obstacle_distances = [min(data['front_left_distance'], data['front_right_distance'],
                                          data['left_side_distance'], data['right_side_distance'],
                                          data['back_left_distance'], data['back_right_distance'])
                                      for data in self.movement_data]
        plt.plot(times, nearest_obstacle_distances, label='Nearest Obstacle Distance')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.title('Distance to Nearest Obstacle Over Time')
        plt.legend()
        plt.grid()
        plt.savefig(f'plots/{timestamp}_nearest_obstacle_distance.png')

        # Action Messages Over Time Plot
        plt.figure()
        action_messages = [data['action_message'] for data in self.movement_data]
        action_times = [data['time'] - self.movement_data[0]['time'] for data in self.movement_data]
        unique_actions = list(set(action_messages))
        action_indices = [unique_actions.index(action) for action in action_messages]
        plt.plot(action_times, action_indices, label='Action Taken Index', drawstyle='steps-post')
        plt.yticks(range(len(unique_actions)), unique_actions)
        plt.xlabel('Time (s)')
        plt.ylabel('Action')
        plt.title('Action Taken Over Time')
        plt.grid()
        plt.savefig(f'plots/{timestamp}_actions_over_time.png')

def main():
    console_log('Initializing simulation...')
    sim.simxFinish(-1)  # Close any ongoing connection
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to the simulator

    if client_id != -1:
        console_log('Connected to CoppeliaSim API.')

        # Initialize the PioneerRobot instance
        robot = PioneerRobot(client_id)

        # Example sensor readings for initial diagnostics
        for _ in range(10):
            console_log("Sensor Check", {
                "Front Left Distance": robot.get_sensor_distance('front_left'),
                "Front Right Distance": robot.get_sensor_distance('front_right'),
                "Left Side Distance": robot.get_sensor_distance('left_side'),
                "Right Side Distance": robot.get_sensor_distance('right_side'),
                "Back Left Distance": robot.get_sensor_distance('back_left'),
                "Back Right Distance": robot.get_sensor_distance('back_right'),
            })
            time.sleep(0.2)

        try:
            # Wandering Mode
            console_log("Wandering Mode", "Starting to wander until an obstacle is detected.")

            while True:
                # Perform wandering
                robot.wander()

                # Continuously check sensor distances
                front_left = robot.get_sensor_distance('front_left')
                front_right = robot.get_sensor_distance('front_right')
                left_side = robot.get_sensor_distance('left_side')
                right_side = robot.get_sensor_distance('right_side')
                back_left = robot.get_sensor_distance('back_left')
                back_right = robot.get_sensor_distance('back_right')

                # If a wall is detected (within a certain distance), switch to wall-following mode
                if (front_left < float('inf') or front_right < float('inf') or
                    left_side < float('inf') or right_side < float('inf') or
                    back_left < float('inf') or back_right < float('inf')):
                    console_log("Wall Detected", "Switching to wall-following mode.")
                    break

            # Switch to Wall-Following Logic
            robot.wander_and_wall_follow_logic()

        except KeyboardInterrupt:
            console_log("Terminating simulation", "Stopping robot.")
            robot.halt()
            robot.generate_plots()

        # Disconnect from the simulator
        sim.simxFinish(client_id)
        console_log('Simulation ended.')
    else:
        console_log('Unable to establish connection with CoppeliaSim.')
    sys.exit()

def console_log(label, message=None):
    """ Helper function to simulate console logging for debugging """
    if message is None:
        print(f"[DEBUG] {label}")
    else:
        print(f"[DEBUG] {label}: {message}")


if __name__ == '__main__':
    main()
