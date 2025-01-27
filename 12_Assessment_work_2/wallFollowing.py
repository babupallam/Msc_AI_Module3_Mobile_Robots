import sim
import time
import math
import random
import sys
import matplotlib.pyplot as plt
from datetime import datetime
import os

class MobileRobotController:
    def __init__(self, client_identifier):
        self.client_id = client_identifier

        # Retrieve handles for both motors
        self.motor_left, self.motor_right = self.obtain_motor_handles()

        # Retrieve handles for only three sensors used for navigation
        self.sensor_lookup = {
            'front_left': 'Pioneer_p3dx_ultrasonicSensor1',
            'front_right': 'Pioneer_p3dx_ultrasonicSensor8',
            'front_center': 'Pioneer_p3dx_ultrasonicSensor5',
        }
        self.sensor_handles = self.obtain_sensor_handles()

        # Begin streaming proximity sensor data
        self.initialize_sensors()

        # Records for data visualization
        self.navigation_data = []

        # PID parameters for wall-following
        self.kp = 9.2
        self.ki = 0.006
        self.kd = 0.22
        self.integrated_error = 0.0
        self.prev_error = None
        self.target_distance = 0.5

        # Movement parameters
        self.forward_velocity = 1.4
        self.roam_velocity = 4.0

    def obtain_motor_handles(self):
        # Acquire handles for the left and right motors in the simulation
        _, left_handle = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
        _, right_handle = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
        return left_handle, right_handle

    def obtain_sensor_handles(self):
        # Acquire handles for the relevant proximity sensors
        sensors_obtained = {}
        for label, sensor_name in self.sensor_lookup.items():
            _, sensor_handle = sim.simxGetObjectHandle(self.client_id, sensor_name, sim.simx_opmode_blocking)
            sensors_obtained[label] = sensor_handle
        return sensors_obtained

    def initialize_sensors(self):
        # Start streaming data from all three sensors
        for handle in self.sensor_handles.values():
            sim.simxReadProximitySensor(self.client_id, handle, sim.simx_opmode_streaming)
        time.sleep(2)

    def read_sensor_distance(self, sensor_label):
        # Get the distance reading from a specific sensor
        sensor_handle = self.sensor_handles[sensor_label]
        _, detected, detection_point, _, _ = sim.simxReadProximitySensor(self.client_id, sensor_handle, sim.simx_opmode_buffer)
        if detected:
            return math.sqrt(sum(coord ** 2 for coord in detection_point))
        else:
            return float('inf')

    def stop_movement(self):
        # Command the motors to cease all motion
        sim.simxSetJointTargetVelocity(self.client_id, self.motor_left, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.client_id, self.motor_right, 0, sim.simx_opmode_blocking)

    def move_straight(self, velocity):
        # Move forward (or backward if velocity is negative) at the same speed on both motors
        sim.simxSetJointTargetVelocity(self.client_id, self.motor_left, velocity, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.motor_right, velocity, sim.simx_opmode_streaming)

    def set_wheel_speeds(self, left_speed, right_speed):
        # Assign different speeds to the two motors
        sim.simxSetJointTargetVelocity(self.client_id, self.motor_left, left_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.motor_right, right_speed, sim.simx_opmode_streaming)

    def calc_pid_adjustment(self, current_distance):
        # Compute the PID correction based on difference from the desired wall distance
        error = self.target_distance - current_distance
        self.integrated_error += error
        if self.prev_error is not None:
            derivative = error - self.prev_error
        else:
            derivative = 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integrated_error + self.kd * derivative

    def random_roaming(self):
        # Choose a random offset speed for turning and make the robot roam
        random_turn = random.uniform(-self.roam_velocity, self.roam_velocity)
        self.set_wheel_speeds(self.roam_velocity + random_turn, self.roam_velocity - random_turn)
        time.sleep(0.1)

    def roam_and_follow_walls(self):
        try:
            while True:
                # Read sensor data
                dist_front_left = self.read_sensor_distance('front_left')
                dist_front_right = self.read_sensor_distance('front_right')
                dist_front_center = self.read_sensor_distance('front_center')

                # Get robot's position for data logging
                _, robot_position = sim.simxGetObjectPosition(self.client_id, self.motor_left, -1, sim.simx_opmode_blocking)
                x_coord, y_coord, _ = robot_position

                action_description = "None"
                speed_left = self.forward_velocity
                speed_right = self.forward_velocity
                correction = 0

                # (1) Avoid obstacles if something is close in front
                if dist_front_center < 0.5:
                    # Turn away from the closer side
                    if dist_front_left < dist_front_right:
                        self.set_wheel_speeds(self.forward_velocity, -self.forward_velocity)  # rotate right
                        action_description = "Front Obstacle Detected, Turning Right"
                    else:
                        self.set_wheel_speeds(-self.forward_velocity, self.forward_velocity)  # rotate left
                        action_description = "Front Obstacle Detected, Turning Left"

                # (2) Use PID for wall-following if an obstacle is detected on either side
                elif dist_front_left < 1.0 or dist_front_right < 1.0:
                    if dist_front_left < dist_front_right:
                        correction = self.calc_pid_adjustment(dist_front_left)
                        speed_left = self.forward_velocity + correction
                        speed_right = self.forward_velocity - correction
                        action_description = "PID Following on Left Wall"
                    else:
                        correction = self.calc_pid_adjustment(dist_front_right)
                        speed_left = self.forward_velocity - correction
                        speed_right = self.forward_velocity + correction
                        action_description = "PID Following on Right Wall"
                    self.set_wheel_speeds(speed_left, speed_right)

                # (3) Explore randomly if no immediate wall or front obstacle is detected
                else:
                    self.random_roaming()
                    action_description = "Random Roaming"

                debug_message("Action", {
                    "Action": action_description,
                    "Distance FL": dist_front_left,
                    "Distance FR": dist_front_right,
                    "Distance FC": dist_front_center,
                    "Left Speed": speed_left,
                    "Right Speed": speed_right,
                    "PID Correction": correction if 'correction' in locals() else None
                })

                # Collect data for later plotting
                self.record_data(x_coord, y_coord, dist_front_left, dist_front_right, dist_front_center,
                                 speed_left, speed_right, action_description)

        except KeyboardInterrupt:
            debug_message("Stopping", "Keyboard Interrupt - Halting Robot.")
            self.stop_movement()
            self.produce_graphs()

    def record_data(self, x, y, dist_fl, dist_fr, dist_fc, l_speed, r_speed, action):
        # Keep logs of important parameters over time for analysis
        self.navigation_data.append({
            'timestamp': time.time(),
            'pos_x': x,
            'pos_y': y,
            'dist_front_left': dist_fl,
            'dist_front_right': dist_fr,
            'dist_front_center': dist_fc,
            'motor_left_speed': l_speed,
            'motor_right_speed': r_speed,
            'current_action': action
        })

    def produce_graphs(self):
        # Prepare a timestamp for file naming
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        if not os.path.exists('plots'):
            os.makedirs('plots')

        # Normalize the time series so plotting starts from 0
        elapsed_times = [entry['timestamp'] - self.navigation_data[0]['timestamp']
                         for entry in self.navigation_data]

        # 1. Plot the robot's 2D path (kept separate as requested)
        plt.figure()
        plt.plot([point['pos_x'] for point in self.navigation_data],
                 [point['pos_y'] for point in self.navigation_data])
        plt.title('Robot Path')
        plt.xlabel('X-Coordinate (m)')
        plt.ylabel('Y-Coordinate (m)')
        plt.savefig(f'plots/{current_time}_trajectory.png')
        plt.close()

        # 2. Create a single figure with subplots for sensor data, motor speeds, and actions
        fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(10, 12))

        # Subplot A: Sensor readings (front-left, front-right, front-center)
        axes[0].plot(elapsed_times,
                     [item['dist_front_left'] for item in self.navigation_data],
                     label='Front-Left')
        axes[0].plot(elapsed_times,
                     [item['dist_front_right'] for item in self.navigation_data],
                     label='Front-Right')
        axes[0].plot(elapsed_times,
                     [item['dist_front_center'] for item in self.navigation_data],
                     label='Front-Center')
        axes[0].set_title('Sensor Distances Over Time')
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Distance (m)')
        axes[0].legend()

        # Subplot B: Motor speeds (left vs. right)
        axes[1].plot(elapsed_times,
                     [item['motor_left_speed'] for item in self.navigation_data],
                     label='Left Motor')
        axes[1].plot(elapsed_times,
                     [item['motor_right_speed'] for item in self.navigation_data],
                     label='Right Motor')
        axes[1].set_title('Motor Speeds Over Time')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Speed')
        axes[1].legend()

        # Subplot C: Actions taken
        all_actions = [entry['current_action'] for entry in self.navigation_data]
        unique_actions = sorted(set(all_actions))
        indexed_actions = [unique_actions.index(action) for action in all_actions]

        axes[2].plot(elapsed_times, indexed_actions, drawstyle='steps-post')
        axes[2].set_yticks(range(len(unique_actions)))
        axes[2].set_yticklabels(unique_actions)
        axes[2].set_title('Actions Over Time')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Action Index')

        fig.tight_layout()
        fig.savefig(f'plots/{current_time}_combined_plots.png')
        plt.close(fig)



def main_program():
    debug_message("Initialization", "Attempting to start the simulation...")
    sim.simxFinish(-1)  # Terminate any current connections

    try:
        # Establish connection to CoppeliaSim
        client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if client_id != -1:
            debug_message('Connection Status', 'Connected to CoppeliaSim API.')
            # Start simulation in synchronous mode
            sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)
            time.sleep(1)  # Let simulation stabilize

            robot_control = MobileRobotController(client_id)

            try:
                # Execute the primary robot behavior
                robot_control.roam_and_follow_walls()

            except KeyboardInterrupt:
                debug_message("End Simulation", "Process was interrupted by user.")
                robot_control.stop_movement()
                robot_control.produce_graphs()

            finally:
                # Ensure the simulation is halted and the connection is closed
                sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
                time.sleep(1)
                sim.simxFinish(client_id)
                debug_message('Cleanup', 'Simulation has been stopped, and the connection is closed.')

        else:
            debug_message('Connection Failed', 'Could not connect to the simulation environment.')

    except Exception as e:
        debug_message("Error Occurred", str(e))
    finally:
        sys.exit()

def debug_message(title, content=None):
    # Helper function to print debug messages
    if content is None:
        print(f"[DEBUG] {title}")
    else:
        print(f"[DEBUG] {title}: {content}")

if __name__ == '__main__':
    main_program()
