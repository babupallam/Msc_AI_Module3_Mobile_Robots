import sim
import time
import math
import random
import sys
import os
import matplotlib.pyplot as plt
from datetime import datetime

class PioneerRobot:
    def __init__(self, client_id):
        self.client_id = client_id
        self.timestamp=""
        # Fetch motor handles
        self.left_motor, self.right_motor = self.fetch_motor_handles()

        # Fetch sensor handles
        self.sensors = self.fetch_sensor_handles()

        # Start sensor data streaming
        self.start_sensor_stream()

        # Data for plotting
        self.movement_data = []

        # PID controller variables
        self.kp = 5  # Tuned Proportional gain
        self.ki = 0.0002  # Tuned Integral gain (very low to prevent windup)
        self.kd = 0.02  # Tuned Derivative gain
        self.error_integral = 0.0
        self.previous_error = None  # Initialize as None
        self.desired_distance = 0.5
        self.base_speed = 2 # Base forward speed
        self.wander_speed = 2.0 #Speed for wandering

    def fetch_motor_handles(self):
        _, left_motor = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
        _, right_motor = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
        return left_motor, right_motor

    def fetch_sensor_handles(self):
        sensor_map = {
            'front_left': 'Pioneer_p3dx_ultrasonicSensor5',
            'front_right': 'Pioneer_p3dx_ultrasonicSensor2',
            'left_side': 'Pioneer_p3dx_ultrasonicSensor16',
            'right_side': 'Pioneer_p3dx_ultrasonicSensor8',
            'back_left': 'Pioneer_p3dx_ultrasonicSensor14',
            'back_right': 'Pioneer_p3dx_ultrasonicSensor9'
        }
        sensor_handles = {}
        for key, sensor in sensor_map.items():
            _, sensor_handle = sim.simxGetObjectHandle(self.client_id, sensor, sim.simx_opmode_blocking)
            sensor_handles[key] = sensor_handle
        return sensor_handles

    def start_sensor_stream(self):
        for sensor_handle in self.sensors.values():
            sim.simxReadProximitySensor(self.client_id, sensor_handle, sim.simx_opmode_streaming)
        time.sleep(2)

    def get_sensor_distance(self, sensor_name):
        sensor_handle = self.sensors[sensor_name]
        _, detected, detected_point, _, _ = sim.simxReadProximitySensor(self.client_id, sensor_handle, sim.simx_opmode_buffer)
        if detected:
            return math.sqrt(sum(coord ** 2 for coord in detected_point))
        else:
            return float('inf')

    def halt(self):
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, 0, sim.simx_opmode_blocking)

    def drive(self, speed):
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, speed, sim.simx_opmode_streaming)

    def steer(self, left_speed, right_speed):
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, left_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, right_speed, sim.simx_opmode_streaming)

    def get_wall_error(self, sensor_distance):
        error = self.desired_distance - sensor_distance
        self.error_integral += error
        if self.previous_error is not None:
            derivative = error - self.previous_error
        else:
            derivative = 0
        self.previous_error = error
        return self.kp * error + self.ki * self.error_integral + self.kd * derivative

    def wander(self):
        random_turn_speed = random.uniform(-self.wander_speed, self.wander_speed)
        self.steer(self.wander_speed + random_turn_speed, self.wander_speed - random_turn_speed)
        time.sleep(0.1)

    def wander_and_wall_follow_logic(self):
        try:
            # saving the variables
            self.save_variables_plot();
            while True:
                front_left = self.get_sensor_distance('front_left')
                front_right = self.get_sensor_distance('front_right')
                left_side = self.get_sensor_distance('left_side')
                right_side = self.get_sensor_distance('right_side')
                back_left = self.get_sensor_distance('back_left')
                back_right = self.get_sensor_distance('back_right')
                _, position = sim.simxGetObjectPosition(self.client_id, self.left_motor, -1, sim.simx_opmode_blocking)
                x, y, _ = position

                action_message = "No Action"  # Default message
                left_motor_speed = self.base_speed
                right_motor_speed = self.base_speed

                # 1. Prioritize Front Obstacle Avoidance
                if front_left < self.desired_distance and front_right < self.desired_distance:  # Both front sensors detect obstacle
                    if front_left < front_right:
                        self.steer(0.1, self.base_speed)  # Turn right
                    else:
                        self.steer(self.base_speed, 0.0)  # Turn left
                    action_message = "Avoiding Front Obstacle (Both)"
                elif front_left < self.desired_distance:
                    self.steer(0.1, self.base_speed)  # Turn right
                    action_message = "Avoiding Front Obstacle (Left)"
                elif front_right < self.desired_distance:
                    self.steer(self.base_speed, 0.0)  # Turn left
                    action_message = "Avoiding Front Obstacle (Right)"

                # 2. Wall Following (if no front obstacles)
                elif left_side < 1.0 or right_side < 1.0:
                    if left_side < right_side:  # Follow left wall
                        error = self.get_wall_error(left_side)
                        left_motor_speed = self.base_speed + error
                        right_motor_speed = self.base_speed - error
                        action_message = "Following Left Wall"
                        # Additional check for sharp turns:
                        if back_left < 0.5:  # If back is too close to the wall
                            self.steer(self.base_speed, -self.base_speed)  # Make a wider turn
                            action_message = "Correcting Sharp Turn (Left)"

                    else:  # Follow right wall
                        error = self.get_wall_error(right_side)
                        left_motor_speed = self.base_speed - error
                        right_motor_speed = self.base_speed + error
                        action_message = "Following Right Wall"
                        # Additional check for sharp turns:
                        if back_right < 0.5:  # If back is too close to the wall
                            self.steer(-self.base_speed, self.base_speed)  # Make a wider turn
                            action_message = "Correcting Sharp Turn (Right)"

                    self.steer(left_motor_speed, right_motor_speed)

                # 3. Handle Back Sensors (if no front obstacles or wall following)
                elif back_left < 0.5:
                    self.steer(self.base_speed, -self.base_speed)  # Turn right
                    action_message = "Correcting Back Left Proximity"
                elif back_right < 0.5:
                    self.steer(-self.base_speed, self.base_speed)  # Turn left
                    action_message = "Correcting Back Right Proximity"

                # 4. Wander (if no other conditions met)
                else:
                    self.wander()
                    action_message = "Wandering"

                console_log("Action Taken", {
                    "Message": action_message,
                    "Front Left Distance": front_left,
                    "Front Right Distance": front_right,
                    "Left Side Distance": left_side,
                    "Right Side Distance": right_side,
                    "Back Left Distance": back_left,
                    "Back Right Distance": back_right,
                    "Left Motor Speed": left_motor_speed,
                    "Right Motor Speed": right_motor_speed,
                    "Error": error if 'error' in locals() else None
                })

                self.collect_data(x, y, front_left, front_right, left_side, right_side, back_left, back_right,
                                  action_message, left_motor_speed, right_motor_speed)
                time.sleep(0.1)

        except KeyboardInterrupt:
            console_log("Terminating simulation", "Stopping robot.")
            self.halt()
            self.generate_plots()

    def collect_data(self, x, y, front_left, front_right, left_side, right_side, back_left, back_right, action_message, left_motor_speed, right_motor_speed):
        self.movement_data.append({
            'time': time.time(),
            'x': x,
            'y': y,
            'front_left_distance': front_left,
            'front_right_distance': front_right,
            'left_side_distance': left_side,
            'right_side_distance': right_side,
            'back_left_distance': back_left,
            'back_right_distance': back_right,
            'action_message': action_message,
            'left_motor_speed': left_motor_speed,
            'right_motor_speed': right_motor_speed
        })
    # saving the variable

    def save_variables_plot(self):
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if not os.path.exists('plots'):
            os.makedirs('plots')


        labels = ["Kp", "Ki", "Kd", "Error Integral", "Desired Distance", "Base Speed", "Wander Speed"]
        values = [self.kp, self.ki, self.kd, self.error_integral, self.desired_distance, self.base_speed,
                  self.wander_speed]

        plt.figure(figsize=(10, 6))  # Increased figure size for better readability
        bars = plt.bar(labels, values, color='skyblue')
        plt.xlabel("Variable")
        plt.ylabel("Value")
        plt.title("PID Controller Variables")
        plt.xticks(rotation=45, ha="right")

        # Annotate each bar with its value
        for bar, value in zip(bars, values):
            height = bar.get_height()
            if abs(value) > 0.001:  # Check if the value is not too close to zero
                plt.text(bar.get_x() + bar.get_width() / 2, height, f'{value:.4g}', ha='center', va='bottom',
                         fontsize=9, fontweight='bold')
            else:
                plt.text(bar.get_x() + bar.get_width() / 2, height, f'{value:.2e}', ha='center', va='bottom',
                         fontsize=9, fontweight='bold')

        plt.tight_layout()
        plt.savefig(f'plots/{self.timestamp}_pid_controller_variables.png')
        plt.close()

    def generate_plots(self):
        times = [data['time'] - self.movement_data[0]['time'] for data in self.movement_data]
        # Trajectory
        plt.figure()
        plt.plot([data['x'] for data in self.movement_data], [data['y'] for data in self.movement_data])
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Robot Trajectory')
        plt.savefig(f'plots/{self.timestamp}_trajectory.png')

        # Sensor Distances
        plt.figure()
        plt.plot(times, [data['front_left_distance'] for data in self.movement_data], label='Front Left')
        plt.plot(times, [data['front_right_distance'] for data in self.movement_data], label='Front Right')
        plt.plot(times, [data['left_side_distance'] for data in self.movement_data], label='Left Side')
        plt.plot(times, [data['right_side_distance'] for data in self.movement_data], label='Right Side')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.title('Sensor Distances')
        plt.legend()
        plt.savefig(f'plots/{self.timestamp}_sensor_distances.png')

        # Motor Speeds
        plt.figure()
        plt.plot(times, [data['left_motor_speed'] for data in self.movement_data], label='Left Motor')
        plt.plot(times, [data['right_motor_speed'] for data in self.movement_data], label='Right Motor')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.title('Motor Speeds')
        plt.legend()
        plt.savefig(f'plots/{self.timestamp}_motor_speeds.png')

        # Actions
        plt.figure()
        actions = [data['action_message'] for data in self.movement_data]
        unique_actions = sorted(list(set(actions)))  # Sort for consistency
        action_indices = [unique_actions.index(action) for action in actions]
        plt.plot(times, action_indices, drawstyle='steps-post')
        plt.yticks(range(len(unique_actions)), unique_actions)
        plt.xlabel("Time (s)")
        plt.ylabel("Action")
        plt.title("Actions Over Time")
        plt.savefig(f'plots/{self.timestamp}_actions.png')


def main():
    console_log('Initializing simulation...')
    sim.simxFinish(-1)
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if client_id != -1:
        console_log('Connected to CoppeliaSim API.')
        robot = PioneerRobot(client_id)
        time.sleep(1) # Important: Give time for the simulation to fully initialize

        try:
            robot.wander_and_wall_follow_logic()
        except KeyboardInterrupt:
            console_log("Terminating simulation", "Stopping robot.")
            robot.halt()
            robot.generate_plots()

        sim.simxFinish(client_id)
        console_log('Simulation ended.')
    else:
        console_log('Unable to establish connection with CoppeliaSim.')
    sys.exit()

def console_log(label, message=None):
    if message is None:
        print(f"[DEBUG] {label}")
    else:
        print(f"[DEBUG] {label}: {message}")

if __name__ == '__main__':
    main()