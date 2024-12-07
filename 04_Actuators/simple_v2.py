import sim
import time
import math
import random
import sys


class PioneerRobot:
    def __init__(self, client_id):
        self.client_id = client_id

        # Fetch motor handles
        self.left_motor, self.right_motor = self.fetch_motor_handles()

        # Fetch sensor handles
        self.sensors = self.fetch_sensor_handles()

        # Start sensor data streaming
        self.start_sensor_stream()

    def fetch_motor_handles(self):
        # Get handles for motors
        _, left_motor = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
        _, right_motor = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
        return left_motor, right_motor

    def fetch_sensor_handles(self):
        # Obtain sensor handles for all important sensors
        sensor_map = {
            'front_left': 'Pioneer_p3dx_ultrasonicSensor5',
            'front_right': 'Pioneer_p3dx_ultrasonicSensor2',
            'left_side': 'Pioneer_p3dx_ultrasonicSensor15',
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
        random_move_speed = random.uniform(-0.5, 2.0)

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
                front_left = self.get_sensor_distance('front_left')
                front_right = self.get_sensor_distance('front_right')
                left_side = self.get_sensor_distance('left_side')
                right_side = self.get_sensor_distance('right_side')
                back_left = self.get_sensor_distance('back_left')
                back_right = self.get_sensor_distance('back_right')

                self.drive(0.3)  # Move forward at constant speed

                if front_left <= 1:
                    self.steer(-2.0, 2.0)
                    console_log("Action Taken", {
                        "Message": "Obstacle detected at front left; turning right.",
                        "Front Left Distance": front_left,
                        "Left Motor Speed": -2.0,
                        "Right Motor Speed": 2.0
                    })
                elif front_right <= 1:
                    self.steer(-2.0, 2.0)
                    console_log("Action Taken", {
                        "Message": "Obstacle detected at front right; turning right.",
                        "Front Right Distance": front_right,
                        "Left Motor Speed": -2.0,
                        "Right Motor Speed": 2.0
                    })
                elif left_side <= 0.3:
                    self.steer(0.3, 0.1)
                    console_log("Action Taken", {
                        "Message": "Too close on the left; adjusting to the right.",
                        "Left Side Distance": left_side,
                        "Left Motor Speed": 0.3,
                        "Right Motor Speed": 0.1
                    })
                elif left_side <= 1:
                    self.steer(0.1, 0.4)
                    console_log("Action Taken", {
                        "Message": "Maintaining distance from left wall.",
                        "Left Side Distance": left_side,
                        "Left Motor Speed": 0.1,
                        "Right Motor Speed": 0.4
                    })
                elif right_side <= 0.3:
                    self.steer(0.1, 0.3)
                    console_log("Action Taken", {
                        "Message": "Too close on the right; adjusting to the left.",
                        "Right Side Distance": right_side,
                        "Left Motor Speed": 0.1,
                        "Right Motor Speed": 0.3
                    })
                elif right_side <= 1:
                    self.steer(0.4, 0.1)
                    console_log("Action Taken", {
                        "Message": "Maintaining distance from right wall.",
                        "Right Side Distance": right_side,
                        "Left Motor Speed": 0.4,
                        "Right Motor Speed": 0.1
                    })
                elif back_left <= 2:
                    self.steer(0.1, 2.0)
                    console_log("Action Taken", {
                        "Message": "Reversing left for better alignment.",
                        "Back Left Distance": back_left,
                        "Left Motor Speed": 0.1,
                        "Right Motor Speed": 2.0
                    })
                elif back_right <= 2:
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
                    # Start wandering only when no walls are detected
                    self.wander()



        except KeyboardInterrupt:
            console_log("Terminating simulation", "Stopping robot.")
            self.halt()

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

        # Begin wandering and wall-following logic
        robot.wander_and_wall_follow_logic()

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
