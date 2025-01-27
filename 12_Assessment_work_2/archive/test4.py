import sim
import time
import math
import random
import sys

# Step 1: Define the PioneerRobot class with sensor and motor setup
class PioneerRobot:
    def __init__(self, client_id):
        # Step 1.1: Store the client ID for CoppeliaSim communications
        self.client_id = client_id

        # Step 1.2: Fetch motor handles (left and right)
        _, self.left_motor = sim.simxGetObjectHandle(self.client_id,
                                                     'Pioneer_p3dx_leftMotor',
                                                     sim.simx_opmode_blocking)
        _, self.right_motor = sim.simxGetObjectHandle(self.client_id,
                                                      'Pioneer_p3dx_rightMotor',
                                                      sim.simx_opmode_blocking)

        # Step 1.3: Fetch sensor handles for front, side, back
        sensor_map = {
            'front_left': 'Pioneer_p3dx_ultrasonicSensor5',
            'front_right': 'Pioneer_p3dx_ultrasonicSensor2',
            'left_side': 'Pioneer_p3dx_ultrasonicSensor16',
            'right_side': 'Pioneer_p3dx_ultrasonicSensor8',
            'back_left': 'Pioneer_p3dx_ultrasonicSensor14',
            'back_right': 'Pioneer_p3dx_ultrasonicSensor9'
        }
        self.sensors = {}
        for key, sensor in sensor_map.items():
            _, sensor_handle = sim.simxGetObjectHandle(self.client_id,
                                                       sensor,
                                                       sim.simx_opmode_blocking)
            self.sensors[key] = sensor_handle

        # Step 1.4: Start streaming data from sensors
        for sensor_handle in self.sensors.values():
            sim.simxReadProximitySensor(self.client_id, sensor_handle, sim.simx_opmode_streaming)
        time.sleep(2)  # Allow some time for streaming to initiate

        # Step 1.5: Initialize PID gains and variables for wall-following
        self.Kp = 656
        self.Ki = 0.0
        self.Kd = 0.0002
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    # Step 2: A helper function to read sensor distances
    def get_sensor_distance(self, sensor_name):
        sensor_handle = self.sensors[sensor_name]
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(self.client_id,
                                                                            sensor_handle,
                                                                            sim.simx_opmode_buffer)
        if detectionState:
            return math.sqrt(sum(coord ** 2 for coord in detectedPoint))
        else:
            return float('inf')

    # Step 3: Basic movement and steering commands
    def steer(self, left_speed, right_speed):
        sim.simxSetJointTargetVelocity(self.client_id,
                                       self.left_motor,
                                       left_speed,
                                       sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id,
                                       self.right_motor,
                                       right_speed,
                                       sim.simx_opmode_streaming)

    def halt(self):
        sim.simxSetJointTargetVelocity(self.client_id,
                                       self.left_motor,
                                       0,
                                       sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.client_id,
                                       self.right_motor,
                                       0,
                                       sim.simx_opmode_blocking)

    # Step 4: Wandering function - random forward/back plus random turn
    def wander(self):
        random_move_speed = random.uniform(-0.5, 3.0)
        random_turn_speed = random.uniform(-1.0, 1.0)
        left_speed = random_move_speed + random_turn_speed
        right_speed = random_move_speed - random_turn_speed
        self.steer(left_speed, right_speed)
        time.sleep(0.5)
        console_log("Action Taken", {
            "Message": "Wandering randomly.",
            "Random Move Speed": random_move_speed,
            "Random Turn Speed": random_turn_speed
        })

    # Step 5: Collect data (stub or implement as needed)
    def collect_data(self, x, y,
                     front_left, front_right, left_side, right_side,
                     back_left, back_right,
                     action_message,
                     left_motor_speed, right_motor_speed):
        # In a real scenario, store data in a list or file
        pass

    # Step 6: Hybrid approach:
    #         if obstacles are detected, override with discrete decisions,
    #         else use a PID approach for side wall following,
    #         or wander if no walls at all.
    def wander_and_wall_follow_pid(self):
        # Step 6.1: Choose desired wall distance, forward speed, and turn clamp
        target_distance = 0.5
        base_speed = 3
        max_turn = 0.3 # Maximum Threshold value given in the Coursework Guidelines.

        console_log("PID Wall-Following", "Starting combined threshold + PID approach")

        try:
            while True:
                # Step 6.2: Read sensor data
                front_left = self.get_sensor_distance('front_left')
                front_right = self.get_sensor_distance('front_right')
                left_side = self.get_sensor_distance('left_side')
                right_side = self.get_sensor_distance('right_side')
                back_left = self.get_sensor_distance('back_left')
                back_right = self.get_sensor_distance('back_right')

                # For logging
                x, y = 0.0, 0.0
                action_message = "PID wall-following"
                left_motor_speed = 0.0
                right_motor_speed = 0.0

                # Step 6.3: Threshold-based checks for urgent or special conditions
                if front_left <= 1:
                    # Obstacle at front left, turn right strongly
                    self.steer(-2.0, 2.0)
                    action_message = "Obstacle at front left; turning right."
                elif front_right <= 1:
                    # Obstacle at front right, also turn right
                    self.steer(-2.0, 2.0)
                    action_message = "Obstacle at front right; turning right."
                elif left_side <= 0.4:
                    # Too close on the left => immediate tweak
                    self.steer(0.9, 0.3)
                    action_message = "Too close on left; adjusting right."
                elif right_side <= 0.4:
                    # Too close on the right => immediate tweak
                    self.steer(0.3, 0.9)
                    action_message = "Too close on right; adjusting left."
                elif back_left <= 2:
                    # Reversing left
                    self.steer(0.1, 2.0)
                    action_message = "Back left close; reversing alignment."
                elif back_right <= 2:
                    # Reversing right
                    self.steer(2.0, 0.1)
                    action_message = "Back right close; reversing alignment."
                elif (
                    front_left == float('inf') and front_right == float('inf') and
                    left_side == float('inf') and right_side == float('inf') and
                    back_left == float('inf') and back_right == float('inf')
                ):
                    # No walls anywhere => wander
                    self.wander()
                    action_message = "No walls detected; wandering."
                else:
                    # Step 6.4: If no urgent conditions, use PID for side wall
                    current_time = time.time()
                    dt = current_time - self.last_time
                    if dt <= 0:
                        dt = 0.01

                    # Choose which side to follow (here, left if it's smaller)
                    use_left = (left_side < right_side)
                    if use_left:
                        distance = left_side
                    else:
                        distance = right_side

                    # Compute error from target distance
                    error = target_distance - distance

                    # Accumulate integral, compute derivative
                    self.error_integral += error * dt
                    d_error = (error - self.last_error) / dt

                    # PID output
                    output = (self.Kp * error) + (self.Ki * self.error_integral) + (self.Kd * d_error)

                    # Clamp if needed
                    if output > max_turn:
                        output = max_turn
                    elif output < -max_turn:
                        output = -max_turn

                    # Adjust left/right speeds
                    if use_left:
                        left_motor_speed = base_speed - output
                        right_motor_speed = base_speed + output
                    else:
                        left_motor_speed = base_speed + output
                        right_motor_speed = base_speed - output

                    self.steer(left_motor_speed, right_motor_speed)

                    # Update for next loop
                    self.last_error = error
                    self.last_time = current_time
                    action_message = f"PID controlling on {'left' if use_left else 'right'} wall."

                # Step 6.5: Log or store the data
                self.collect_data(x, y,
                                  front_left, front_right, left_side, right_side, back_left, back_right,
                                  action_message,
                                  left_motor_speed, right_motor_speed)

                # Small delay for loop
                time.sleep(0.1)

        except KeyboardInterrupt:
            console_log("Terminating simulation", "Stopping robot.")
            self.halt()

# Step 7: Console logging helper
def console_log(label, message=None):
    if message is None:
        print(f"[DEBUG] {label}")
    else:
        print(f"[DEBUG] {label}: {message}")

# Step 8: Main function to connect, create robot, and run logic
def main():
    console_log('Initializing simulation...')
    sim.simxFinish(-1)  # close ongoing connections if any
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if client_id != -1:
        console_log('Connected to CoppeliaSim API.')

        # Start the simulation from code
        console_log("Simulation Control", "Starting the simulation from code.")
        sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)

        # Create robot instance
        robot = PioneerRobot(client_id)

        # Example: Check sensors briefly
        for _ in range(3):
            console_log("Sensor Check", {
                "Front Left": robot.get_sensor_distance('front_left'),
                "Front Right": robot.get_sensor_distance('front_right'),
                "Left Side": robot.get_sensor_distance('left_side'),
                "Right Side": robot.get_sensor_distance('right_side')
            })
            time.sleep(0.2)

        # Call the combined threshold + PID wall-following
        robot.wander_and_wall_follow_pid()

        # Stop simulation before ending
        console_log("Simulation Control", "Stopping the simulation.")
        sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
        time.sleep(1)

        # Disconnect
        sim.simxFinish(client_id)
        console_log('Simulation ended.')
    else:
        console_log('Unable to connect to CoppeliaSim.')
    sys.exit()

# Step 9: Run the main function
if __name__ == '__main__':
    main()
