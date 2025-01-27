# STEP 1: Import necessary libraries to interface with CoppeliaSim and handle data processing
import sim
import time
import math
import sys

# STEP 2: Define a helper function to clamp sensor distances, preventing inf or very large values
def clamp_distance(distance, max_dist=2.0):
    """
    If the distance from a sensor is infinity (or beyond max_dist),
    clamp it to a chosen maximum distance (e.g., 2.0 meters).
    """
    if distance == float('inf'):
        return max_dist
    return min(distance, max_dist)

# STEP 3: Define a PioneerRobot class that encapsulates basic robot controls and PID-based wall following
class PioneerRobot:
    def __init__(self, client_id):
        # STEP 3.1: Assign the connected client's ID
        self.client_id = client_id

        # STEP 3.2: Fetch motor handles for the robot's wheels
        self.left_motor, self.right_motor = self.fetch_motor_handles()

        # STEP 3.3: Fetch sensor handles for front, left, and right sensors
        self.sensors = self.fetch_sensor_handles()

        # STEP 3.4: Start streaming data from these sensors
        self.start_sensor_stream()
        # Let's say you want to maintain 0.5 meters from the left wall
        self.desired_right_distance = 0.5
        self.minimum_right_distance = 0.3  # e.g., threshold to avoid getting too close on the right

        # STEP 3.5: Initialize PID parameters
        self.Kp = 10   # Proportional gain
        self.Ki = 0.00002   # Integral gain
        self.Kd = 0.2   # Derivative gain

        # STEP 3.6: Initialize PID state variables
        self.prev_error = 0.0
        self.integral = 0.0

        # STEP 3.7: We aim to stay centered, so the target difference is zero
        # (i.e., left_dist ~ right_dist).
        self.target_side_balance = 0.0

        # STEP 3.8: Baseline forward speed
        self.base_speed = 3

    def fetch_motor_handles(self):
        """
        STEP 4: Get object handles for the robot's left and right motors from CoppeliaSim
        """
        _, left_motor = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
        _, right_motor = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
        return left_motor, right_motor

    def fetch_sensor_handles(self):
        """
        STEP 5: Use only front, left, and right sensors.
        Adjust these sensor names to match your actual sensor objects in CoppeliaSim.
        """
        sensor_map = {
            'front_sensor': 'Pioneer_p3dx_ultrasonicSensor5',
            'left_sensor': 'Pioneer_p3dx_ultrasonicSensor1',
            'right_sensor': 'Pioneer_p3dx_ultrasonicSensor8'
        }
        sensor_handles = {}
        for key, sensor_name in sensor_map.items():
            _, sensor_handle = sim.simxGetObjectHandle(self.client_id, sensor_name, sim.simx_opmode_blocking)
            sensor_handles[key] = sensor_handle
        return sensor_handles

    def start_sensor_stream(self):
        """
        STEP 6: Initiate data streaming for all three sensors
        """
        for sensor_handle in self.sensors.values():
            sim.simxReadProximitySensor(self.client_id, sensor_handle, sim.simx_opmode_streaming)
        # Allow some time for sensor streaming to initialize
        time.sleep(2)

    def get_sensor_distance(self, sensor_name):
        """
        STEP 7: Retrieve the distance measurement from the specified sensor,
        clamping infinite values to avoid generating inf or NaN in the PID.
        """
        sensor_handle = self.sensors[sensor_name]
        _, detected, detected_point, _, _ = sim.simxReadProximitySensor(
            self.client_id, sensor_handle, sim.simx_opmode_buffer
        )
        if detected:
            distance = math.sqrt(sum(coord ** 2 for coord in detected_point))
            return clamp_distance(distance)
        else:
            # Return a clamped max distance if no obstacle is detected
            return clamp_distance(float('inf'))

    def halt(self):
        """
        STEP 8: Stop both motors
        """
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, 0, sim.simx_opmode_blocking)

    def steer(self, left_speed, right_speed):
        """
        STEP 9: Set individual speeds for the left and right motors
        """
        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor, left_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor, right_speed, sim.simx_opmode_streaming)

    def wall_follow_pid(self):
        """
        STEP 10: Main loop implementing PID control for wall-following, using:
            - front_sensor: for checking front obstacles
            - left_sensor and right_sensor: for staying centered.
        """
        previous_time = time.time()

        try:
            while True:
                # STEP 10.1: Get current sensor distances (already clamped)
                front_dist = self.get_sensor_distance('front_sensor')
                left_dist  = self.get_sensor_distance('left_sensor')
                right_dist = self.get_sensor_distance('right_sensor')

                # STEP 10.2: If both left and right are at max, no walls around -> zero error
                # Otherwise, the error is the difference between left and right from our target
                if left_dist == right_dist == 2.0:
                    error = 0.0
                else:
                    error = (left_dist - right_dist) - self.target_side_balance

                # STEP 10.3: Compute time delta for integral and derivative
                current_time = time.time()
                delta_time   = current_time - previous_time
                previous_time = current_time

                # STEP 10.4: PID calculation
                # Guard against very small delta_time (or zero)
                if delta_time <= 0:
                    delta_time = 1e-3

                self.integral += error * delta_time
                derivative = (error - self.prev_error) / delta_time

                pid_output = (
                    (self.Kp * error) +
                    (self.Ki * self.integral) +
                    (self.Kd * derivative)
                )

                self.prev_error = error

                # STEP 10.5: Check for a front obstacle and handle it
                if front_dist < 0.3:
                    # If an obstacle is directly in front, pivot to avoid collision
                    self.steer(-2.0, 2.0)
                    console_log("PID Action", f"Obstacle ahead. Pivoting. FrontDist={front_dist:.2f}")
                else:
                    # STEP 10.6: Map PID output to motor speeds
                    left_speed  = self.base_speed + pid_output
                    right_speed = self.base_speed - pid_output

                    # STEP 10.7: Optional speed clamping (to keep speeds within a safe range)
                    left_speed  = max(min(left_speed, 2.0), -2.0)
                    right_speed = max(min(right_speed, 2.0), -2.0)

                    # STEP 10.8: Send speed commands to the motors
                    self.steer(left_speed, right_speed)

                    # STEP 10.9: Log relevant debug information
                    console_log("PID Action", {
                        "Error": error,
                        "P": self.Kp * error,
                        "I": self.Ki * self.integral,
                        "D": self.Kd * derivative,
                        "PID Output": pid_output,
                        "Left Speed": left_speed,
                        "Right Speed": right_speed,
                        "Front Dist": front_dist,
                        "Left Dist": left_dist,
                        "Right Dist": right_dist
                    })

                # STEP 10.10: Delay to avoid excessive CPU usage
                time.sleep(0.1)

        except KeyboardInterrupt:
            console_log("Terminating simulation", "Stopping robot via KeyboardInterrupt.")
            self.halt()

# STEP 11: Define the main function to initialize the robot and run PID-based wall following
def main():
    console_log('Initializing simulation...')
    sim.simxFinish(-1)  # Close any existing connections
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim

    if client_id != -1:
        console_log('Connected to CoppeliaSim API.')

        # STEP 11.1: Create an instance of the PioneerRobot
        robot = PioneerRobot(client_id)

        # STEP 11.2: Start the PID-based wall-following logic
        robot.wall_follow_pid()

        # STEP 11.3: Disconnect after the loop ends
        sim.simxFinish(client_id)
        console_log('Simulation ended.')
    else:
        console_log('Unable to establish connection with CoppeliaSim.')
    sys.exit()

# STEP 12: Helper function for console logs
def console_log(label, message=None):
    if message is None:
        print(f"[DEBUG] {label}")
    else:
        print(f"[DEBUG] {label}: {message}")

# STEP 13: Entry point for script execution
if __name__ == '__main__':
    main()
