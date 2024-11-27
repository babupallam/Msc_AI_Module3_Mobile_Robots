function test(serPort)
% Complete robot navigation program with obstacle avoidance.
% serPort is the serial port number for robot communication.

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('==================')
disp('Program Starting')
disp('------------------')

% Step 1: Define parameters
safeDistance = 0.5;  % Minimum safe distance (in meters) to an obstacle
speed = 0.3;         % Speed for forward movement (in m/s)
turnSpeed = 0.2;     % Speed for turning (in rad/s)
turnAngleCCW = 90;   % Angle for counter-clockwise turn
turnAngleCW = -90;   % Angle for clockwise turn

% Step 2: Navigate the environment with sensor-based safety checks
navigateWithSensors(serPort, speed, 2, safeDistance);  % Move forward 2m safely
turnWithSensors(serPort, turnSpeed, turnAngleCCW);      % Turn counter-clockwise

navigateWithSensors(serPort, speed, 2.5, safeDistance); % Move forward 2.5m safely
turnWithSensors(serPort, turnSpeed, turnAngleCCW);       % Turn counter-clockwise

% Additional navigation logic can be added here for extended paths.

disp('Program Completed Successfully')
disp('===============================')

end

%% Function: Navigate forward safely with sensor checks
function navigateWithSensors(serPort, speed, distance, safeDistance)
% Moves the robot forward while checking for obstacles.

currentDistance = 0;  % Track how far the robot has traveled
SetDriveWheelsCreate(serPort, speed, speed);  % Start moving forward
while currentDistance < distance
    % Check front sonar for obstacles
    frontDistance = ReadSonar(serPort, 2);  % Front sensor (Sonar 2)
    if ~any(frontDistance) 
        frontDistance = 100;  % Default to a high value if no reading
    end

    if frontDistance < safeDistance
        % Obstacle detected: stop and avoid
        disp('Obstacle detected! Initiating avoidance.')
        SetDriveWheelsCreate(serPort, 0, 0);  % Stop
        avoidObstacle(serPort, safeDistance);  % Call obstacle avoidance logic
    end
    
    pause(0.1);  % Short delay to prevent sensor overload
    currentDistance = currentDistance + speed * 0.1;  % Approximate travel distance
end

SetDriveWheelsCreate(serPort, 0, 0);  % Stop once target distance is reached
end

%% Function: Turn the robot safely
function turnWithSensors(serPort, turnSpeed, angle)
% Turns the robot with sensor monitoring (optional).
turnAngle(serPort, turnSpeed, angle);  % Execute the turn command
end

%% Function: Obstacle avoidance logic
function avoidObstacle(serPort, safeDistance)
% Implements a simple obstacle avoidance maneuver.

turnSpeed = 0.2;    % Speed for turning
backupDistance = 0.3;  % Distance to back up (in meters)

% Step 1: Back up slightly
disp('Backing up...')
travelDist(serPort, -0.2, -backupDistance);  % Move backward

% Step 2: Turn to avoid the obstacle
disp('Turning to avoid obstacle...')
turnAngle(serPort, turnSpeed, 90);  % Turn 90 degrees counter-clockwise

% Step 3: Resume forward motion briefly to clear the obstacle
disp('Moving forward to clear obstacle...')
travelDist(serPort, 0.2, 0.5);  % Move forward slightly
end
