function test(serPort)
% Experimental robot navigation with enhanced logic and experiments.
% serPort is the serial port number for robot communication.

disp('========================')
disp('Experimentation Starting')
disp('------------------------')

% Step 1: Experiment with different speeds and distances
safeDistances = [0.3, 0.5, 0.7];  % Experimenting with safe distances
speeds = [0.2, 0.4, 0.6];         % Experimenting with speeds

% Step 2: Loop through experimental parameters
for safeDistance = safeDistances
    for speed = speeds
        fprintf('Testing with safeDistance = %.2f and speed = %.2f\n', safeDistance, speed);
        % Test basic navigation with varying parameters
        navigateWithSensorsExperiment(serPort, speed, 2, safeDistance); % Move forward safely
        turnWithSensorsExperiment(serPort, 0.3, 90);                    % Test a 90-degree turn
    end
end

% Step 3: Experiment with complex obstacle scenarios
disp('Starting obstacle course experiment...')
obstacleCourseExperiment(serPort);

disp('Experimentation Completed Successfully')
disp('======================================')

end

%% Function: Navigate forward safely with experimental parameters
function navigateWithSensorsExperiment(serPort, speed, distance, safeDistance)
% Moves the robot forward while experimenting with different parameters.

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
        fprintf('Obstacle detected at %.2f meters! Avoiding...\n', frontDistance);
        SetDriveWheelsCreate(serPort, 0, 0);  % Stop
        avoidObstacleExperiment(serPort, safeDistance);  % Call obstacle avoidance logic
    end
    
    pause(0.1);  % Short delay to prevent sensor overload
    currentDistance = currentDistance + speed * 0.1;  % Approximate travel distance
end

SetDriveWheelsCreate(serPort, 0, 0);  % Stop once target distance is reached
end

%% Function: Turn the robot safely during experiments
function turnWithSensorsExperiment(serPort, turnSpeed, angle)
% Turns the robot with experimental logging.
fprintf('Turning robot by %d degrees...\n', angle);
turnAngle(serPort, turnSpeed, angle);  % Execute the turn command
end

%% Function: Obstacle avoidance with experimental adjustments
function avoidObstacleExperiment(serPort, safeDistance)
% Enhanced obstacle avoidance logic for experiments.

turnSpeed = 0.2;        % Speed for turning
backupDistance = 0.3;   % Distance to back up (in meters)

% Step 1: Back up slightly
disp('Backing up during obstacle avoidance...')
travelDist(serPort, -0.2, -backupDistance);  % Move backward

% Step 2: Experiment with turn angles for better avoidance
turnAngles = [45, 90, 135];  % Testing different angles
for angle = turnAngles
    fprintf('Testing avoidance turn with %d degrees...\n', angle);
    turnAngle(serPort, turnSpeed, angle);  % Turn robot by specified angle
end

% Step 3: Resume forward motion briefly to clear the obstacle
disp('Moving forward to clear obstacle...')
travelDist(serPort, 0.2, 0.5);  % Move forward slightly
end

%% Function: Obstacle course experiment
function obstacleCourseExperiment(serPort)
% Navigates a simulated obstacle course.

% Sequence of movements and obstacle avoidance
navigateWithSensorsExperiment(serPort, 0.3, 1, 0.5);  % Forward 1m
turnWithSensorsExperiment(serPort, 0.2, -90);         % Turn 90 degrees clockwise
navigateWithSensorsExperiment(serPort, 0.3, 1.5, 0.4); % Forward 1.5m
turnWithSensorsExperiment(serPort, 0.2, 90);          % Turn 90 degrees counter-clockwise
navigateWithSensorsExperiment(serPort, 0.3, 1, 0.5);  % Forward 1m
end
