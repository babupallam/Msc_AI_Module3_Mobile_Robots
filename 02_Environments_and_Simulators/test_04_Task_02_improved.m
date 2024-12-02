function reactiveRobot()
    % Initialization
    vrep = initializeVREP();

    try
        % Establish connection to V-REP
        clientID = connectToVREP(vrep);
        
        if clientID > -1
            disp('Connected to V-REP remote API server');
            
            % Initialize robot components
            handles = getRobotHandles(vrep, clientID);

            % Configure motion parameters
            params = configureMotionParams();

            % Start reactive behavior
            runReactiveMotion(vrep, clientID, handles, params);
        else
            error('Failed connecting to V-REP remote API server');
        end
    catch ME
        disp(['Error: ', ME.message]);
    end

    % Cleanup
    cleanupVREP(vrep, clientID);
end

% Function to initialize V-REP remote API
function vrep = initializeVREP()
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1); % Close any previous connections
end

% Function to connect to V-REP server
function clientID = connectToVREP(vrep)
    clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
end

% Function to retrieve robot handles
function handles = getRobotHandles(vrep, clientID)
    handles = struct();
    [~, handles.leftMotor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking);
    [~, handles.rightMotor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking);
    [~, handles.frontSensor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5', vrep.simx_opmode_blocking);

    % Initialize proximity sensor streaming
    vrep.simxReadProximitySensor(clientID, handles.frontSensor, vrep.simx_opmode_streaming);
    pause(0.1); % Allow streaming to initialize
end

% Function to define motion parameters
function params = configureMotionParams()
    params = struct();
    params.moveSpeed = 10;          % Constant forward speed
    params.objectDetectedDist = 1.0; % Distance to detect object (in meters)
    params.stopDist = 0.5;         % Distance to stop (in meters)
    params.safetyDist = 0.3;       % Safety buffer to ensure no contact (in meters)
    params.turnSpeed = 0.5;        % Speed during turning
    params.turnMinTime = 2;        % Minimum turn duration (seconds)
    params.turnMaxTime = 5;        % Maximum turn duration (seconds)
end

% Function to execute reactive motion
function runReactiveMotion(vrep, clientID, handles, params)
    while true
        % Read distance from proximity sensor
        dist = readSensorDistance(vrep, clientID, handles.frontSensor);

        if dist > 0 && dist < params.objectDetectedDist
            disp(['Object detected at distance: ', num2str(dist)]);
            
            % Stop the robot if within stopping distance or safety distance
            if dist < params.safetyDist
                disp('Stopping to avoid collision. Object too close.');
                stopRobot(vrep, clientID, handles);
                avoidObstacle(vrep, clientID, handles, params);
            elseif dist < params.stopDist
                disp('Stopping for safety. Preparing to avoid.');
                stopRobot(vrep, clientID, handles);
                avoidObstacle(vrep, clientID, handles, params);
            end
        else
            moveForward(vrep, clientID, handles, params.moveSpeed);
        end
        
        pause(0.1); % Delay for sensor updates
    end
end

% Function to read distance from proximity sensor
function dist = readSensorDistance(vrep, clientID, sensor)
    [~, detectionState, detectedPoint, ~, ~] = vrep.simxReadProximitySensor(clientID, sensor, vrep.simx_opmode_buffer);
    if detectionState
        dist = norm(detectedPoint); % Distance to detected object
    else
        dist = inf; % No object detected
    end
end

% Function to move the robot forward
function moveForward(vrep, clientID, handles, speed)
    vrep.simxSetJointTargetVelocity(clientID, handles.leftMotor, speed, vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID, handles.rightMotor, speed, vrep.simx_opmode_blocking);
end

% Function to stop the robot
function stopRobot(vrep, clientID, handles)
    vrep.simxSetJointTargetVelocity(clientID, handles.leftMotor, 0, vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID, handles.rightMotor, 0, vrep.simx_opmode_blocking);
end

% Function to avoid an obstacle by turning randomly
function avoidObstacle(vrep, clientID, handles, params)
    turnDuration = params.turnMinTime + rand() * (params.turnMaxTime - params.turnMinTime);
    turnDirection = sign(randn()); % Randomly choose -1 (left) or 1 (right)

    disp(['Turning ', num2str(turnDuration), ' seconds in direction: ', num2str(turnDirection)]);
    vrep.simxSetJointTargetVelocity(clientID, handles.leftMotor, turnDirection * params.turnSpeed, vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID, handles.rightMotor, -turnDirection * params.turnSpeed, vrep.simx_opmode_blocking);
    pause(turnDuration);

    disp('Avoidance complete. Resuming motion.');
end

% Function to cleanup V-REP connection
function cleanupVREP(vrep, clientID)
    if clientID > -1
        vrep.simxFinish(clientID);
    end
    vrep.delete();
end
