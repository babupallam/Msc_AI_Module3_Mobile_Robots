% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function reactiveMotion()
    % Step 1: Find the V-REP library
    vrep = remApi('remoteApi');
    % Step 2: Close all open connections just in case
    vrep.simxFinish(-1);
    % Step 3: Create a new simulation and connect
    clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

    % Step 4: Check if a connection exists
    if (clientID > -1)
        disp('Connected to V-REP remote API server');
        
        % Step 5: Get handles for motors and sensors
        [returnCode, left_Motor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking);
        [returnCode, right_Motor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking);
        [returnCode, front_Sensor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5', vrep.simx_opmode_blocking);

        % Step 6: Start streaming the proximity sensor data
        [returnCode, detectionState, detectedPoint, ~, ~] = vrep.simxReadProximitySensor(clientID, front_Sensor, vrep.simx_opmode_streaming);
        pause(0.1); % Allow streaming to initialize

        % Step 7: Define motion parameters
        moveSpeed = 20;       % Constant forward speed
        objectDetectedDist = 1.0; % Distance to detect object (in meters)
        stopDist = 0.5;      % Distance to stop (in meters)

        % Step 8: Main reactive loop
        while true
            % Step 8.1: Read the sensor data
            [returnCode, detectionState, detectedPoint, ~, ~] = vrep.simxReadProximitySensor(clientID, front_Sensor, vrep.simx_opmode_buffer);
            dist = norm(detectedPoint); % Compute distance to detected object

            % Step 8.2: Check if an object is within detection range
            if detectionState && dist < objectDetectedDist
                disp(['Object detected at distance: ', num2str(dist)]);
                
                % Step 8.3: Stop if the object is within stop distance
                if dist < stopDist
                    disp('Stopping as object is too close.');
                    vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0, vrep.simx_opmode_blocking);
                    vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0, vrep.simx_opmode_blocking);
                    
                    % Step 8.4: Turn randomly in a direction
                    turnSpeed = 0.9; % Speed for turning
                    turnDuration = randi([1, 3]); % Random turn duration in seconds
                    direction = sign(randn()); % Random direction (-1 for left, 1 for right)
                    disp(['Turning ', num2str(turnDuration), ' seconds in direction: ', num2str(direction)]);
                    vrep.simxSetJointTargetVelocity(clientID, left_Motor, direction * turnSpeed, vrep.simx_opmode_blocking);
                    vrep.simxSetJointTargetVelocity(clientID, right_Motor, -direction * turnSpeed, vrep.simx_opmode_blocking);
                    pause(turnDuration);

                    % Step 8.5: Resume forward motion
                    disp('Resuming forward motion.');
                    vrep.simxSetJointTargetVelocity(clientID, left_Motor, moveSpeed, vrep.simx_opmode_blocking);
                    vrep.simxSetJointTargetVelocity(clientID, right_Motor, moveSpeed, vrep.simx_opmode_blocking);
                end
            else
                % Step 8.6: Move forward at constant speed
                vrep.simxSetJointTargetVelocity(clientID, left_Motor, moveSpeed, vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID, right_Motor, moveSpeed, vrep.simx_opmode_blocking);
            end

            pause(0.1); % Delay for sensor data update
        end

        % Step 9: Stop the simulation (never reached in this loop)
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    
    % Step 10: Clean up and delete the API object
    vrep.delete();
end
