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


function simpleExample()
    % Find the VREP library
    vrep=remApi('remoteApi');
    % Close all open connections just in case
    vrep.simxFinish(-1);
    % Create a new simualation and connect
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    
    % If a connection exists
    if (clientID>-1)
        disp('Connected')
        %Handle
        % Create the objects to call the motors and sensors, notice the
        % opmode?
        [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
        [returnCode,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);

        %Other Code
        % Start the motor turning
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 1 ,vrep.simx_opmode_blocking);
        % First read of the sonar sensor, again notice the opmode?
        [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);

        
        for i=1:20
            % Read the sonar sensor, opmode type?
           [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
           disp(norm(detectedPoint));
           pause(0.1);
        end
        
        % Stop the motors
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
        
        % Stop the simulation
        vrep.simxFinish(-1);
    
    else
        disp('Failed connecting to remote API server');
    end
    % Call the destructor
    vrep.delete();
end