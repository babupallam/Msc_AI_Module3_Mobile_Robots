function TestProg_1(serPort)
% Robot moves along a path, part blindly, part sensing.
% serPort is the serial port number (used for communication with the robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('==================')
disp('Program Starting')
disp('------------------')

% Step 1: Move forward blindly
% Robot moves forward at 0.5 m/s for a distance of 2 meters.
% This is a "blind" movement, as no sensor data is checked during this step.
travelDist(serPort, .5, 2)  
% Notes:
% - Speed must be within [0.025, 0.5] m/s.
% - Positive distance moves forward, negative moves backward.
% - This command does not monitor obstacles, so walls or objects could block the robot.

% Step 2: Turn counter-clockwise
% Robot turns 90 degrees counter-clockwise at a speed of 0.2 rad/s.
turnAngle(serPort, .2, 90)  
% Notes:
% - Speed must be within [0.025, 0.2] rad/s.
% - Positive angle turns counter-clockwise, negative turns clockwise.

% Step 3: Move forward again
% Robot moves forward at 0.5 m/s for 2.5 meters.
% This is also a blind movement.
travelDist(serPort, .5, 2.5)

% Step 4: Turn counter-clockwise again
% Robot turns another 90 degrees counter-clockwise.
turnAngle(serPort, .2, 90)

% Step 5: Check front sonar sensor
% The robot's front sonar sensor (sensor 2) is used to detect obstacles.
% If no reading is returned, the sonar value is set to 100 meters (out of range).
SonFF = ReadSonar(serPort, 2);  % Front sonar reading
if ~any(SonFF) 
    SonFF = 100;  % Default to a high value if no reading is returned
end

% Step 6: Begin sensor-based movement
% The robot moves forward while continuously monitoring the front sonar sensor.
% It stops when an obstacle is detected within 0.5 meters.
SetDriveWheelsCreate(serPort, 0.4, 0.4)  % Set forward velocity (both wheels at 0.4 m/s).

while (SonFF > 0.5)  % Loop while no obstacle is within 0.5 meters
    pause(.1)  % Short delay to avoid overloading the sensor
    SonFF = ReadSonar(serPort, 2);  % Update the sonar reading
    if ~any(SonFF)
        SonFF = 100;  % Default to a high value if no reading is returned
    end
end

% Display the final sonar reading (distance to the detected obstacle).
disp(SonFF)

% Step 7: Stop the robot
% The robot's wheels are stopped once an obstacle is detected.
SetDriveWheelsCreate(serPort, 0, 0)

end
