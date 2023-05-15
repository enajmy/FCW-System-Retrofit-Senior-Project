function [allData, scenario, sensor] = Scenario2_Straight_15ms_MultipleLanes()
%Scenario1_Straight_15ms_NoBarrier - Returns sensor detections
%    allData = Scenario1_Straight_15ms_NoBarrier returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = Scenario1_Straight_15ms_NoBarrier optionally returns
%    the drivingScenario and detection generator objects.

% Close all previous plots and figures
close all

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
sensor = createSensor(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {});
running = true;

pauseSim = true;
pauseTime = 0.01;

clusterer = clusterDBSCAN('EpsilonSource','Property','Epsilon',4,'MinNumPoints',3);

while running

    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    time  = scenario.SimulationTime;

    % Generate detections for the sensor
    [objectDetections, isValidTime] = sensor(poses, time);
    numObjects = length(objectDetections);
    objectDetections = objectDetections(1:numObjects);

    % Aggregate all detections into a structure for later use
    if isValidTime
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections});
    end

    objectData = getObjectData(numObjects, objectDetections);

    if (numObjects)
        
        clustererPlot = clusterer(objectData(:, 1:2));
        plot(clusterer,objectData(:, 1:2),clustererPlot,'Title','Cluster Plot')
        xlim([-10 10])
        ylim([-5 50])

        % clustererPlot2 = clusterer(objectData(:, 1:2));
        % plot(clusterer,objectData(:, 1:2),clustererPlot2,'Title','Cluster Plot')
        % xlim([-100 100])
        % ylim([-5 100])
    end

    if (pauseSim)
        pause(pauseTime)
    end
    
    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release the sensor object so it can be used again.
release(sensor);
end
%% Additional Helper Functions
%% Get ObjectDetections x,y,vx,vy as an array
function [objectData] = getObjectData(numObjects, objectDetections)
objectData = zeros(numObjects, 4);

for i = 1:numObjects
    objectData(i, :) = [objectDetections{i,1}.Measurement(2);objectDetections{i,1}.Measurement(1);objectDetections{i,1}.Measurement(4);objectDetections{i,1}.Measurement(5)];
end

end
%--------------------------------------------------------------------------
%% Setup Bird's Eye Plot
function [olPlotter, radarDetPlotter, tPlotter] = createPlot(x_lim, y_lim, sensor)
bep = birdsEyePlot('XLim',x_lim,'YLim',y_lim);
olPlotter = outlinePlotter(bep);
caPlotter = coverageAreaPlotter(bep,'DisplayName','Coverage area','FaceColor','blue');
tPlotter = trackPlotter(bep,'DisplayName','Tracks','LabelOffset',[3 0], 'VelocityScaling', 0.1);
radarDetPlotter = detectionPlotter(bep, 'DisplayName','radar detection', ...
    'MarkerEdgeColor','red');
plotCoverageArea(caPlotter,sensor.MountingLocation(1:2),sensor.RangeLimits(2),sensor.MountingAngles(1),sensor.FieldOfView(1));
end
%--------------------------------------------------------------------------
%% Plot Objects on Bird's Eye Plot
function  [velocities, positions, cluttervel, clutterpos] = plotObjects(numObjects, realRadarObjects, clutterRadarObjects, egoVehicle, olPlotter, radarDetPlotter, tPlotter, velocities, positions, cluttervel, clutterpos)
[position,yaw,long,width,originOffset,color] = targetOutlines(egoVehicle);
plotOutline(olPlotter,position,yaw,long,width,'OriginOffset',originOffset,'Color',color);

[velocities, positions] = getObjectPosVel(realRadarObjects, numObjects, velocities, positions);
[cluttervel, clutterpos] = getObjectPosVel(clutterRadarObjects, length(clutterRadarObjects), cluttervel, clutterpos);
plotTrack(tPlotter, clutterpos, cluttervel);
plotDetection(radarDetPlotter,positions,velocities);
end
%-------------------------------------------------------------------------
%% Get Object's Velocities and Positions
function [velocities, positions] = getObjectPosVel(realRadarObjects, numObjects, velocities, positions)
%     velocities = zeros(numObjects, 2);
%     positions = zeros(numObjects, 2);

for i = 1:numObjects
    position_x = realRadarObjects{i,1}.Measurement(1);
    position_y = realRadarObjects{i,1}.Measurement(2);
    positions(i,1) = position_x;
    positions(i,2) = position_y;
    velocitiy_x = realRadarObjects{i,1}.Measurement(4);
    velocitiy_y = realRadarObjects{i,1}.Measurement(5);
    velocities(i,1) = velocitiy_x;
    velocities(i,2) = velocitiy_y;
end
end

%--------------------------------------------------------------------------
%% Calculate Object Speed
function [vx, vy] = calculateObjectSpeed(vxi, vyi, egoSpeed)
vx = vxi + egoSpeed;
theta = atan2(vyi, vxi);
vy = vx * tan(theta);
end

%--------------------------------------------------------------------------
%% Remove Clutter Detections
function [realRadarObjects, clutterRadarObjects] = removeClutter(objectDetections, numObjects, egoSpeed)
normVs = zeros(numObjects, 1);
inLane = zeros(numObjects, 1);
inZone = zeros(numObjects, 1);
inYSpeed = zeros(numObjects, 1);

LaneWidth = 3.4;
ZoneWidth = 1.7*LaneWidth;
minV = 3;
maxV = 13;
maxVy = 1;
for i = 1:numObjects
    [vx, vy] = calculateObjectSpeed(objectDetections{i,1}.Measurement(4),objectDetections{i,1}.Measurement(5), egoSpeed);
    normVs(i) = norm([vx,vy]);
    inLane(i) = (abs(objectDetections{i,1}.Measurement(2)) <= LaneWidth/2);
    inZone(i) = (abs(objectDetections{i,1}.Measurement(2)) <= max(abs(vy)*2, ZoneWidth));
    inYSpeed(i) = (abs(objectDetections{i,1}.Measurement(5)) <= maxVy);
end
realRadarObjectsIdx = intersect(find((inYSpeed == 1) & (normVs < maxV) & (normVs > minV)), find(inLane == 1));
clutterObjectsIdx = union(union(find(normVs < minV), find(inLane == 0)), union(find(normVs > maxV), find(inYSpeed > maxVy)));
% realRadarObjectsIdx = union(...
%intersect(find(normVs > minV), find(inZone == 1)), ...
%find(inLane == 1));

realRadarObjects = objectDetections(realRadarObjectsIdx);
clutterRadarObjects = objectDetections(clutterObjectsIdx);
end
%--------------------------------------------------------------------------
%% Most Important Object
function mostImportantObject = findMostImportantObject(realRadarObjects, numObjects, positions, velocities)
% Initialize outputs and parameters
FCW = 0;                % By default, if there is no MIO, then FCW is 'safe'
threatColor = 'green';
LaneWidth = 3.6;
maxX = 1000;  % Far enough forward so that no track is expected to exceed this distance
gAccel = 9.8; % Constant gravity acceleration, in m/s^2
maxDeceleration = 0.4 * gAccel; % Euro NCAP AEB definition
delayTime = 1.2; % Delay time for a driver before starting to brake, in seconds
x = 0;
y = 0;
v = 0;

[velocities, positions] = getObjectPosVel(realRadarObjects, numObjects, velocities, positions);

for i = 1:numObjects
    x = x + positions(i,1);
    y = y + positions(i,2);
    v = v + velocities(i, 1);
end

x = x / numObjects;
y = y / numObjects;
relSpeed = v / numObjects; % The relative speed between the cars, along the lane

if x < maxX && x > 0 % No point checking otherwise
    if (y <= LaneWidth/2)
        if relSpeed < -0.02 % Relative speed indicates object is getting closer
            % Calculate expected braking distance according to
            % Euro NCAP AEB Test Protocol
            d = abs(relSpeed) * delayTime + (relSpeed*relSpeed) / 2 / maxDeceleration;
            if x <= d % 'warn'
                FCW = 1;
                threatColor = 'red';
            else
                FCW = 0;
                threatColor = 'yellow';
            end
        end
    end
end
mostImportantObject = struct('Warning', FCW, 'ThreatColor', threatColor, 'relSpeed', relSpeed);
end
%--------------------------------------------------------------------------
%% Plot Warning
function h = plotCircle()
figure;
axis equal;
hold on;
theta = linspace(0, 2*pi, 100);
x = cos(theta);
y = sin(theta);
h = fill(x, y, 'r');
end

% Function to change the circle color
function change_color(h, new_color)
set(h, 'FaceColor', new_color);
end
%-------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function sensor = createSensor(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensor = drivingRadarDataGenerator('SensorIndex', 1, ...
    'MountingLocation', [3.7 0 0.2], ...
    'RangeLimits', [0 100], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [120 5], ...
    'Profiles', profiles);
end

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [50 0 0;
    0 0 0];
laneSpecification = lanespec(3, 'Width', 6);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [2 0 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [2 0 0;
    3 0 0;
    40 0 0];
speed = [0;15;15];
waittime = [0.5;0;0];
trajectory(egoVehicle, waypoints, speed, waittime);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [8 0 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [8 0 0;
    40 0 0;
    45 0 0];
speed = [15;15;0];
waittime = [0;0;0];
trajectory(car1, waypoints, speed, waittime);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [6.5 6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [6.5 6 0;
    47 6 0];
speed = [20;20];
waittime = [0;0];
trajectory(car2, waypoints, speed, waittime);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [12 -6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [12 -6 0;
    47 -6 0];
speed = [15;15];
waittime = [0;0];
trajectory(car3, waypoints, speed, waittime);
end