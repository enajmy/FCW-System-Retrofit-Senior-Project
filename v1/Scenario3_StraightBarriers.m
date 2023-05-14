function [allData, scenario, sensor] = Scenario3_StraightBarriers()

close all

[scenario, egoVehicle] = createDrivingScenario;
sensor = createSensor(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {});
running = true;

pauseSim = true;
pauseTime = 0.01;

xLimits = [-10 30];
yLimits = [-15 15];
velocities = [0 0];
positions = [0 0];
cluttervel = [0 0];
clutterpos = [0 0];

[olPlotter, radarDetPlotter, tPlotter] = createPlot(xLimits, yLimits, sensor);
h = plotCircle();

while running

    poses = targetPoses(egoVehicle);
    time  = scenario.SimulationTime;

    [objectDetections, isValidTime] = sensor(poses, time);
    numObjects = length(objectDetections);
    objectDetections = objectDetections(1:numObjects);

    if isValidTime
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections});
    end

    [realRadarObjects, clutterRadarObjects] = removeClutter(objectDetections, numObjects, egoVehicle.Velocity(1));
    mostImportantObject = findMostImportantObject(realRadarObjects, length(realRadarObjects), positions, velocities);
    
    running = advance(scenario);
    [velocities, positions, cluttervel, clutterpos] = plotObjects(length(realRadarObjects), realRadarObjects, clutterRadarObjects, egoVehicle, olPlotter, radarDetPlotter, tPlotter, velocities, positions, cluttervel, clutterpos);
    if isValidTime
        change_color(h, mostImportantObject.ThreatColor)
        %fprintf('Color: %s\n', FCWcolor)
        %fprintf('RelSpeed: %.2f\n', relSpeed)

    end

    if (pauseSim)
        pause(pauseTime)
    end
end

restart(scenario);
release(sensor);

end
%% Additional Helper Functions
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

    LaneWidth = 3.4;
    ZoneWidth = 1.7*LaneWidth;
    minV = 3;
    for i = 1:numObjects
        [vx, vy] = calculateObjectSpeed(objectDetections{i,1}.Measurement(4),objectDetections{i,1}.Measurement(5), egoSpeed);
        normVs(i) = norm([vx,vy]);
        inLane(i) = (abs(objectDetections{i,1}.Measurement(2)) <= LaneWidth/2);
        inZone(i) = (abs(objectDetections{i,1}.Measurement(2)) <= max(abs(vy)*2, ZoneWidth));
    end
    realRadarObjectsIdx = intersect(find(normVs > minV), find(inLane == 1));
    clutterObjectsIdx = union(find(normVs < minV), find(inLane == 0));
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
%% Prebuilt Helper Functions
function sensor = createSensor(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensor = drivingRadarDataGenerator('SensorIndex', 1, ...
    'MountingLocation', [3.7 0 0.2], ...
    'RangeLimits', [0 100], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [45 5], ...
    'Profiles', profiles);
end

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [50 0 0;
    0 0 0];
roadWidth = 3.6;
road1 = road(scenario, roadCenters, roadWidth, 'Name', 'Road');

% Add the barriers
barrier(scenario, road1, 'RoadEdge', 'right', ...
    'ClassID', 6, ...
    'Width', 0.433, ...
    'Mesh', driving.scenario.guardrailMesh, 'PlotColor', [0.55 0.55 0.55], 'Name', 'Guardrail');

barrier(scenario, road1, 'RoadEdge', 'left', ...
    'ClassID', 6, ...
    'Width', 0.433, ...
    'Mesh', driving.scenario.guardrailMesh, 'PlotColor', [0.55 0.55 0.55], 'Name', 'Guardrail1');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [3 0 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [3 0 0;
    5 0 0;
    46 0 0];
speed = [0;15;15];
waittime = [0.3;0;0];
trajectory(egoVehicle, waypoints, speed, waittime);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [9 0 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [9 0 0;
    40 0 0;
    46 0 0];
speed = [15;15;0];
waittime = [0;0;0];
trajectory(car1, waypoints, speed, waittime);
end
