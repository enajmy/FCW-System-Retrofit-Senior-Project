function [allData, scenario, sensor] = Scenario()

[scenario, egoVehicle] = createDrivingScenario;
sensor = createSensor(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {});
running = true;

%%% ---- Need these variables for code to work ---- %%%
pauseSim = true;
pauseTime = 0.1;

xLimits = [-10 30];
yLimits = [-15 15];

positions = [0 0];
velocities = [0 0];
%%% ----------------------------------------------- %%%

%%% -------- Plot Creation Function -------- %%%
[olPlotter, radarDetPlotter] = createPlot(xLimits, yLimits, sensor);


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

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);

    
    %%% -------- Plot Objects Function -------- %%%
    [positions, velocities] = plotObjects(numObjects, objectDetections, egoVehicle, olPlotter, radarDetPlotter, positions, velocities);
    
    %%% -------- Used to Slow Sim Time -------- %%%
    if (pauseSim)
        pause(pauseTime)
    end

end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release the sensor object so it can be used again.
release(sensor);

%% Helper Functions

function [olPlotter, radarDetPlotter] = createPlot(x_lim, y_lim, sensor)
    % Create plot
    bep = birdsEyePlot('XLim',x_lim,'YLim',y_lim);
    olPlotter = outlinePlotter(bep);
    % Create radar coverage outline
    caPlotter = coverageAreaPlotter(bep,'DisplayName','Coverage area','FaceColor','blue');
    % Combine all radar detections into one entry and store it for later update
    radarDetPlotter = detectionPlotter(bep, 'DisplayName','radar detection', ...
        'MarkerEdgeColor','red');
    plotCoverageArea(caPlotter,sensor.MountingLocation(1:2),sensor.RangeLimits(2),sensor.MountingAngles(1),sensor.FieldOfView(1));


function [positions, velocities] = plotObjects(numObjects, objectDetections, egoVehicle, olPlotter, radarDetPlotter, positions, velocities)
    [position,yaw,long,width,originOffset,color] = targetOutlines(egoVehicle);
    plotOutline(olPlotter,position,yaw,long,width, ...
        'OriginOffset',originOffset,'Color',color);

    for i = 1:numObjects
        position_x = objectDetections{i,1}.Measurement(1);
        position_y = objectDetections{i,1}.Measurement(2);
        positions(i,1) = position_x;
        positions(i,2) = position_y;
        velocitiy_x = objectDetections{i,1}.Measurement(4);
        velocitiy_y = objectDetections{i,1}.Measurement(5);
        velocities(i,1) = velocitiy_x;
        velocities(i,2) = velocitiy_y;
    end
    plotDetection(radarDetPlotter,positions,velocities);



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

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [50 0 0;
    0 0 0];
roadWidth = 3.6;
road(scenario, roadCenters, roadWidth, 'Name', 'Road');

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
waittime = [0.1;0;0];
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

