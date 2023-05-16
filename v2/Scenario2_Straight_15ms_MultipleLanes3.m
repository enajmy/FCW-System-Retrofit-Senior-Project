function [allData, scenario, sensor] = Scenario2_Straight_15ms_MultipleLanes3()

close all

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
sensor = createSensor(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {});
running = true;

plot(scenario)
hold on;

clusterer = clusterDBSCAN('EpsilonSource','Property','Epsilon',4,'MinNumPoints',3,'EnableDisambiguation',false);

% bep = birdsEyePlot('XLim',[-10 50],'YLim',[-20 20]);
% olPlotter = outlinePlotter(bep);
% caPlotter = coverageAreaPlotter(bep,'DisplayName','Coverage area','FaceColor','blue');
% plotCoverageArea(caPlotter,sensor.MountingLocation(1:2),sensor.RangeLimits(2),sensor.MountingAngles(1),sensor.FieldOfView(1));
% % Create road boundary plotter
% rbPlotter = laneBoundaryPlotter(bep);

while running

    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    time  = scenario.SimulationTime;

    % Generate detections for the sensor
    [objectDetections, isValidTime] = sensor(poses, time);
    numObjects = length(objectDetections);
    objectDetections = objectDetections(1:numObjects);

    if (numObjects)

        objectData = zeros(numObjects, 4);

        for i = 1:numObjects
            objectData(i, :) = [objectDetections{i,1}.Measurement(1)+egoVehicle.Position(1);objectDetections{i,1}.Measurement(2)+egoVehicle.Position(2);objectDetections{i,1}.Measurement(4);objectDetections{i,1}.Measurement(5)];
        end

        clustererPlot = clusterer(objectData(:, 1:2));
        % Get the unique cluster labels
        clusterLabels = unique(clustererPlot);
        % For each cluster
        for j = 1:length(clusterLabels)
            % Get the data points for the current cluster
            currentClusterData = objectData(clustererPlot == clusterLabels(j), :);

            % Calculate the minimum and maximum x and y coordinates
            minX = min(currentClusterData(:,1));
            maxX = max(currentClusterData(:,1));
            minY = min(currentClusterData(:,2));
            maxY = max(currentClusterData(:,2));

            % Draw a rectangle around the current cluster in the overlaying axes
            rectangle('Position',[minX minY maxX-minX maxY-minY],'EdgeColor','r');
        end
    end

    % [position,yaw,long,width,originOffset,color] = targetOutlines(egoVehicle);
    % plotOutline(olPlotter,position,yaw,long,width,'OriginOffset',originOffset,'Color',color);
    % % Plot the road boundaries
    % plotLaneBoundary(rbPlotter,scenario.roadBoundaries);

    % Aggregate all detections into a structure for later use
    if (isValidTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections});
    end

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);

    cla;
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release the sensor object so it can be used again.
release(sensor);

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

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime',4);

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
    'Position', [6.1 6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [6.1 6 0;
    47 6 0];
speed = [22;22];
waittime = [0;0];
trajectory(car2, waypoints, speed, waittime);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [13 -6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [13 -6 0;
    47 -6 0];
speed = [15;15];
waittime = [0;0];
trajectory(car3, waypoints, speed, waittime);

