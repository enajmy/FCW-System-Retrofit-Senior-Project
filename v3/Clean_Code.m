%% Main Function

% Close all previous displays
close all

% Generate data
[simulationData, scenarioData, sensorData] = generateDrivingData();

% ---- DEFINE VARIABLES ---------------------------------------------------
% bepLimits - used for x & y axis limits on Birds Eye Plot ----------------
%       -> bepLimits(1:2) = lower x & upper x limits
%       -> bepLimits(3:4) = lower y & upper y limits
bepLimits = [-45, 45, -25, 25];
% cluster<x/y>Lim - used for x & y axis limits on Cluster Plot ------------
%       -> clusterXLim = lower x & upper x limits
%       -> clusterYLim = lower y & upper y limits
clusterXLim = [-30 30];
clusterYLim = [-10 80];
% egoVehicle - used to represent the egoVehicle of the simulation ---------
%       -> formatted as a struct used by Driving Scenario Designer
egoVehicle = scenarioData.Actors(1);

% ---- BEGIN CODE ---------------------------------------------------------
% Plotting using Birds Eye Plot method
plotterDataStruct = createBirdsEye(sensorData, bepLimits);

% Plotting using scenario(plot) method
plot(scenarioData);

% Plotting for clusters
figure();
hold on
clusterAxesHandle = axes();
xlim(clusterXLim);
ylim(clusterYLim);

% Create clusteringDBSCAN object
clusterer = clusterDBSCAN('EpsilonSource','Property','Epsilon',4,'MinNumPoints',3,'EnableDisambiguation',false);

for i = 1:(scenarioData.StopTime/scenarioData.SampleTime)

    % Check to see if it is time to update the sensor position
    % Needed because sensor updates every 100ms but sim updates every 10ms,
    % therefore we have 10x more sim time steps than sensor steps
    if mod(i, 10) == 1
        % Determine current index from i value
        index = ((i - 1) / 10) + 1;

        % Get object positions and velocities
        objectData = getObjectData(simulationData(index).ObjectDetections);
        
        % Plot on normal Driving Scenario Designer Plot
        clearData(plotterDataStruct.radarDetPlotter);
        plotDetection(plotterDataStruct.radarDetPlotter,objectData(:, 1:2),objectData(:, 3:4));

        % Swap x & y columns and negate y values for plotting clusters
        objectData(:, [1, 2]) = [-objectData(:, 2), objectData(:, 1)];

        hold on
        % Create cluster plot object
        clustererPlot = clusterer(objectData(:, 1:2));
        
        % Plot data points with clusterDBSCAN plot function
        plot(clusterer,objectData(:, 1:2),clustererPlot,'Parent',clusterAxesHandle);
        xlim(clusterAxesHandle, clusterXLim);
        ylim(clusterAxesHandle, clusterYLim);
     
        % Get the unique cluster labels
        clusterLabels = unique(clustererPlot);

        % Remove previous rectangles
        oldRectangles = findobj('Parent',clusterAxesHandle,'Type', 'rectangle');
        delete(oldRectangles);

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
            rectangle('Parent', clusterAxesHandle, 'Position',[minX minY maxX-minX maxY-minY],'EdgeColor','r');
        end
        hold off
    end

    % Plot the vehicles
    [position,yaw,long,width,originOffset,color] = targetOutlines(egoVehicle);
    plotOutline(plotterDataStruct.olPlotter,position,yaw,long,width,'OriginOffset',originOffset,'Color',color);

    % Plot the lane boundaries/lines
    [lmv,lmf] = laneMarkingVertices(egoVehicle);
    plotLaneMarking(plotterDataStruct.lmPlotter,lmv,lmf)
    rbEgoVehicle = roadBoundaries(egoVehicle);
    plotLaneBoundary(plotterDataStruct.lbPlotter,rbEgoVehicle)

    % Pause the simulation to make visible on plots
    pause(0.005)

    % Advance scenario time point
    advance(scenarioData);
end

%% Function Used to Get Objects Positions and Velocities
function [objectData] = getObjectData(objectDetections)
    % Get number of objects detected for array length
    numObjects = length(objectDetections);

    % Initialize an array of zeros to hold x,y,vx,vy
    objectData = zeros(numObjects,4);

    % Grab values and store in objectData
    % objectData is formated where (i, 1) = x,
    % (i, 2) = y, (i, 3) = vx, and (i, 4) = vy
    for j = 1:numObjects
        objectData(j, :) = [objectDetections{j,1}.Measurement(1);
            objectDetections{j,1}.Measurement(2);
            objectDetections{j,1}.Measurement(4);
            objectDetections{j,1}.Measurement(5)];
    end
end
%% Function Used to Create a Birds Eye Plot
function [plotterDataStruct] = createBirdsEye(sensor, bepLimits)

%Create birds eye plot
bep = birdsEyePlot('XLim',bepLimits(1:2),'YLim',bepLimits(3:4));

%Create coverage area plotter
caPlotter = coverageAreaPlotter(bep, 'DisplayName','Radar Coverage Area','FaceColor','b');
plotCoverageArea(caPlotter,sensor.MountingLocation(1:2),sensor.RangeLimits(2),sensor.MountingAngles(1),sensor.FieldOfView(1));

%Create road boundaries plotter
lbPlotter = laneBoundaryPlotter(bep,'DisplayName','Road');
lmPlotter = laneMarkingPlotter(bep);

%Create vehicle plotter
olPlotter = outlinePlotter(bep);

%Create radar detections plotter
radarDetPlotter = detectionPlotter(bep, 'DisplayName','Radar Detection', 'MarkerEdgeColor','red');

plotterDataStruct = struct('BEP', bep, ...
    'caPlotter', caPlotter, ...
    'lbPlotter', lbPlotter, ...
    'lmPlotter', lmPlotter, ...
    'olPlotter', olPlotter, ...
    'radarDetPlotter', radarDetPlotter);

end
%% DRIVING SCENARIO DESIGNER FUNCTIONS BELOW =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#
% =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
%% Function Used to Generate Driving Data Based Off Below Scenarios -------
function [allData, scenario, sensor] = generateDrivingData()

[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
sensor = createSensor(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {});
running = true;
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
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release the sensor object so it can be used again.
release(sensor);
end
%% Function Used to Create Radar Sensor -----------------------------------
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
%% Function Used to Create the Driving Scenario ---------------------------
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
end