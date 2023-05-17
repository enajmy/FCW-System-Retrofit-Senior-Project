close all;
load('curvedBarriers.mat');

egoVehicle = scenarioData.Actors(1);
numObjects = length(simulationData);
allObjectData = cell(numObjects,1);

%% Initialize birds eye plot
bepFig = figure;
bepAx = axes;
hold on;
plotterDataStruct = createBirdsEye(sensorData, [-40 40 -20 20], bepAx);

%% Convert objectDetections to usable data
for i = 1:numObjects
    allObjectData{i} = getObjectData(simulationData(i).ObjectDetections);
end

%% Run Simulation
for i = 1:numObjects
    plotDetection(plotterDataStruct.radarDetPlotter,allObjectData{i}(:,1:2),allObjectData{i}(:,3:4));

    for j = 1:10
    % Plot the vehicles
    [position,yaw,long,width,originOffset,color] = targetOutlines(egoVehicle);
    plotOutline(plotterDataStruct.olPlotter,position,yaw,long,width,'OriginOffset',originOffset,'Color',color);

    % Plot the lane boundaries/lines
    [lmv,lmf] = laneMarkingVertices(egoVehicle);
    plotLaneMarking(plotterDataStruct.lmPlotter,lmv,lmf)
    rbEgoVehicle = roadBoundaries(egoVehicle);
    plotLaneBoundary(plotterDataStruct.lbPlotter,rbEgoVehicle)

    advance(scenarioData);
    pause(0.01);
    end

end

%% Function Used to Create a Birds Eye Plot
function [plotterDataStruct] = createBirdsEye(sensor, bepLimits, axes)

%Create birds eye plot
bep = birdsEyePlot('XLim',bepLimits(1:2),'YLim',bepLimits(3:4),'Parent',axes);

%Create coverage area plotter
caPlotter = coverageAreaPlotter(bep, 'DisplayName','Radar Coverage Area','FaceColor','b');
plotCoverageArea(caPlotter,sensor.MountingLocation(1:2),sensor.RangeLimits(2),sensor.MountingAngles(1),sensor.FieldOfView(1));

%Create road boundaries plotter
lbPlotter = laneBoundaryPlotter(bep);
lmPlotter = laneMarkingPlotter(bep,'DisplayName','Road');

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

%% Get other functions
function [objectData] = getObjectData(objectDetections)
    % Get number of objects detected for array length
    numDetections = length(objectDetections);

    % Initialize an array of zeros to hold x,y,vx,vy
    objectData = zeros(numDetections,4);

    % Grab values and store in objectData
    % objectData is formated where (i, 1) = x,
    % (i, 2) = y, (i, 3) = vx, and (i, 4) = vy
    for j = 1:numDetections
        objectData(j, :) = [objectDetections{j,1}.Measurement(1);
            objectDetections{j,1}.Measurement(2);
            objectDetections{j,1}.Measurement(4);
            objectDetections{j,1}.Measurement(5)];
    end
end