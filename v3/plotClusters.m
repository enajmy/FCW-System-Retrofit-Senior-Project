close all;
load('curvedBarriers.mat');

numObjects = length(simulationData);
allObjectData = cell(numObjects,1);
allClusterData = cell(numObjects,1);

scenarioFigure = figure;
scenarioAxes = axes;
hold on
axis manual
axis([-30 30 -10 80]);

%% Convert objectDetections to usable data
for i = 1:numObjects
    allObjectData{i} = getObjectData(simulationData(i).ObjectDetections);
    allClusterData{i} = [-allObjectData{i}(:,2), allObjectData{i}(:,1)];
end

%% Create clusteringDBSCAN plot
clusterer = clusterDBSCAN('EpsilonSource','Property','Epsilon',4,'MinNumPoints',3,'EnableDisambiguation',false);
clusterHandle = clusterer(allClusterData{1});
plot(clusterer,allClusterData{1},clusterHandle,'Parent',scenarioAxes);
axis([-30 30 -10 80]);
hold on;

%% Advance clustering plot
for i = 1:numObjects
    clusterHandle = clusterer(allClusterData{i});
    plot(clusterer,allClusterData{i},clusterHandle,'Parent',scenarioAxes);
    axis([-30 30 -10 80]);

    % For each cluster
    clusterLabels = unique(clusterHandle);

    % Remove previous rectangles
    oldRectangles = findobj('Parent',scenarioAxes,'Type', 'rectangle');
    delete(oldRectangles);

    for j = 1:length(clusterLabels)
        % Get the data points for the current cluster
        currentClusterData = allClusterData{i}(clusterHandle == clusterLabels(j), :);

        % Calculate the minimum and maximum x and y coordinates
        minX = min(currentClusterData(:,1));
        maxX = max(currentClusterData(:,1));
        minY = min(currentClusterData(:,2));
        maxY = max(currentClusterData(:,2));

        % Draw a rectangle around the current cluster in the overlaying axes
        rectangle('Parent', scenarioAxes, 'Position',[minX minY maxX-minX maxY-minY],'EdgeColor','r');

        pause(0.05);
    end
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
