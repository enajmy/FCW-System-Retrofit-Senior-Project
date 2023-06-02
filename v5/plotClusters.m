close all;
load('IIHS_12MPH_workspace_data.mat');

numObjects = length(simulationData);
allObjectData = cell(numObjects,1);
allClusterData = cell(numObjects,1);

figure;
subplot(1,2,1);
axis equal;
hold on;
theta = linspace(0, 2*pi, 100);
x = cos(theta);
y = sin(theta);
h = fill(x, y, 'g');
hold off;

subplot(1,2,2);
scenarioAxes = gca;
hold on
axis manual
axis([-30 30 -10 80]);

%% Convert objectDetections to usable data
for i = 1:numObjects
    allObjectData{i} = getObjectData(simulationData(i).ObjectDetections);
    allClusterData{i} = [-allObjectData{i}(:,2), allObjectData{i}(:,1)];
end

%% Create clusteringDBSCAN plot
clustererHandle = clusterDBSCAN('EpsilonSource','Property','Epsilon',4,'MinNumPoints',3,'EnableDisambiguation',false);
clusterIndex = clustererHandle(allClusterData{1});
plot(clustererHandle,allClusterData{1},clusterIndex,'Parent',scenarioAxes);
axis([-30 30 -10 80]);
hold on;

pause(1);

%% Advance clustering plot

threatColor = 'green';
for i = 1:numObjects
    clusterIndex = clustererHandle(allClusterData{i});
    plot(clustererHandle,allClusterData{i},clusterIndex,'Parent',scenarioAxes);
    axis([-30 30 0 80]);

    % For each cluster
    clusterLabels = unique(clusterIndex);

    % Remove previous rectangles
    oldRectangles = findobj('Parent',scenarioAxes,'Type', 'rectangle');
    delete(oldRectangles);

    for j = 1:length(clusterLabels)
        % Get the data points for the current cluster
        currentClusterData = allClusterData{i}(clusterIndex == clusterLabels(j), :);

        % Calculate the minimum and maximum x and y coordinates
        minX = min(currentClusterData(:,1));
        maxX = max(currentClusterData(:,1));
        minY = min(currentClusterData(:,2));
        maxY = max(currentClusterData(:,2));

        % Draw a rectangle around the current cluster in the overlaying axes
        rectangle('Parent', scenarioAxes, 'Position',[minX minY maxX-minX maxY-minY],'EdgeColor','r');

        if (clusterLabels(j) == 1)
            threatColor = 'green';
            avgXPos = (maxX-minX)/2;
            if (avgXPos > -3 && avgXPos < 3)
                relSpeed = mean(allObjectData{i}(clusterIndex == 1, 3));
                if (relSpeed < -0.02)
                    %yellow
                    threatColor = 'yellow';
                    d = abs(relSpeed) * 1.2 + (relSpeed*relSpeed) / (2*0.4*9.8);
                    if (minY < d)
                        %warn
                        
                        threatColor = 'red';
                    end
                end
            end
        end
        set(h, 'FaceColor', threatColor);
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
