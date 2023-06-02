close all;
load('IIHS_25MPH_workspace_data.mat');

egoVehicle = scenarioData.Actors(1);
numObjects = length(simulationData);
allObjectData = cell(numObjects,1);
allClusterData = cell(numObjects,1);

figure;
subplot(2,2,4);
axis equal;
hold on;
theta = linspace(0, 2*pi, 100);
x = cos(theta);
y = sin(theta);
h = fill(x, y, 'g');
hold off;

%% Initialize birds eye plot
subplot(2,2,[1,3])
bepAx = gca;
legend('boxoff')
hold on;
plotterDataStruct = createBirdsEye(sensorData, [-40 40 -20 20], bepAx);
hold off;

subplot(2,2,2);
scenarioAxes = gca;
hold on
axis manual
axis([-30 30 -10 60]);


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

    plotDetection(plotterDataStruct.radarDetPlotter,allObjectData{i}(:,1:2),allObjectData{i}(:,3:4));

    for k = 1:10
    % Plot the vehicles
    [position,yaw,long,width,originOffset,color] = targetOutlines(egoVehicle);
    plotOutline(plotterDataStruct.olPlotter,position,yaw,long,width,'OriginOffset',originOffset,'Color',color);

    % Plot the lane boundaries/lines
    [lmv,lmf] = laneMarkingVertices(egoVehicle);
    plotLaneMarking(plotterDataStruct.lmPlotter,lmv,lmf)
    rbEgoVehicle = roadBoundaries(egoVehicle);
    plotLaneBoundary(plotterDataStruct.lbPlotter,rbEgoVehicle)

    advance(scenarioData);
    % pause(0.01);
    end

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
%% Function Used to Create a Birds Eye Plot
function [plotterDataStruct] = createBirdsEye(sensor, bepLimits, axes)

%Create birds eye plot
bep = birdsEyePlot('XLim',bepLimits(1:2),'YLim',bepLimits(3:4),'Parent',axes);

%Create coverage area plotter
caPlotter = coverageAreaPlotter(bep,'FaceColor','b');
plotCoverageArea(caPlotter,sensor.MountingLocation(1:2),sensor.RangeLimits(2),sensor.MountingAngles(1),sensor.FieldOfView(1));

%Create radar detections plotter
radarDetPlotter = detectionPlotter(bep,'MarkerEdgeColor','red');

%Create road boundaries plotter
lbPlotter = laneBoundaryPlotter(bep);
lmPlotter = laneMarkingPlotter(bep);

%Create vehicle plotter
olPlotter = outlinePlotter(bep);

plotterDataStruct = struct('BEP', bep, ...
    'caPlotter', caPlotter, ...
    'lbPlotter', lbPlotter, ...
    'lmPlotter', lmPlotter, ...
    'olPlotter', olPlotter, ...
    'radarDetPlotter', radarDetPlotter);
end