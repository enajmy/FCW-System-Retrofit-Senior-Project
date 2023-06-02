%% Main Function
% Variables needed - scenarioData
close all;
load('testWorkspace.mat');

figure1 = displayScenarioPlot(scenarioData);

function [scenarioFigure] = displayScenarioPlot(scenarioData)
    % Plotting using scenario(plot) method
    scenarioFigure = figure;
    scenarioAxes = axes;
    hold on
    axis manual
    axis([1 40 -15 15]);
    plot(scenarioData,'Parent',scenarioAxes,'Centerline','on','RoadCenters','on','Waypoints','on');
    
    pause(0.5);
    
    for i = 1:(scenarioData.StopTime/scenarioData.SampleTime)
        advance(scenarioData);
        pause(0.001);
    end
end