%{
Determines relationship between wind, volume, and angle of ascent for a
balloon sonde.

Created: 1/23/18 - Connor Ott
Last Modified: 1/27/18 - Connor Ott
%}

clear variables
close all
clc
set(0, 'defaulttextinterpreter', 'latex');

% Number library 
nums = balloonNums;
t0 = 0;
tf = 20; % s

%% Validation case
initialVals = [0, 0, 0, 0, 0, 0, 0, 1];
[~, F] = ode45('balloonODE', [t0 tf], initialVals);

figure
hold on; grid on; grid minor;
plot3(F(:, 4), F(:, 5), -F(:, 6), ...
    'r-', 'linewidth', 1.5)
xlabel('E Position [m]')
ylabel('N Position [m]')
zlabel('Down Position [m]')
title('Balloon Trajectory w/ Prevailing wind 4 m/s North, 2 m/s East')
set(gca, 'TickLabelInterpreter', 'latex',...
         'fontsize', 13, ...
         'box', 'on',...
         'Zdir','reverse'); 

%% Wind and Volume Variation
windVels = linspace(0, 20, 20); % [m/s]
vols = linspace(0, 10000, 50);   % [m^3]

FCell = cell(length(windVels), length(vols));
angles = zeros(length(windVels), length(vols));

% Varying both volume and North windspeed 
for i = 1:length(windVels)
    for j = 1:length(vols)
        initialVals = [0, 0, 0, 0, 0, 0, windVels(i), vols(j)];
        [~, FCell{i, j}] = ode45('balloonODE', [t0 tf], initialVals);
        
        angles(i, j) = atand(FCell{i, j}(end, 6)/FCell{i, j}(end, 5));
    end
end

%% Plotting Data
levels = 35:5:85;
figure
hold on;
set(gca, 'TickLabelInterpreter', 'latex',...
         'fontsize', 12, ...
         'box', 'on'); 
contour(windVels, vols, angles', levels, 'showtext', 'on')
title('Windspeed vs. Balloon Volume vs. Ascent Angle')
xlabel('Windspeed, [m/s]')
ylabel('Balloon Volume, [$m^3$]')
axis([0, 20, 250, 10000])

%% 
