%{
Determines trajectory of a golf ball with a given initial velocity. Varies
mass and crosswind speed. 

Created: 1/23/18 - Connor Ott
Last Modified: 1/27/18 - Connor Ott
%}

clear variables
close all
clc

nums = numbers;
%% Getting the Initial V for the trajectory of the ball.
vInit = [nums.V_xi, 0, nums.V_zi];  % [m/s]

%% Calling ode45 with initial velocity and position

V_0x = vInit(1);
V_0y = vInit(2);
V_0z = vInit(3); 

[X_0x, X_0y, X_0z] = deal(0); % m - Inital position at (0,0,0) 

t0 = 0;
tf = 5; % s

%% Wind Variation
windVels = linspace(-5, 5, 5);
colors = distinguishable_colors(length(windVels));
for i = 1:length(windVels)
    
    legStr{i} = sprintf('W = %.2f $\\hat{N}$ m/s', windVels(i));

    initialVals = [V_0x, V_0y, V_0z, 0, 0, 0, windVels(i), nums.m];
    [~, Fwind{i}] = ode45('golfBallODE', [t0 tf], initialVals);
    
    figure(1)
    hold on; grid on; grid minor;
    plot3(Fwind{i}(:, 4), Fwind{i}(:, 5), -Fwind{i}(:, 6), ... 
          'linewidth', 1.2, 'color', colors(i, :));
    xlabel('E Position [m]')
    ylabel('N Position [m]')
    zlabel('Down Position [m]')
    title('Golf Ball Trajectory with Varied Crosswind')
    set(gca, 'TickLabelInterpreter', 'latex',...
         'fontsize', 13, ...
         'box', 'on',...
         'Zdir','reverse'); 
    deflect(i) = Fwind{i}(end, 5);
end
leg = legend(legStr);
set(leg, 'interpreter', 'latex', ...
         'fontsize', 10);

tempFit = polyfit(windVels, deflect, 1);
% Ball sensitivity to crosswind
wSense = tempFit(1); % m deflect / m/s crosswind Vel
fprintf('Ball sensitivity to crosswind: %.3f m/m/s\n', wSense)

%% Mass Variation
% Varying mass by 20% in either direction
massVec = linspace(nums.m*0.8, nums.m*1.2, 3);
kE_max = 0.5 * nums.m * sqrt(2) * nums.V_xi; % max kinetic energy
colors = distinguishable_colors(length(massVec)); % plotting
for i = 1:length(massVec)
    V_0x = kE_max * 2 / (massVec(i) * sqrt(2));
    V_0z = V_0x;
    legStr{i} = sprintf('m = %.2f kg', massVec(i));
    initialVals = [V_0x, V_0y, V_0z, 0, 0, 0, 0, massVec(i)];
    [~, Fmass{i}] = ode45('golfBallODE', [t0 tf], initialVals);
    
    figure(2)
    hold on; grid on; grid minor;
    plot3(Fmass{i}(:, 4), Fmass{i}(:, 5), -Fmass{i}(:, 6), ...
          'linewidth', 1.2, 'color', colors(i, :));
    xlabel('E Position [m]')
    ylabel('N Position [m]')
    zlabel('Down Position [m]')
    title('Golf Ball Trajectory with Varied Mass')
    set(gca, 'TickLabelInterpreter', 'latex',...
         'fontsize', 13, ...
         'box', 'on',...
         'Zdir','reverse'); 
     
        % Max ranges, indicates LIGHTER Ball will work best
    rangeVec(i) = Fmass{i}(end, 4);
end
leg = legend(legStr);
set(leg, 'interpreter', 'latex');
fprintf('Maximum Range and ball mass: %.2fm, %.2fkg\n', ...
         rangeVec(1), massVec(1));



profile viewer







