%--------------------------------------------------------------------------
% quadCopterSim_Ass3 simulates the dynamics of a small quad copter using
% numeical integration of Euler's Moment Equations
%
% Created: 1/30/18 - Connor Ott
% Last Modified: 1/30/18 - Connor Ott
%--------------------------------------------------------------------------

clc; clear; close all;
set(0, 'defaulttextinterpreter', 'latex');

%% Hover Trim
ti = 0;
tf = 5; % s
analyticalTrim = 0 * ones(1, 4);
initConds = zeros(1, 12);
initConds(12) = -1;

options = odeset('Events', @termEvents, 'RelTol', 1e-8);
[t, F] = ode45(@(t, F)quadCopterODE_lin(t, F, analyticalTrim), ...
                                           [ti, tf], initConds, options);

figure('visible', 'off')
hold on; grid on; axis equal;
plot3(F(:, 10), F(:, 11), F(:, 12), 'ro')
set(gca, 'zdir', 'reverse')
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Steady Hover Trim - Linearized')
set(gca, 'ticklabelinterpreter', 'latex', ...
         'fontsize', 12);
view(-59, 17)

%% Part 2 
% a) - f) Deviations in different parameters and comparision between linear
% and nonlinear models. 
ti = 0;
tf = 5; % s
linTrim = 0 * ones(1, 4);
nLTrim = ones(1, 4)*0.068*9.81 / 4;

condMat = zeros(6, 12);
condMat(:, 12) = -4; % initial height
condMat(1, 7) = deg2rad(5); % [rad] - Bank
condMat(2, 8) = deg2rad(5); % [rad] - Elevation
condMat(3, 9) = deg2rad(5); % [rad] - Azimuth
condMat(4, 4) = 0.1; % [rad/s] - Roll
condMat(5, 5) = 0.1; % [rad/s] - Pitch
condMat(6, 6) = 0.1; % [rad/s] - Yaw

titleCell = {'Bank Deviation', 'Elevation Deviation', ...
             'Azimuth Deviation', 'Roll Deviation', ...
             'Pitch Deviation', 'Yaw Deviation'};
options = odeset('Events', @termEvents, 'RelTol', 1e-8);

[r, ~] = size(condMat);
sSize = get(0, 'screensize');
for i = 1:r

    tspan = [0, 5]; %s
    initConds = condMat(i, :);
    
    [t_lin, F_lin] = ode45(@(t, F)quadCopterODE_lin(t, F, linTrim), ...
                                tspan, initConds);
    [t_nL, F_nL] = ode45(@(t, F)quadCopterODE(t, F, nLTrim), ...
                                tspan, initConds);

    %% pqr Plots
    figure('pos', [sSize(3)*0.25, sSize(4)*0.20, ...
                   sSize(3)*0.40, sSize(4)*0.70]);
    subplot(3, 1, 1)
    hold on; grid on;
    plot(t_lin, F_lin(:, 4), 'r-.', 'linewidth', 1.2)
    plot(t_nL, F_nL(:, 4), 'k--', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('Roll, [rad/s]')
    legend('Linearized', 'Non-linearized')
    
    subplot(3, 1, 2)
    hold on; grid on;
    plot(t_lin, F_lin(:, 5), 'r-.', 'linewidth', 1.2)
    plot(t_nL, F_nL(:, 5), 'k--', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('Pitch, [rad/s]')
    
    subplot(3, 1, 3)
    hold on; grid on;
    plot(t_lin, F_lin(:, 6), 'r-.', 'linewidth', 1.2)
    plot(t_nL, F_nL(:, 6), 'k--', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('Yaw, [rad/s]')
    
    [~, t] = suplabel(titleCell{i}, 't', [.1 .1 .84 .84]);
    set(t, 'fontsize', 15)
%     set(gcf,'Visible','off')
%     if exist(['./Ass3figs/',titleCell{i}, '_pqr.png'], 'file') ~=2
%         saveas(gcf, ['./Ass3figs/',titleCell{i}, '_pqr.png']);
%     end
    
    
    %% uvw Plots
    figure('pos', [sSize(3)*0.25, sSize(4)*0.20, ...
                   sSize(3)*0.40, sSize(4)*0.70]);
    suplabel(titleCell{i}, 't'); 
    subplot(3, 1, 1)
    hold on; grid on;
    plot(t_lin, F_lin(:, 1), 'r-.', 'linewidth', 1.2)
    plot(t_nL, F_nL(:, 1), 'k--', 'linewidth', 1.5)
    xlabel('Time, [s]')
    ylabel('u, [m/s]')
    legend('Linearized', 'Non-linearized')
    
    subplot(3, 1, 2)
    hold on; grid on;
    plot(t_lin, F_lin(:, 2), 'r-.', 'linewidth', 1.2)
    plot(t_nL, F_nL(:, 2), 'k--', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('v, [rad/s]')
    
    subplot(3, 1, 3)
    hold on; grid on; 
    plot(t_lin, F_lin(:, 3), 'r-.', 'linewidth', 1.2)
    plot(t_nL, F_nL(:, 3), 'k--', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('w, [rad/s]')
    
    [~, t] = suplabel(titleCell{i}, 't', [.1 .1 .84 .84]); 
    set(t, 'fontsize', 15)
%     set(gcf,'Visible','off')
%     if exist(['./Ass3figs/',titleCell{i}, '_uvw.png'], 'file') ~=2
%         saveas(gcf, ['./Ass3figs/',titleCell{i}, '_uvw.png']);
%     end
end

%% Plotting experimental quad copter data
load('RSdata_Drone01_1330.mat');
times = rt_estim.time(:);
pdata = rt_estim.signals.values(:, 4);
qdata = rt_estim.signals.values(:, 5);
rdata = rt_estim.signals.values(:, 6);

load('RSdata_Wed_1315.mat')
time_d = rt_estim.time(:);
pdata_d = rt_estim.signals.values(:, 4);
qdata_d = rt_estim.signals.values(:, 5);
rdata_d = rt_estim.signals.values(:, 6);


figure
hold on; grid on; grid minor;
plot(times, pdata, 'm-', 'linewidth', 1.3)
plot(times, qdata, 'k-', 'linewidth', 1.3)
plot(times, rdata, 'c-', 'linewidth', 1.3)
title('Quad Copter Angular Rates w/ No Feedback Control')
xlabel('Time, [s]')
ylabel('Angular rate, [rad/s]')
legend('Yaw', 'Pitch', 'Roll')

figure
hold on; grid on; grid minor;
plot(time_d, pdata_d, 'm-', 'linewidth', 1.3)
plot(time_d, qdata_d, 'k-', 'linewidth', 1.3)
plot(time_d, rdata_d, 'c-', 'linewidth', 1.3)
title('Quad Copter Angular Rates w/ Derivative Feedback Control')
xlabel('Time, [s]')
ylabel('Angular rate, [rad/s]')
legend('Yaw', 'Pitch', 'Roll')
