%--------------------------------------------------------------------------
% quadCopterSim_Ass4 simulates the dynamics of a small quad copter using
% numerical integration of Euler's Moment Equations. This simulation
% introduces PD feedback control.
%
% Created: 2/14/18 - Connor Ott
% Last Modified: 2/14/18 - Connor Ott
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
[t, F] = ode45(@(t, F)quadCopterODE_linFBC(t, F, analyticalTrim), ...
                                           [ti, tf], initConds, options);

%% Part 2 
% a) - f) Deviations in different parameters and comparision between linear
% and nonlinear models. 
ti = 0;
tf = 5; % s
linTrim = zeros(1, 4);
nLTrim = 0.068 * 9.81 / 4 * ones(1, 4);

% Initial Conditions Matrix
condMat = zeros(4, 12);
condMat(:, 12) = -4; % initial height
condMat(1, 7) = deg2rad(5); % [rad] - Bank
condMat(2, 8) = deg2rad(5); % [rad] - Elevation
condMat(3, 4) = 0.1; % [rad/s] - Roll
condMat(4, 5) = 0.1; % [rad/s] - Pitch

titleCell = {'Bank Deviation', 'Elevation Deviation', ...
             'Roll Deviation', 'Pitch Deviation'};
options = odeset('Events', @termEvents, 'RelTol', 1e-8);

r = size(condMat, 1);
sSize = get(0, 'screensize');
for i = 1:r

    tspan = [0, 5]; %s
    initConds = condMat(i, :);
    
    [t_nc, F_nc] = ode45(@(t, F)quadCopterODE_lin(t, F, linTrim), ...
                                tspan, initConds);
    [t_linC, F_linC] = ode45(@(t, F)quadCopterODE_linFBC(t, F, linTrim), ...
                                tspan, initConds);
    [t_nonLinC, F_nonLinC] = ode45(@(t, F)quadCopterODE_FBCPD(t, F, nLTrim), ...
                                tspan, initConds);
    
    %% pqr Plots
    figure('pos', [sSize(3)*0.25, sSize(4)*0.20, ...
                   sSize(3)*0.40, sSize(4)*0.70]);
    subplot(3, 1, 1)
    hold on; grid on;
    plot(t_nc, F_nc(:, 4), 'r-.', 'linewidth', 1.5)
    plot(t_linC, F_linC(:, 4), 'k--', 'linewidth', 1.5)
    plot(t_nonLinC, F_nonLinC(:, 4), 'b-', 'linewidth', 1.5)
    xlabel('Time, [s]')
    ylabel('Roll, [rad/s]')
    leg = legend('Linearized No Control', 'Linearized PD Control',...
           'Non Linearized PD Control');
    set(leg, 'interpreter', 'latex', ...
             'location', 'best')
    legend('boxoff')

    
    subplot(3, 1, 2)
    hold on; grid on;
    plot(t_nc, F_nc(:, 5), 'r-.', 'linewidth', 1.5)
    plot(t_linC, F_linC(:, 5), 'k-', 'linewidth', 1.5)
    plot(t_nonLinC, F_nonLinC(:, 5), 'b-', 'linewidth', 1.5)

    xlabel('Time, [s]')
    ylabel('Pitch, [rad/s]')
    
    subplot(3, 1, 3)
    hold on; grid on;
    plot(t_nc, F_nc(:, 6), 'r-.', 'linewidth', 1.5)
    plot(t_linC, F_linC(:, 6), 'k--', 'linewidth', 1.5)
    plot(t_nonLinC, F_nonLinC(:, 6), 'b-', 'linewidth', 1.5)
    xlabel('Time, [s]')
    ylabel('Yaw, [rad/s]')
    
    [~, t] = suplabel(titleCell{i}, 't', [.1 .1 .84 .84]);
    set(t, 'fontsize', 15)
%     set(gcf,'Visible','off')
%     if exist(['./Ass4figs/',titleCell{i}, '_pqr.png'], 'file') ~=2
        saveas(gcf, ['./Ass4figs/',titleCell{i}, '_pqr.png']);
%     end
     
    
    %% uvw Plots
    figure('pos', [sSize(3)*0.25, sSize(4)*0.20, ...
                   sSize(3)*0.40, sSize(4)*0.70]);
    suplabel(titleCell{i}, 't'); 
    subplot(3, 1, 1)
    hold on; grid on;
    plot(t_nc, F_nc(:, 1), 'r.-', 'linewidth', 1.5)
    plot(t_linC, F_linC(:, 1), 'k--', 'linewidth', 1.5)
    plot(t_nonLinC, F_nonLinC(:, 1), 'b-', 'linewidth', 1.5)
    xlabel('Time, [s]')
    ylabel('u, [m/s]')
    leg = legend('Linearized No Control', 'Linearized PD Control',...
           'Non Linearized PD Control');
    set(leg, 'interpreter', 'latex', ...
             'location', 'best')
    legend('boxoff')
 
    subplot(3, 1, 2)
    hold on; grid on;
    plot(t_nc, F_nc(:, 2), 'r.-', 'linewidth', 1.5)
    plot(t_linC, F_linC(:, 2), 'k--', 'linewidth', 1.5)
    plot(t_nonLinC, F_nonLinC(:, 2), 'b-', 'linewidth', 1.5)
    xlabel('Time, [s]')
    ylabel('v, [rad/s]')
    
    subplot(3, 1, 3)
    hold on; grid on; 
    plot(t_nc, F_nc(:, 3), 'r.-', 'linewidth', 1.5)
    plot(t_linC, F_linC(:, 3), 'k--', 'linewidth', 1.5)
    plot(t_nonLinC, F_nonLinC(:, 3), 'b-', 'linewidth', 1.5)
    xlabel('Time, [s]')
    ylabel('w, [rad/s]')
    
    [~, t] = suplabel(titleCell{i}, 't', [.1 .1 .84 .84]); 
    set(t, 'fontsize', 15)
%     set(gcf,'Visible','off')
    %if exist(['./Ass4figs/',titleCell{i}, '_uvw.png'], 'file') ~=2
        saveas(gcf, ['./Ass4figs/',titleCell{i}, '_uvw.png']);
    %end
end

%% Plotting experimental quad copter data

ti = 0;
tf = 5; % s
analyticalTrim = 0 * ones(1, 4);
initConds = zeros(1, 12);
initConds(12) = -1;
initConds(8) = deg2rad(-10);

options = odeset('Events', @termEvents, 'RelTol', 1e-8);
[t, F] = ode45(@(t, F)quadCopterODE_linFBC(t, F, analyticalTrim), ...
                                           [ti, tf], initConds, options);

load('RSdata_Drone09_1345.mat');
times = rt_estim.time(:);
pdata = rt_estim.signals.values(:, 4);
qdata = rt_estim.signals.values(:, 5);
rdata = rt_estim.signals.values(:, 6);

figure
hold on; grid on; grid minor;
plot(times, pdata, 'm-', 'linewidth', 1.3)
plot(times, qdata, 'k-', 'linewidth', 1.3)
plot(times, rdata, 'c-', 'linewidth', 1.3)
title('Quad Copter Angular Rates w/ Feedback Control')
xlabel('Time, [s]')
ylabel('Angular rate, [rad/s]')
legend('Yaw', 'Pitch', 'Roll')

figure
hold on; grid on; grid minor;
plot(t, F(:, 4), 'm-', 'linewidth', 1.3)
plot(t, F(:, 5), 'k-', 'linewidth', 1.3)
plot(t, F(:, 6), 'c-', 'linewidth', 1.3)
title('Quad Copter Simulation Angular Rates w/ Feedback Control')
xlabel('Time, [s]')
ylabel('Angular rate, [rad/s]')
legend('Yaw', 'Pitch', 'Roll')

