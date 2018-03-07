%--------------------------------------------------------------------------
% quadCopterSim simulates the dynamics of a small quad copter using
% numeical integration of Euler's Moment Equations
%
% Created: 1/30/18 - Connor Ott
% Last Modified: 1/30/18 - Connor Ott
%--------------------------------------------------------------------------

clc; clear; close all;

%% Hover Trim
ti = 0;
tf = 5; % s
momPert = zeros(1, 3); % No perturbation
analyticalTrim = ones(1, 4) * 0.068 * 9.81 / 4;
initConds = zeros(1, 12);
initConds(12) = -1;

options = odeset('Events', @termEvents, 'RelTol', 1e-8);
[t, F] = ode45(@(t, F)quadCopterODE(t, F, analyticalTrim, momPert), ...
                                           [ti, tf], initConds, options);

figure
hold on; grid on; axis equal;
plot3(F(:, 10), F(:, 11), F(:, 12), 'ro')
set(gca, 'zdir', 'reverse')
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Steady Hover Trim')
set(gca, 'ticklabelinterpreter', 'latex');

%% 5 m/s East trim
ti = 0;
tf = 5; % [s]

momPert = zeros(1, 3); % No perturbation
V = 5;          % [m/s]
heta = 1e-3;    % [N/(m/s)^2]
xsi = 3e-3;     % [N/(m/s)^2]
m = 0.068;      % [kg]
g = 9.81;       % [m/s^2]
solveF = @(x) m*g*sin(x) - 25*heta*cos(x).^2;
phi = fzero(solveF, 0);
f_Mag = m*g*cos(phi) + 25*xsi*sin(phi).^2;

steadyTrim_East = ones(1, 4) * f_Mag / 4;
initConds = zeros(1, 12);
initConds(12) = -1;
initConds(2) = V*cos(phi);
initConds(3) = -V*sin(phi);
initConds(7) = phi;

options = odeset('Events', @termEvents, 'RelTol', 1e-8);
[t, F] = ode45(@(t, F)quadCopterODE(t, F, steadyTrim_East, momPert), ...
                                           [ti, tf], initConds, options);

figure
hold on; grid on; axis equal;
plot3(F(:, 10), F(:, 11), F(:, 12), 'bo')
set(gca, 'zdir', 'reverse')
zlabel('Down Position, [m]')
ylabel('East Position, [m]')
xlabel('North Position, [m]')
title('Steady 5 m/s East trim')

%% Velocity Plots
figure
subplot(3, 1, 1)
hold on; grid on; grid minor;
plot(t, F(:, 1), 'b-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('North Velocity, [m/s]')

subplot(3, 1, 2)
hold on; grid on; grid minor;
plot(t, F(:, 2), 'b-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('East Velocity, [m/s]')

subplot(3, 1, 3)
hold on; grid on; grid minor;
plot(t, F(:, 3), 'b-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('Down Velocity, [m/s]')

%% Position Plots
figure
subplot(3, 1, 1)
hold on; grid on; grid minor;
plot(t, F(:, 10), 'm-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('North Position, [m/s]')

subplot(3, 1, 2)
hold on; grid on; grid minor;
plot(t, F(:, 11), 'm-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('East Position, [m/s]')

subplot(3, 1, 3)
hold on; grid on; grid minor;
plot(t, F(:, 12), 'm-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('Down Position, [m/s]')

%% Repeat of above with azimuth at 90 deg
% It travels north now, since the body frame has been rotated and the bank
% angle no longer causes motion in East direction.

ti = 0;
tf = 5; % [s]

momPert = zeros(1, 3); % No perturbation
u = 5;          % [m/s]
heta = 1e-3;    % [N/(rad/s)^2]
m = 0.068;      % [kg]
g = 9.81;       % [m/s^2]
theta = atan((heta^2 * u^2)/(m*g)); % [rad] - Bank
psi = pi/2;      % [rad] - Azimuth
f_Mag = sqrt((m*g)^2 + (heta^2*u^2)^2); %[N]

steadyTrim_East = ones(1, 4) * f_Mag / 4;
initConds = zeros(1, 12);
initConds(12) = -1;
initConds(1) = u;
initConds(8) = theta;
initConds(9) = psi;

options = odeset('Events', @termEvents, 'RelTol', 1e-8);
[t, F] = ode45(@(t, F)quadCopterODE(t, F, steadyTrim_East, momPert), ...
                                           [ti, tf], initConds, options);


figure
hold on; grid on; axis equal;
plot3(F(:, 10), F(:, 11), F(:, 12), 'ro')
set(gca, 'zdir', 'reverse')
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Steady 5 m/s East trim - 90 deg azimuth')

%% Position Plots
figure
subplot(3, 1, 1)
hold on; grid on; grid minor;
plot(t, F(:, 10), 'm-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('East Position, [m/s]')

subplot(3, 1, 2)
hold on; grid on; grid minor;
plot(t, F(:, 11), 'm-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('North Position, [m/s]')
axis([0, 5, -0.5, 0.5])

subplot(3, 1, 3)
hold on; grid on; grid minor;
plot(t, F(:, 12), 'm-', 'linewidth', 1.2)
xlabel('Time, [s]')
ylabel('Down Position, [m/s]')
axis([0, 5, -1.5, -0.5])

%% Simulate a perturbation in steady hover to see how quad copter reacts

ti = 0;
tf = 5; % s
analyticalTrim = ones(1, 4) * 0.068 * 9.81 / 4;
initConds = zeros(1, 12);
initConds(12) = -1;

figure
hold on; grid on; axis equal;
set(gca, 'zdir', 'reverse')
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Random Moment Perturbation')
plot3(0, 0, -1, 'ro', 'linewidth', 1.2)
momPert = randn(3, 1);
options = odeset('Events', @termEvents, 'RelTol', 1e-8);
[t, F] = ode45(@(t, F)quadCopterODE(t, F, analyticalTrim, momPert), ...
                                           [ti, tf], initConds, options);

plot3(F(:, 10), F(:, 11), F(:, 12), '-', 'linewidth', 1.1)

%% Plotting experimental quad copter data
load('RSdata_Drone01_1330.mat');
times = rt_estim.time(:);
xdata = rt_estim.signals.values(:, 1);
ydata = rt_estim.signals.values(:, 2);
zdata = rt_estim.signals.values(:, 3);

pdata = rt_estim.signals.values(:, 4);
qdata = rt_estim.signals.values(:, 5);
rdata = rt_estim.signals.values(:, 6);

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
hold on; grid on;
plot3(xdata(1), ydata(1), zdata(1), 'ro')
plot3(xdata, ydata, zdata, 'b-')
title('Quad Copter Trajectory w/ No Feedback Control')
xlabel('x Position, [m]')
ylabel('y Position, [m]')
zlabel('down Position, [m]')

