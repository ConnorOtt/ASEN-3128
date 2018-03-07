% Quick sim for Greg to fix linearized model for Assignment 3
clear; close all; clc;

% Initial Conditions matrix
condMat = zeros(6, 12);
condMat(:, 12) = -4; % initial height
condMat(1, 7) = deg2rad(5); % [rad] - Bank
condMat(2, 8) = deg2rad(5); % [rad] - Elevation
condMat(3, 9) = deg2rad(5); % [rad] - Azimuth
condMat(4, 4) = 0.1; % [rad/s] - Roll
condMat(5, 5) = 0.1; % [rad/s] - Pitch
condMat(6, 6) = 0.1; % [rad/s] - Yaw

titleCell = {'Bank Deviation, 5$^{\circ}$', 'Elevation Deviation, 5$^{\circ}$', ...
             'Azimuth Deviation, 5$^{\circ}$', 'Roll Deviation, 0.1 rad/s', ...
             'Pitch Deviation, 0.1 rad/s', 'Yaw Deviation, 0.1 rad/s'};
options = odeset('Events', @termEvents, 'RelTol', 1e-8);

tspan = [0, 5]; %s
linTrim = 0 * ones(1, 4);
[r, ~] = size(condMat);
sSize = get(0, 'screensize');
set(0, 'defaulttextinterpreter', 'latex')
for i = 1:r

    initConds = condMat(i, :);
    
    [t_lin, F_lin] = ode45(@(t, F)quadCopterODE_lin(t, F, linTrim), ...
                                tspan, initConds);
    %% pqr Plots
    figure('pos', [sSize(3)*0.25, sSize(4)*0.20, ...
                   sSize(3)*0.40, sSize(4)*0.70]);
    subplot(3, 1, 1)
    hold on; grid on;
    plot(t_lin, F_lin(:, 4), 'r-.', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('Roll, [rad/s]')
    legend('Linearized Model')
    
    subplot(3, 1, 2)
    hold on; grid on;
    plot(t_lin, F_lin(:, 5), 'r-.', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('Pitch, [rad/s]')
    
    subplot(3, 1, 3)
    hold on; grid on;
    plot(t_lin, F_lin(:, 6), 'r-.', 'linewidth', 1.2)
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
    xlabel('Time, [s]')
    ylabel('u, [m/s]')
    legend('Linearized')
    
    subplot(3, 1, 2)
    hold on; grid on;
    plot(t_lin, F_lin(:, 2), 'r-.', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('v, [rad/s]')
    
    subplot(3, 1, 3)
    hold on; grid on; 
    plot(t_lin, F_lin(:, 3), 'r-.', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('w, [rad/s]')
    
    [~, t] = suplabel(titleCell{i}, 't', [.1 .1 .84 .84]); 
    set(t, 'fontsize', 15)
%     set(gcf,'Visible','off')
%     if exist(['./Ass3figs/',titleCell{i}, '_uvw.png'], 'file') ~=2
%         saveas(gcf, ['./Ass3figs/',titleCell{i}, '_uvw.png']);
%     end
end