%--------------------------------------------------------------------------
% quadCopterSim_FBC simulates the dynamics of a quad copter with
% derivative feedback control.
%
% Dependancies: 
%       quadCopterODE_FBC - ODE function defining dynamics of quad copter.
%
% Created: 2/10/18 - Connor Ott
% Last Modified: 2/10/18 - Connor Ott
%--------------------------------------------------------------------------
%% Introducing Feedback control

sSize = get(0, 'screensize');
condMat = zeros(3, 12);
condMat(:, 12) = -4; % initial height
condMat(1, 4) = 0.1; % [rad/s]
condMat(2, 5) = 0.1; % [rad/s]
condMat(3, 6) = 0.1; % [rad/s]

titleCell = {'Pitch Deviation', 'Roll Deviation', 'Yaw Deviation'};
tspan = [0, 5]; %s
fbcTrim = ones(1, 4)*0.068*9.81 / 4;
for i = 1:length(titleCell)
    
    initConds = [condMat(i, :), fbcTrim];
    [t_FBC, F_FBC] = ode45(@(t, F)quadCopterODE_FBC(t, F), ...
                                                    tspan, initConds);

    %% pqr Plots
    figure('pos', [sSize(3)*0.25, sSize(4)*0.15, ...
                   sSize(3)*0.40, sSize(4)*0.70]);
    subplot(3, 1, 1)
    hold on; grid on;           
    plot(t_FBC, F_FBC(:, 4), 'b-', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('Roll, [rad/s]')
    
    subplot(3, 1, 2)
    hold on; grid on;
    plot(t_FBC, F_FBC(:, 5), 'b-', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('Pitch, [rad/s]')
    
    subplot(3, 1, 3)
    hold on; grid on;
    plot(t_FBC, F_FBC(:, 6), 'b-', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('Yaw, [rad/s]')
    
    [~, t] = suplabel([titleCell{i}, ' - With Feeback Control'],...
                       't', [.1 .1 .84 .84]);
    set(t, 'fontsize', 15)
    set(gcf,'Visible','off')
%     if exist(['./Ass3figs_fbc/',titleCell{i}, '_pqrFBC.png'], 'file') ~=2
        saveas(gcf, ['./Ass3figs_fbc/',titleCell{i}, '_pqrFBC.png']);
%     end
    
    
    %% uvw Plots
    figure('pos', [sSize(3)*0.25, sSize(4)*0.15, ...
                   sSize(3)*0.40, sSize(4)*0.70]);
    suplabel(titleCell{i}, 't'); 
    subplot(3, 1, 1)
    hold on; grid on;
    plot(t_FBC, F_FBC(:, 1), 'b-', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('u, [m/s]')
    
    subplot(3, 1, 2)
    hold on; grid on;
    plot(t_FBC, F_FBC(:, 2), 'b-', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('v, [rad/s]')
    
    subplot(3, 1, 3)
    hold on; grid on; 
    plot(t_FBC, F_FBC(:, 3), 'b-', 'linewidth', 1.2)
    xlabel('Time, [s]')
    ylabel('w, [rad/s]')
    
    [~, t] = suplabel([titleCell{i}, ' - With Feeback Control'], ...
                                     't', [.1 .1 .84 .84]); 
    set(t, 'fontsize', 15)
    set(gcf,'Visible','off')
%     if exist(['./Ass3figs_fbc/',titleCell{i}, '_uvwFBC.png'], 'file') ~=2
        saveas(gcf, ['./Ass3figs_fbc/',titleCell{i}, '_uvwFBC.png']);
%     end
end


