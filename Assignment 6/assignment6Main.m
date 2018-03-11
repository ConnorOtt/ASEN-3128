%--------------------------------------------------------------------------
% Main function for calculations for Assignment 6 problems
%
% Created: 3/7/18 - Connor Ott
%--------------------------------------------------------------------------

clc; clear; close all;
mpf = 0.3048;    % meters per foot
h = 40000 * mpf; % [ft]
[~, ~, ~, rho] = atmosisa(h);


%% Problem 1 - Aerodynamic Derivatives
% Values from table 6.1
C_X_u = -0.108;
C_xa = 0.2193;
C_xq = 0;
C_xahat = 0;

C_zu = -0.106;
C_za = -4.920;
C_zq = -5.921;
C_zahat = 5.896;

C_mu = 0.1043;
C_ma = -1.023;
C_mq = -23.92;
C_mahat = -6.314;

% Values from table E.1
theta_0 = 0;           % [deg]
u_0 = 265.48;          % [m/s]
w_0 = 0;               % [m/s]
S = 5500 * mpf^2;      % [m^2]
cbar = 27.31 * mpf;    % [m]
W = 6.366e5 * 4.44822; % [N]
C_w0 = W / (0.5*rho*u_0^2*S);

% X derivatives
X_u = rho*u_0*S*C_w0*sind(theta_0) + 0.5*rho*u_0*S*C_X_u;
X_w = 0.5*rho*u_0*S*C_xa;
X_q = 0.25*rho*u_0*cbar*S*C_xq;
X_wdot = 0.25*rho*cbar*S*C_xahat;

% Z derivatives
Z_u = -rho*u_0*S*C_w0*cosd(theta_0) + 0.5*rho*u_0*S*C_zu;
Z_w = 0.5*rho*u_0*S*C_za;
Z_q = 0.25*rho*u_0*cbar*S*C_zq;
Z_wdot = 0.25*rho*cbar*S*C_zahat;

% Moment derivative
M_u = 0.5*rho*u_0*cbar*S*C_mu;
M_w = 0.5*rho*u_0*cbar*S*C_ma;
M_q = 0.25*rho*u_0*cbar^2*S*C_mq;
M_wdot = 0.25*rho*cbar^2*S*C_mahat;

% Final Longitudinal Dimensional Derivatives
Xcol = [X_u, X_w, X_q, X_wdot]';
Zcol = [Z_u, Z_w, Z_q, Z_wdot]';
Mcol = [M_u, M_w, M_q, M_wdot]';
T = table(Xcol, Zcol, Mcol);
% disp(T)

%% Problem 2 - A Matrix
W = 2.83176e6; % [N]
g = 9.81; % [m/s^2] 
m = W/g; % [kg]
Ix = 0.247e8;
Iy = 0.449e8;
Iz = 0.673e8;


A = [X_u/m, X_w/m, 0, -g*cos(theta_0); ...
    Z_u/(m-Z_wdot), Z_w/(m-Z_wdot), (Z_q+m*u_0)/(m-Z_wdot), -m*g*sin(theta_0)/(m-Z_wdot); ...
    (1/Iy)*(M_u + M_wdot*Z_u/(m-Z_wdot)), (1/Iy)*(M_w + M_wdot*Z_w/(m-Z_wdot)), ...
    (1/Iy)*(M_q + M_wdot*(Z_q+m*u_0)/(m-Z_wdot)), M_wdot*m*g*sin(theta_0)/...
    (Iy*(m-Z_wdot)); ...
    0, 0, 1, 0];

[vecs, vals] = eig(A, 'vector');

%% Problem 3 - eigs, zeta, omega_n
phuVal = vals(3:4);
shortVal = vals(1:2);

% Natural Frequency 
w_nPhu = abs(phuVal(1));
w_nShort = abs(shortVal(1));

zetaPhu = -real(phuVal(1))/w_nPhu;
zetaShort = -real(shortVal(1))/w_nShort;


%% Problem 4 - Approximations vs. Full A Matrix
lamSaprx(:, 1) = [ (M_q/(2*Iy) + 1/(2*Iy)*sqrt(M_q^2 + 4*Iy*u_0*M_w));
                   (M_q/(2*Iy) - 1/(2*Iy)*sqrt(M_q^2 + 4*Iy*u_0*M_w)) ];
                                
TPhu_aprx =  pi * sqrt(2) * u_0 / g;
TPhu = 2*pi/imag(phuVal(1));

%% Problem 5 - ODE Sim
initCondsMat = diag([10, 10, 0.1, 0.1]);
tSpanLong = [0, 300];
tSpanShort = [0, 10];
titles = {'$\Delta u$ Deviation of 10 m/s', ...
          '$\Delta w$ Deviation of 10 m/s', ...
          '$\Delta q$ Deviation of 0.1 rad/s', ...
          '$\Delta \theta$ Deviation of 0.1 rad'};
yLabels = {'$\Delta u$ [m/s]', '$\Delta w$ [m/s]', ...
           '$\Delta q$ [rad/s]', '$\Delta \theta$ [rad]'};

for i = 1:length(initCondsMat)
   [t_s, F_s] = ode45(@(t, F)longSimODE(t, F, A), tSpanShort, ...
                                    initCondsMat(i, :)); 
   [t_p, F_p] = ode45(@(t, F)longSimODE(t, F, A), tSpanLong, ...
                                    initCondsMat(i, :));
   % Short Period Plots
   plotPlots(t_s, F_s, titles{i}, yLabels)
   
   % Long Period Plots
   plotPlots(t_p, F_p, titles{i}, yLabels)
end






