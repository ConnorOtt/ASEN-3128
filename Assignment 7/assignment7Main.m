%--------------------------------------------------------------------------
% Main function for calculations for Assignment 6 problems
%
% Created: 3/7/18 - Connor Ott
%--------------------------------------------------------------------------

clc; clear; close all;
mpf = 0.3048;    % meters per foot
h = 40000 * mpf; % [ft]
[~, ~, ~, rho] = atmosisa(h);

%% Assignment 6 Problem 1 - Aerodynamic Derivatives
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
C_ma = -1.023; % Part a) modification
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
dataMat = [Xcol, Zcol, Mcol];
% disp(T)

% Creating LaTex table
input.data = dataMat;
input.tableColLabels = {'X', 'Y', 'M'};
input.tableRowLabels = {'u', 'w', 'q', '\dot{w}'};
input.dataFormat = {'%.2e', 2, '%.2e', 1}; 
input.tableColumnAlignment = 'c';
input.dataNanString = '-';
input.tableBorders = 0;
input.booktabs = 1;
input.tableCaption = 'Aerodynamic Derivatives';
input.tableLabel = 'aeroDer';
% call latexTable:
% latex = latexTable(input); % Maybe don't output this every time



%% Assignment 6 Problem 2 - A Matrix
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

% Creating LaTex table
input.data = A;
input.tableColLabels = {'', '', '', ''};
input.tableRowLabels = {'', '', '', ''};
input.dataFormat = {'%.2e', 3, '%.2e', 1}; 
input.tableColumnAlignment = 'c';
input.dataNanString = '-';
input.tableBorders = 0;
input.booktabs = 1;
input.tableCaption = '';
input.tableLabel = '';
% call latexTable:
% latex = latexTable(input); % Maybe don't output this every time


%% Assignment 6 Problem 3 - eigs, zeta, omega_n
phuVal = vals(3:4);
shortVal = vals(1:2);

% Natural Frequency 
w_nPhu = abs(phuVal(1));
w_nShort = abs(shortVal(1));

zetaPhu = -real(phuVal(1))/w_nPhu;
zetaShort = -real(shortVal(1))/w_nShort;

%% Assignment 6 Problem 4 - Approximations vs. Full A Matrix
lamSAprx(:, 1) = [ (M_q/(2*Iy) + 1/(2*Iy)*sqrt(M_q^2 + 4*Iy*u_0*M_w));
                   (M_q/(2*Iy) - 1/(2*Iy)*sqrt(M_q^2 + 4*Iy*u_0*M_w)) ];

w_nSAprx = abs(lamSAprx(1));
zetaSAprx = -real(real(lamSAprx(1)))/w_nSAprx;
TPhu_aprx =  pi * sqrt(2) * u_0 / g;
TPhu = 2*pi/imag(phuVal(1));


%% Assignment 7 Problem 2 - Adding Control Derivatives
C_xde = -3.818e-6;
C_zde = -0.3648;
C_mde = -1.444;

X_de = C_xde * 0.5 * rho * u_0^2 * S;
Z_de = C_zde * 0.5 * rho * u_0^2 * S;
M_de = C_mde * 0.5 * rho * u_0^2 * S * cbar;

% disp({X_de, Z_de, M_de})

% X_dp = C_xdp * 0.5 * rho * u_0^2 * S;
% Y_dp = C_zdp * 0.5 * rho * u_0^2 * S;
% M_dp = C_mdp * 0.5 * rho * u_0^2 * S * cbar;

%% Assignment 7 - Problem 3
% Creating the B matrix
B = zeros(4, 2);
B(1:3, 1) = [X_de/m, Z_de/(m-Z_wdot), M_de/Iy + M_wdot/(Iy*(m-Z_wdot))]';

% Choosing gains based on short period approximation of longitudinal
% dynamics. 

k_s = 1:0.01:20;
[k1, k2, imVecPlusApx, imVecMinApx, reVecApx] = deal(zeros(length(k_s), 1));

for i = 1:length(k_s)
   k1(i) = (1-sqrt(k_s(i)))*M_q/M_de;
   k2(i) = (1-k_s(i))*u_0*M_w/M_de;
   
   K_short = [k1(i), k2(i); 0, 0];
   Aaprx = [M_q/Iy, u_0*M_w/Iy; 1, 0];
   A_new = Aaprx - M_de/Iy * K_short;
   [~, svals] = eig(A_new, 'vector');
   
   imVecPlusApx(i) = imag(svals(1));
   imVecMinApx(i) = imag(svals(2));
   reVecApx(i) = real(svals(1));
  
end

%% Using short period approximation gains in full linearized dynamics

[imVecPlus, imVecMin, reVec] = deal(zeros(length(k_s), 1));
for i = 1:length(k_s)
    K_cell = zeros(2, 4);
    K_cell(1, 3:4) = [k1(i), k2(i)];
    
    A_star = A - B*K_cell; % Closed loop matrix
    [~, vals] = eig(A_star, 'vector');
    imVecPlus(i) = imag(vals(1));
    imVecMin(i) = imag(vals(2));
    reVec(i) = real(vals(1));
end

figure
hold on; grid on; grid minor;

plot(reVec, imVecPlus, 'k', 'linewidth', 1.1);
plot(reVec, imVecMin, 'm', 'linewidth', 1.1);
plot(reVecApx, imVecPlusApx, 'b', 'linewidth', 1.1);
plot(reVecApx, imVecMinApx, 'r', 'linewidth', 1.1);

leg = legend('Full Dynamics Positive Pair', ...
             'Full Dynamics Negative Pair', ...
             'Approximation Positive Pair', ...
             'Approximation Negative Pair');
title('Short Mode Full Dynamics and Approximation Eigenvalues');
xlabel('Real Values');
ylabel('Imaginary Values');

% %% Assignment 6 - Problem 5
% initCondsMat = diag([10, 10, 0.1, 0.1]);
% initCondsMat = [0, 0, 0, 0; initCondsMat];
% tSpanLong = [0, 300];
% tSpanShort = [0, 10];
% titlesS = {'Trim State', ...
%           'Initial $\Delta u$ of 10 m/s - Short Period', ...
%           'Initial $\Delta w$ of 10 m/s - Short Period', ...
%           'Initial $\Delta q$ of 0.1 rad/s - Short Period', ...
%           'Initial $\Delta \theta$ of 0.1 rad - Short Period'};
% titlesP = {'Trim State', ...
%           'Initial $\Delta u$ of 10 m/s - Phugoid', ...
%           'Initial $\Delta w$ of 10 m/s - Phugoid', ...
%           'Initial $\Delta q$ of 0.1 rad/s - Phugoid', ...
%           'Initial $\Delta \theta$ of 0.1 rad - Phugoid'};
% yLabels = {'$\Delta u$ [m/s]', '$\Delta w$ [m/s]', ...
%            '$\Delta q$ [rad/s]', '$\Delta \theta$ [rad]'};
% printTitlesS = {'Trim', 'deltaU', 'deltaW','deltaQ', 'deltaTheta'};
% printTitlesP = {'Trim', 'deltaUPhu', 'deltaWPhu', ...
%                 'deltaQPhu', 'deltaThetaPhu'};
% for i = 1:length(initCondsMat)
%    [t_s, F_s] = ode45(@(t, F)longSimODE(t, F, A), tSpanShort, ...
%                                     initCondsMat(i, :)); 
%    [t_p, F_p] = ode45(@(t, F)longSimODE(t, F, A), tSpanLong, ...
%                                     initCondsMat(i, :));
% %    % Short Period Plots
% %    plotPlots(t_s, F_s, titlesS{i}, yLabels, printTitlesS{i})
% %    
% %    % Long Period Plots
% %    plotPlots(t_p, F_p, titlesP{i}, yLabels, printTitlesP{i})
% end






