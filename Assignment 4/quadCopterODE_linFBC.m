function dfdt = quadCopterODE_linFBC(t, F, trim)
% Defines dynamics of quad copter


%% Physical properties and constants
alpha = 2e-6;   % [N/(m/s)^2]
beta = 1e-6;    % [N/(m/s)^2]
heta = 1e-3;    % [N/(rad/s)^2]
xsi = 3e-3;     % [N/(rad/s)^2]

I_x = 6.8e-5;   % [kg m^2]
I_y = 9.2e-5;   % ['']
I_z = 1.35e-4;  % ['']

m = 0.068;      % [kg]
d = 0.06;       % [m]
k = 0.0024;     % [~]
rad = d/sqrt(2);  % [m]
g = 9.81;       % [m/s^2]

%% Pulling from input, F and trim
% Inertial velocity in body coords
delta_u = F(1);
delta_v = F(2);
delta_w = F(3);

% Inertial angular velocity in body coords
delta_p = F(4); % roll
delta_q = F(5); % pitch
delta_r = F(6); % yaw

% Euler angles
delta_phi = F(7); % Bank
delta_theta = F(8); % Elevation 
delta_psi = F(9); % Azimuth

% Position vector from origin to COM in inertial coords
x_E = F(10);
y_E = F(11);
z_e = F(12);

% Solving for gains and then motor forces. 
zeta = 0.8;      % Damping factor 
tau = 0.1;       % [s] time constant
k_r = 0.0012;    % [Nm/(rad/s)]
        
omega_n = 1/(tau * zeta);
k_1p = 2 * zeta * omega_n * I_x;
k_2p = omega_n^2 * I_x;

k_1q = 2 * zeta * omega_n * I_y;
k_2q = omega_n^2 * I_y;

% A = [-rad, rad, rad, -rad; ...
%      -rad, -rad, rad, rad; ...
%      -k, k, -k, k; ...
%      -1, -1, -1, -1];
% % RHS
% deltaL_c = -k_1p*delta_p - k_2p*delta_phi;
% deltaM_c = -k_1q*delta_q - k_2q*delta_theta;
% deltaN_c = -k_r * r;
% deltaZ_c = 0;
% deltaC = [deltaL_c, deltaM_c, deltaN_c, deltaZ_c]';

% % Solving Ax = b
% deltaf = A^(-1) * deltaC;
% 
% delta_f1 = deltaf(1);
% delta_f2 = deltaf(2);
% delta_f3 = deltaf(3);
% delta_f4 = deltaf(4);

%% Aerodynamic and Control Moments and Forces

% Moments
L_a = 0; %- alpha^2 * p^2 * sign(p);
M_a = 0; %- alpha^2 * q^2 * sign(q);
N_a = 0; %- beta^2 * r^2 * sign(r);

deltaL_c = -k_1p*delta_p - k_2p*delta_phi;
deltaM_c = -k_1q*delta_q - k_2q*delta_theta;
deltaN_c = -k_r * delta_r;

% 
% L_c = ((delta_f2 + delta_f3) - (delta_f1 + delta_f4)) * rad;
% M_c = ((delta_f3 + delta_f4) - (delta_f1 + delta_f2)) * rad;
% N_c = (delta_f2 + delta_f4 - (delta_f1 + delta_f2)) * k;

L = L_a + deltaL_c;
M = M_a + deltaM_c;
N = N_a + deltaN_c;

% if t > 1 && t < 1.5 % Throw a moment perturbation in for 0.2 seconds
%     L = L + momPert(1);
%     M = M + momPert(2);
%     N = N + momPert(3);
% end

% Forces
X_a = 0; %- heta^2 * delta_u^2 * sign(delta_u);
Y_a = 0; %- heta^2 * delta_v^2 * sign(delta_v);
Z_a = 0; %- xsi^2 * delta_w^2 * sign(delta_w); 

X_c = 0;
Y_c = 0;
deltaZ_c = -sum(trim);

deltaX = X_a + X_c;
deltaY = Y_a + Y_c;
deltaZ = Z_a + deltaZ_c;

% Determining Roll, Pitch, and Yaw rates of change
deltap_dot = 1/I_x * deltaL_c;
deltaq_dot = 1/I_y * deltaM_c;
deltar_dot = 1/I_z * deltaN_c;
dOmega_bdt = [deltap_dot, deltaq_dot, deltar_dot]';

% Determining translation rates
deltau_dot = -g * delta_theta;
deltav_dot = g * delta_phi;
deltaw_dot = 1/m * deltaZ_c;
dV_bdt = [deltau_dot, deltav_dot, deltaw_dot]';

% Tranlating body to inertial coordinates
x_dot =  delta_u * cos(delta_theta)*cos(delta_psi) + ...
         delta_v * (sin(delta_phi)*sin(delta_theta)*cos(delta_psi) - cos(delta_phi)*sin(delta_psi)) + ...
         delta_w * (cos(delta_phi)*sin(delta_theta)*cos(delta_psi) + sin(delta_phi)*sin(delta_psi));
y_dot =  delta_u * cos(delta_theta)*sin(delta_psi) + ...
         delta_v * (sin(delta_phi)*sin(delta_theta)*sin(delta_psi) + cos(delta_phi)*cos(delta_psi)) + ...
         delta_w * (cos(delta_phi)*sin(delta_theta)*sin(delta_psi) - sin(delta_phi)*cos(delta_psi));
z_dot = -delta_u * sin(delta_theta) + ...
         delta_v * sin(delta_phi)*cos(delta_theta) + ...
         delta_w * cos(delta_phi)*cos(delta_theta);
dV_Edt = [x_dot, y_dot, z_dot]';
    
% Translating Angular Momentum to Euler angles
deltaphi_dot   = delta_p;
deltatheta_dot = delta_q;
deltapsi_dot   = delta_r;
dEuldt = [deltaphi_dot, deltatheta_dot, deltapsi_dot]';

% Concatenating for output
dfdt = [dV_bdt; dOmega_bdt; dEuldt; dV_Edt];



end