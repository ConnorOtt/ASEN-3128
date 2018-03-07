function dfdt = quadCopterODE_lin(t, F, trim)
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
delta_p = F(4);
delta_q = F(5);
delta_r = F(6);

% Euler angles
delta_phi = F(7);
delta_theta = F(8);
delta_psi = F(9);

% Position vector from origin to COM in inertial coords
x_E = F(10);
y_E = F(11);
z_e = F(12);

delta_f1 = trim(1); % motor forces
delta_f2 = trim(2); 
delta_f3 = trim(3); 
delta_f4 = trim(4);

%% Aerodynamic and Control Moments and Forces

% Moments
L_a = 0; %- alpha^2 * p^2 * sign(p);
M_a = 0; %- alpha^2 * q^2 * sign(q);
N_a = 0; %- beta^2 * r^2 * sign(r);

L_c = ((delta_f2 + delta_f3) - (delta_f1 + delta_f4)) * rad;
M_c = ((delta_f3 + delta_f4) - (delta_f1 + delta_f2)) * rad;
N_c = (delta_f2 + delta_f4 - (delta_f1 + delta_f2)) * k;

L = L_a + L_c;
M = M_a + M_c;
N = N_a + N_c;

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
Z_c = -sum(trim);

X = X_a + X_c;
Y = Y_a + Y_c;
Z = Z_a + Z_c;

% Determining Roll, Pitch, and Yaw rates of change
deltap_dot = rad/I_x * (delta_f2 + delta_f3 - delta_f1 - delta_f4);
deltaq_dot = rad/I_y * (delta_f3 + delta_f4 - delta_f1 - delta_f2);
deltar_dot = k/I_z * (delta_f2 + delta_f4 - delta_f1 - delta_f3);
dOmega_bdt = [deltap_dot, deltaq_dot, deltar_dot]';

% Determining translation rates
deltau_dot = -g * delta_theta;
deltav_dot = g * delta_phi;
deltaw_dot = 1/m * Z;
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