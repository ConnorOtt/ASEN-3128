function dfdt = quadCopterODE(t, F, trim, momPert)
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
u = F(1);
v = F(2);
w = F(3);

% Inertial angular velocity in body coords
p = F(4);
q = F(5);
r = F(6);

% Euler angles
phi = F(7);
theta = F(8);
psi = F(9);

% Position vector from origin to COM in inertial coords
x_E = F(10);
y_E = F(11);
z_e = F(12);

f1 = trim(1); % motor forces
f2 = trim(2); 
f3 = trim(3); 
f4 = trim(4);

%% Aerodynamic and Control Moments and Forces

% Moments
L_a = - alpha^2 * p^2 * sign(p);
M_a = - alpha^2 * q^2 * sign(q);
N_a = - beta^2 * r^2 * sign(r);

L_c = ((f2 + f3) - (f1 + f4)) * rad;
M_c = ((f3 + f4) - (f1 + f2)) * rad;
N_c = (f2 + f4 - (f1 + f2)) * k;

L = L_a + L_c;
M = M_a + M_c;
N = N_a + N_c;

if t > 1 && t < 1.5 % Throw a moment perturbation in for 0.2 seconds
    L = L + momPert(1);
    M = M + momPert(2);
    N = N + momPert(3);
end

% Forces
X_a = - heta^2 * u^2 * sign(u);
Y_a = - heta^2 * v^2 * sign(v);
Z_a = - xsi^2 * w^2 * sign(w); 

X_c = 0;
Y_c = 0;
Z_c = -sum(trim);

X = X_a + X_c;
Y = Y_a + Y_c;
Z = Z_a + Z_c;

% Determining Roll, Pitch, and Yaw rates of change
p_dot = (I_y - I_z)/I_x * q * r + 1/I_x * L;
q_dot = (I_z - I_x)/I_y * p * r + 1/I_y * M;
r_dot = (I_x - I_y)/I_z * p * q + 1/I_z * N;
dOmega_bdt = [p_dot, q_dot, r_dot]';

% Determining translation rates
u_dot = r*v - q*w - g*sin(theta) + 1/m * X;
v_dot = p*w - r*u + g*sin(phi)*cos(theta) + 1/m * Y;
w_dot = q*u - p*v + g*cos(phi)*cos(theta) + 1/m * Z;
dV_bdt = [u_dot, v_dot, w_dot]';

% Tranlating body to inertial coordinates
x_dot =  u * cos(theta)*cos(psi) + ...
         v * (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + ...
         w * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
y_dot =  u * cos(theta)*sin(psi) + ...
         v * (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + ...
         w * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
z_dot = -u * sin(theta) + ...
         v * sin(phi)*cos(theta) + ...
         w * cos(phi)*cos(theta);
dV_Edt = [x_dot, y_dot, z_dot]';
    
% Translating Angular Momentum to Euler angles
phi_dot   = p + (q*sin(phi)+ r*cos(phi))*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot   = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
dEuldt = [phi_dot, theta_dot, psi_dot]';

% Concatenating for output
dfdt = [dV_bdt; dOmega_bdt; dEuldt; dV_Edt];



end