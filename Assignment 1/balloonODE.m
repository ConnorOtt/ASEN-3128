% Event function for balloon sonde simulation

function [ dfdt ] = balloonODE( t, f )

% Importing number library
nums = balloonNums;
dfdt = zeros(6, 1);

V_g = [f(1), f(2), f(3)]; % Ground Speed [x, y, z] m/s
R = [f(4), f(5), f(6)]; % Positon [x, y, z] m
crossWind = f(7);
volume = f(8);
mass = nums.m + volume*nums.rho_He; % total mass of sonde

% Balloon shape for calculating drag
r = (3/(pi*4) * volume)^(1/3);
A = pi * r^2; 

windx = nums.W_E;
windy = nums.W_N;

% Relative wind speed (x, y, z) m/s
V_rel = [V_g(1) - windx, V_g(2) - windy - crossWind, V_g(3)]; 

head = V_rel/norm(V_rel); % Heading vector (direction of rocket)
D = nums.rho_air/2 * norm(V_rel)^2 * nums.C_D * A * head; % N - Drag force

% Gravity
F_g = [0, 0, -nums.g * mass];

% Bouyant Force
F_b = [0, 0, nums.g * nums.rho_air * volume];

% if norm(F_b) < norm(F_g)
%     dV_gdt = [0, 0, 0];
% else
    % Sum of forces divided by mass to get acceleration
    dV_gdt = (-D + F_g + F_b)/mass;
% end

dRdt = V_g; % Vector of Ground Speed [x, y, z] m/s


dfdt(1:3) = dV_gdt;
dfdt(4:6) = dRdt;
dfdt(7) = 0;
dfdt(8) = 0;                                                                                         
end

