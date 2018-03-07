function [ dfdt ] = golfBallODE( t, f )

% Importing number library
nums = numbers;
dfdt = zeros(6, 1);

V_g = [f(1), f(2), f(3)]; % Ground Speed [x, y, z] m/s
R = [f(4), f(5), f(6)]; % Positon [x, y, z] m
windy = f(7);
mass = f(8);

% Relative wind speed (x, y, z) m/s
V_rel = [V_g(1) - 0, V_g(2) - windy, V_g(3) - 0]; 

head = V_rel/norm(V_rel); % Heading vector (direction of rocket)
D = nums.rho/2 * norm(V_rel)^2 * nums.C_D * nums.A * head; % N - Drag force

F_g = [0, 0, -nums.g * mass];

% Sum of forces divided by mass to get acceleration
dV_gdt = (-D + F_g)/mass; 

dRdt = V_g; % Vector of Ground Speed [x, y, z] m/s

% Once the rocket hits the ground, it should no longer travel in the x and
% z directions.
if R(3) <= 0 && t > 2
   dRdt = 0;
end

dfdt(1:3) = dV_gdt;
dfdt(4:6) = dRdt;
dfdt(7) = 0;
dfdt(8) = 0;                                                                                         
end

