% Small number library for balloon simulation

classdef balloonNums
   properties (Constant)
       m = 0.5;         % [kg]
       C_D = 0.5;       % []
       rho_air = 1.225; % [kg/m^3]
       rho_He = 0.164;  % [kg/m^3]
       g = 9.81;        % [m/s^2]
       W_E = 2;         % [m/s] x wind
       W_N = 4;         % [m/s] y wind
   end
   properties (SetAccess  = immutable)
       A            % [m^2]
       r            % [m]
   end
end