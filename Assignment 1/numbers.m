classdef numbers
   properties (Constant)
       m = 0.05;   % [kg]
       d = 0.03;   % [m]
       C_D = 0.5;  % []
       rho = 1.0;  % [kg/m^3]
       g = 9.81;   % [m/s^2]
       V_xi = 20;  % [m/s] x initial
       V_zi = 20;  % [m/s] Up initial
   end
   properties (SetAccess  = immutable)
       A            % [m^2]
   end
   properties (SetAccess = public)
       windVec = [0, -5, 0];
   end
   methods
       function obj = numbers()
           obj.A = pi * obj.d^2 / 4;
       end
   end
end