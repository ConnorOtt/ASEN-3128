%--------------------------------------------------------------------------

%Assignment 2: Quad Copter ODE45 stopper

%sets a stopping point for ODE45 so it doesn't not iterate past 0 (ground
%level)

%Created on 1/31/2018 by Ryan Stewart
%Edited on 1/31/2018 by Ryan Stewart

%--------------------------------------------------------------------------

function [ value,isterminal,direction ] = Stop_Quad( t,state)
%command checks that the copter is not going "below" the ground during the
%integration
value  = state(12); 
%If condition is met, and copter is going past 0 in the positive direction
%(negative Z is positive down so positive in this coordinate frame means
%the ball is going through the ground in the XYZ frame) these commands will
%stop ode45 from continuing integration

isterminal = 1;   
direction  = 1;
end