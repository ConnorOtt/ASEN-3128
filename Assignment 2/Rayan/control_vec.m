%--------------------------------------------------------------------------

%Assignment 2: Quadcopter simulation control separation function
%takes in rotor thrusts and generates the control vectors for the quad
%copter, reduces clutter in main script

%Created on 1/30/2018 by Ryan Stewart
%Edited on 1/30/2018 by Ryan Stewart

%--------------------------------------------------------------------------

function [ F_c,M_c ] = control_vec( k,r,f1,f2,f3,f4 )
%Forces
X_c=0;
Y_c=0;
Z_c=-f1-f2-f3-f4;

%Moments
L_c=r*(f2+f3-f1-f4);
M_c=r*(-f1-f2+f3+f4);
N_c=k*(f2+f4-f1-f3);

%generate outputs
F_c=[X_c;Y_c;Z_c];
M_c=[L_c,M_c,N_c];

end

