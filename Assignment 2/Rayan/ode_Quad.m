%--------------------------------------------------------------------------

%Assignment 2: Quadcopter simulation ode function
%Takes in initial conditions of Quadcopter and integrates over time
%(including aerodynamic forces and moments and control forces and moments)
%to track helicopter position and attitude

%Created on 1/30/2018 by Ryan Stewart
%Edited on 1/30/2018 by Ryan Stewart

%--------------------------------------------------------------------------

function [ state ] = ode_Quad(t,initial_state,I_G,mass,g,F_c,M_c,nu,zeta,alpha,beta)
%Pull initial state conditions from ODE
U_e = initial_state(1);
V_e = initial_state(2);
W_e = initial_state(3);
psi = initial_state(4);
theta = initial_state(5);
phi = initial_state(6);
p = initial_state(7);
q = initial_state(8);
r = initial_state(9);
F_control=F_c;
M_control=M_c;
Ix=I_G(1);
Iy=I_G(2,2);
Iz=I_G(3,3);

%Calculate aerodynamic forces and moments
X_aero=-nu*U_e^2*sign(U_e);
Y_aero=-nu*V_e^2*sign(V_e);
Z_aero=-zeta*W_e^2*sign(W_e);

L_aero=-alpha*p^2*sign(p);
M_aero=-alpha*q^2*sign(q);
N_aero=-beta*r^2*sign(r);

%Force equations in Body Frame coordinates (Equations 4.7,1 on Page 104)

dUeDt=r*V_e-q*W_e+(1/mass)*(F_control(1)+X_aero-mass*g*sind(theta));
dVeDt=p*W_e-r*U_e+(1/mass)*(F_control(2)+Y_aero+mass*g*cosd(theta)*sind(phi));
dWeDt=q*U_e-p*V_e+(1/mass)*(F_control(3)+Z_aero+mass*g*cosd(theta)*cosd(phi));

%Moment equations in Body Frame coordinates (Equations 4.7,2 on Page 104)

dPdt=(1/Iz)*(M_control(1)+L_aero-q*r*(Iz-Iy));
dQdt=(1/Iy)*(M_control(2)+M_aero-p*r*(Ix-Iz));
dRdt=(1/Iz)*(M_control(3)+N_aero-q*p*(Iy-Ix));

%Kinematic equations (Equations 4.7,3 on Page 104)

dPsiDt=(q*sind(phi)+r*cosd(phi))*secd(theta);
dThetaDt=q*cosd(phi)-r*sind(phi);
dPhiDt=p+(q*sind(phi)+r*cosd(phi))*tand(theta);

%Kinematic equations [inertial velocities] (Equations 4.7,4 on Page 104)

DCM=DCM_B2In([psi theta phi]);

Inertial_Velocities=DCM*[U_e;V_e;W_e];

dXedt=Inertial_Velocities(1);
dYedt=Inertial_Velocities(2);
dZedt=Inertial_Velocities(3);

%assign the outputs to be integrated by the program

state=[dUeDt;dVeDt;dWeDt;dPsiDt;dThetaDt;dPhiDt;dPdt;dQdt;dRdt;dXedt;...
    dYedt;dZedt];


end

