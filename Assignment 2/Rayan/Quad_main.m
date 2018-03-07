%--------------------------------------------------------------------------

%Assignment 2: Quadcopter simulation main script
%Sets initial constants and provides workspace for numerical integration of
%the equations of motions for Roller Spider Quadcopter

%Created on 1/30/2018 by Ryan Stewart
%Edited on 1/4/2018 by Ryan Stewart

%--------------------------------------------------------------------------

%Housekeeping

clearvars;close all;clc

%define initial constants for the vehicle

g=[0;0;9.81]; %[m/s^2]
mass=0.068; %[kg]
L_arm=0.06; %[m]
I_x=6.8e-5; %[Kg*m^2]
I_y=9.2e-5; %[kg*m^2]
I_z=1.35e-4; %[kg*m^2]
I_mat = [I_x 0 0;0 I_y 0;0 0 I_z]; %I tensor, may not be needed but idk yet


k=.0024; %[m] k constant for control moment equations
r=L_arm*cosd(45); %[m] r constant for lever arm distance for moments

nu=1e-3; %[N/(m/s)^2]
zeta=3e-3; %[N/(m/s)^2]
alpha=2e-6; %[(N*m)/(rad/s)^2]
beta=1e-6; %[(N*m)/(rad/s)^2]

PHI_steady = fzero(@(phi) ((cosd(phi)^2)/sind(phi))-((mass*9.81)/(25*nu)),10);
Theta_steady =fzero(@(theta) ((cosd(theta)^2)/sind(theta))+((mass*9.81)/(25*nu)),-2);


%Initial quad copter state vectors-MUST BE CHANGED BEFORE RUNNING THE OTHER
%TRIM STATES

Pos_inertial=[0;0;-3]; %[m] initial height of 2 m for hover conditions
V_inertial=[0;5;0]; %corresponds to x_e',y_e',z_e' in Earth coordinates
%Attitude = [0;0;0]; %corresponds to psi,theta,phi-azimuth, elevation, bank
Attitude = [0;0;PHI_steady]; %Attitude for the 0 azimuth case
%Attitude = [90;Theta_steady;0]; %Attitude for the 90 azimuth case

%DCM for inertial to body frame coordinates
DCM=DCM_B2In(Attitude);

V_body=DCM\V_inertial; %inertial v in bfc, [u v w]
 
w_EB_body=[0;0;0];%angular velocity of quad copter in bcf, [p q r]

%Rotor thrusts: determined w/ all zero initial conditions and no aero
%forces -STEADY STATE
% coeff=[1 1 1 1; -1 -1 1 1;-1 1 -1 1;-1 1 1 -1];
% b=[mass*g(3); 0;0;0];


%Forces - East @ 5m/s with azimuth zero
coeff_1=mass*g(3)*cosd(PHI_steady)+zeta*(5*sind(PHI_steady))^2;
coeff=[1 1 1 1; -1 -1 1 1;-1 1 -1 1;-1 1 1 -1];
b=[coeff_1; 0;0;0];



%5 m/s East with 90 deg Azimuth

% coeff_1=mass*g(3)*cosd(Theta_steady)+zeta*(5*sind(Theta_steady))^2;
% coeff=[1 1 1 1; -1 -1 1 1;-1 1 -1 1;-1 1 1 -1];
% b=[coeff_1; 0;0;0];

F=coeff\b;

f1=F(1);
f2=F(2);
f3=F(3);
f4=F(4);

%Call control vector to determine control forces on the Quadcopter and set
%gyroscopic couples for the rotors
[F_c,M_c]=control_vec(k,r,f1,f2,f3,f4);


%Initial case vector, order: U_e,V_e,W_e,psi,theta,phi,p,q,r,x_e,y_e,z_e
initial_state=[V_body(1),V_body(2),V_body(3),Attitude(1),Attitude(2),...
    Attitude(3),w_EB_body(1),w_EB_body(2),w_EB_body(3),Pos_inertial(1)...
    Pos_inertial(2),Pos_inertial(3)];

%Call ode45 to numerically integrate equations of motion for the quad
%copter

 Opts=odeset('Events',@Stop_Quad);
[t,state] = ode45(@(t,initial_state) ode_Quad(t,initial_state,I_mat,mass,g(3),F_c,M_c,nu,zeta,alpha,beta),0:0.01:10,initial_state,Opts); %ODE 45 function

%Plot quad copter trajectory and switch axis to NED coordinates
plot3(state(:,10),state(:,11),state(:,12),'-o')
grid on
set(gca, 'Zdir','reverse');
set(gca, 'Ydir','reverse');
axis([-10,10,-10,50,-5,0])

xlabel('North Heading Distance [m]');
ylabel('East Heading Range [m]');
zlabel('Down Heading Altitude [m]');
title('Steady Trim State for Quadcopter')
legend('Quadcopter Trajectory')



%% Experimental Quad Copter data handling


%Basic trajectory plot of the Quad copter position 


%load in data struct from D2L and extract time first
figure
load RSdata_Drone01_1335.mat
time=rt_estim.time(:);

%extract XYZ data from struct and flip z and x to match NED coordinate
%transformation
xdata=-rt_estim.signals.values(:,1);
ydata=rt_estim.signals.values(:,2);
zdata=-rt_estim.signals.values(:,3);
%Plot trajectory and 5 sec time marker to indicate controls free for copter
plot3(xdata,ydata,zdata,'k-','LineWidth',5);
hold on
plot3(xdata(time==5),ydata(time==5),zdata(time==5),'r*','LineWidth',10);
grid on
xlabel('North Position [m]')
ylabel('East Position [m]')
zlabel('Down Position [m]')
title('Experimental Quadcopter Trajectory-Set 1335')
set(gca, 'Zdir','reverse');
set(gca, 'Xdir','reverse');
legend('Copter Trajectory','Controls Free-5 Second Mark')


%Get rotation rates as a function of time all on the same plot along with
%the marker for controls free
roll_Rate=rt_estim.signals.values(:,10);
pitch_Rate=rt_estim.signals.values(:,11);
yaw_Rate=rt_estim.signals.values(:,12);
time=rt_estim.time(:);

figure
plot(time,roll_Rate,time,pitch_Rate,time,yaw_Rate);
hold on
plot(time(time==5),0,'r*','LineWidth',2);

grid on
xlabel('Time [s]')
ylabel('Angular Rate [deg/sec]')
title('Angular Rates wrt Time of Flight')
legend('Roll Rate - P','Pitch Rate - Q', 'Yaw Rate - R','Controls Free - 5 Second Mark')

%Translation vs time plots
figure
plot(time,xdata,time,ydata,time,zdata);
hold on
line([time(time==5) time(time==5)],[-.6 1.2],'LineStyle','--','Color',[1 0 0]) 

grid on
xlabel('Time [s]')
ylabel('Translation Data [m]')
title('Translation wrt Time of Flight')
legend('X-Translation','Y-Translation', 'Z-Translation','Controls Free - 5 Second Mark')

%% Instability plot

%Rerun initial set but this time include small disturbance in rotation and
%show in stable flight, w/o PID, copter is unstable

Pos_inertial=[0;0;-3]; %[m] initial height of 2 m for hover conditions
V_inertial=[.5;.2;0]; %corresponds to x_e',y_e',z_e' in Earth coordinates
Attitude = [1;2;1.3]; %corresponds to psi,theta,phi-azimuth, elevation, bank

%DCM for inertial to body frame coordinates
DCM=DCM_B2In(Attitude);

V_body=DCM\V_inertial; %inertial v in bfc, [u v w]
 
w_EB_body=[.1;.3;.25];%angular velocity of quad copter in bcf, [p q r]

%Coefficients for steady level flight (Derived above)
coeff_1=mass*g(3);
coeff=[1 1 1 1; -1 -1 1 1;-1 1 -1 1;-1 1 1 -1];
b=[coeff_1; 0;0;0];

F=coeff\b;

f1=F(1);
f2=F(2);
f3=F(3);
f4=F(4);

%Call control vector to determine control forces on the Quadcopter and set
%gyroscopic couples for the rotors
[F_c,M_c]=control_vec(k,r,f1,f2,f3,f4);


%Initial case vector, order: U_e,V_e,W_e,psi,theta,phi,p,q,r,x_e,y_e,z_e
initial_state=[V_body(1),V_body(2),V_body(3),Attitude(1),Attitude(2),...
    Attitude(3),w_EB_body(1),w_EB_body(2),w_EB_body(3),Pos_inertial(1)...
    Pos_inertial(2),Pos_inertial(3)];

%Call ode45 to numerically integrate equations of motion for the quad
%copter

 Opts=odeset('Events',@Stop_Quad);
[t,state] = ode45(@(t,initial_state) ode_Quad(t,initial_state,I_mat,mass,g(3),F_c,M_c,nu,zeta,alpha,beta),0:0.01:100,initial_state,Opts); %ODE 45 function

%Plot quad copter trajectory in new state 
figure
plot3(state(:,10),state(:,11),state(:,12),'-o')
grid on
set(gca, 'Zdir','reverse');
set(gca, 'Ydir','reverse');
axis([-10,10,-10,50,-10,0])

xlabel('North Heading Distance [m]');
ylabel('East Heading Range [m]');
zlabel('Down Heading Altitude [m]');
title('Steady Trim State for Quadcopter')
legend('Quadcopter Trajectory-Initial Perturbations')


