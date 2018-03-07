%--------------------------------------------------------------------------

%Assignment 2: Quadcopter simulation DCM function
%Takes in Quad copter orientation and generates DCM in order to convert
%between body and inertial coordinates

%Created on 1/31/2018 by Ryan Stewart
%Edited on 1/31/2018 by Ryan Stewart

%--------------------------------------------------------------------------

function [ L_eb ] = DCM_B2In(Attitude)

%Plug values into DCM formula in order to eliminate clutter in the main
%script
L_eb = [cosd(Attitude(2))*cosd(Attitude(1)) sind(Attitude(3))*sind(Attitude(2))*cosd(Attitude(1))-cosd(Attitude(3))*sind(Attitude(1)) cosd(Attitude(3))*sind(Attitude(2))*cosd(Attitude(1))+sind(Attitude(3))*sind(Attitude(1));...
    cosd(Attitude(2))*sind(Attitude(1)) sind(Attitude(3))*sind(Attitude(2))*sind(Attitude(1))+cosd(Attitude(3))*cosd(Attitude(1)) cosd(Attitude(3))*sind(Attitude(2))*sind(Attitude(1))-sind(Attitude(3))*cosd(Attitude(1));...
    -sind(Attitude(2)) sind(Attitude(3))*cosd(Attitude(2)) cosd(Attitude(3))*cosd(Attitude(2))];

end

