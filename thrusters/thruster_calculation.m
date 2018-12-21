%% Torque Calculation for Thrusters
% Author:   Jan Hinder
% Date: 11/12/2018

% In this script, the required torques for roll, pitch and yaw thrusters
% are calculated. Moreover, with the provided thrust of the chosen motors,
% the position of the thrusters is calculated.

close, clc, clear

% The robot is modeled as a rectangular solid with a longer side in z
% direction, because of the leg. This will affect the inertia in pitch
% direction.
%% parameters

m = 0.4; %mass of robot in kg
g = 9.81; %gravity in N/kg

% dimension of robot in x,y and z direction in m
l_x = 0.1;
l_y = l_x;
l_z = 0.4;
% thrusts of the motors in kg
th_F10 = 0.136;
th_F15 = 0.244;
th_uruav = 0.026;

%% moment of inertia in kg*m^2
I_xx = m*(l_y^2+l_z^2)/12; %roll
I_yy = m*(l_x^2+l_z^2)/12; %pitch
I_zz = m*(l_x^2+l_y^2)/12; %yaw

%% angular acceleration in rad/s^2
% constant acceleration until half of the angle, then constant deceleration
t = 0.4; %time of flight in is 1s, time to thrust considered to be 0.2s
theta_1 = 0; % initial angle in rad
theta_2 = pi/4; % angle with highest speed in rad
%omega_avg = (theta_2 - theta_1)/t; % average angular velocity in rad/s
%omega_max = 2*omega_avg; % maximum angular velocity in rad/2


a = 2*(theta_2-theta_1)/t^2; % rad/s^2
a_x = a; % roll
a_y = a; % pitch
a_z = a; % yaw

%% torque needed in Nm
t_x = I_xx*a_x; % roll
t_y = I_yy*a_y; % pitch
t_z = I_zz*a_z; % yaw

% length of poles in m from COG in m
l_pitch = t_y/(th_F10*g); % in either direction on the y=0 plane
l_yaw = t_x/(th_uruav*g*2); % two motors used, distance from z=0 plane
l_roll = t_z/(th_uruav*g*2); % two motors used, distance from x=0 plane



