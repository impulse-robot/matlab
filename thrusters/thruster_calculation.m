%% Torque Calculation for Thrusters
% Author:   Jan Hinder
% Date: 11/12/2018

% In this script, the required torques for roll, pitch and yaw thrusters
% are calculated. Moreover, with the provided thrust of the chosen motors,
% the position of the thrusters is calculated.

close all; clear; clc;

% The robot is modeled as a rectangular solid with a longer side in z
% direction, because of the leg. This will affect the inertia in pitch and
% roll direction.
%% parameters

m_b = 1.0; % mass of body [kg]
m_l = 0.3; % mass of leg [kg]
g = 9.81; % gravity [N/kg]

% dimensions body [m]
l_x_b = 0.12;
l_y_b = 0.08;
l_z_b = 0.1;

% dimensions leg [m]
l_x_l = 0.015;
l_y_l = 0.015;
l_z_l = 0.3;

d = l_z_l / 2 + l_z_b / 2; % distance COG leg to COG body [m]

% thrusts of the motors [kg]
th_F10 = 0.100; % 0.136
th_F15 = 0.130; %0.244
th_uruav = 0.026;

%% moment of intertia

% moment of inertia body [kg*m^2]
I_xx_b = m_b * (l_y_b^2 + l_z_b^2) / 12; % roll
I_yy_b = m_b * (l_x_b^2 + l_z_b^2) / 12; % pitch
I_zz_b = m_b * (l_x_b^2 + l_y_b^2) / 12; % yaw

% moment of intertia leg [kg*m^s]
I_xx_l = m_l * (l_y_b^2 + l_z_b^2) / 12; % roll
I_yy_l = m_l * (l_x_b^2 + l_z_b^2) / 12; % pitch
I_zz_l = m_l * (l_x_b^2 + l_y_b^2) / 12; % yaw

% parallel axis theorem to move leg intertia in to COG of body
I_xx_l = I_xx_l + m_l * d^2;
I_yy_l = I_yy_l + m_l * d^2;

% total moment of intertia [kg*m^2]
I_xx_tot = I_xx_b + I_xx_l;
I_yy_tot = I_yy_b + I_yy_l;
I_zz_tot = I_zz_b + I_zz_b;

%% torque requirements

% angular acceleration in rad/s^2
% constant acceleration until half of the angle, then constant deceleration

% theta_1 = 0; % initial angle in rad
% theta_2 = pi/4; % angle with highest speed in rad
%omega_avg = (theta_2 - theta_1)/t; % average angular velocity in rad/s
%omega_max = 2*omega_avg; % maximum angular velocity in rad/2

t = 0.35; % time of flight in is 1s, time to thrust considered to be 0.2s

% pitch
d_theta_p = deg2rad(100); % angle difference [rad]
a_y = 2 * d_theta_p / t^2; % angular acceleration [rad/s^2]
t_y = I_yy_tot * a_y;

% roll
d_theta_r = deg2rad(30);
a_x = 2 * d_theta_r / t^2;
t_x = I_xx_tot * a_x;

% yaw
d_theta_y = deg2rad(90);
a_z = 2 * d_theta_y / t^2;
t_z = I_zz_tot * a_z;

%% thruster rod lengths

% F10 + URUAV combination
l_pitch_F10 = t_y/(th_F10*g); % in either direction on the y=0 plane
l_pitch_F15 = t_y/(th_F15*g); % in either direction on the y=0 plane
l_yaw = t_z/(th_uruav*g*2); % two motors used, distance from z=0 plane
l_roll = t_x/(th_uruav*g*2); % two motors used, distance from x=0 plane
rod_angle = atan(l_yaw / l_roll);
l_roll_yaw = sqrt(l_yaw^2 + l_roll^2);


%% visualization
if ~isfile('../leg_analysis/salto_geometry/data/leg_geometry_stroke.csv') 
    disp('-- Did not find leg_geometry file')
    disp('-- Running leg_kinemtics script...(this could take a minute)')
    run('../leg_analysis/salto_geometry/leg_kinematics/leg_kinematics.m')
    close all; clear; clc
    disp('-- leg_kinematics script finished.')
end

leg_geometry = csvread('../leg_analysis/salto_geometry/data/leg_geometry_stroke.csv', 1, 0);

extension = 50; % extension for visualization [%]
P_y = (leg_geometry(end, 22) - leg_geometry(1, 22)) * extension / 100 + leg_geometry(1, 22);
[val, idx] = min(abs(leg_geometry(:, 22) - P_y));

pivots = leg_geometry(idx, 1:22);
visualize_thrusters(pivots, l_pitch_F15, l_roll_yaw, rod_angle);


