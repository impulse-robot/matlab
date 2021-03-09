% Author: Alex Dietsche
% Date: 12/02/2020

% In this script, the required torques for roll, pitch and yaw thrusters
% are calculated. Moreover, with the provided thrust of the chosen motors,
% the position of the thrusters is calculated.
close all; clear; clc;

%% Model configuration
if (~isfile('../leg_analysis/slider_crank_geometry_1D/data/model_parameter_optimized_80.mat'))
    disp('Optimized model parameter are not available... run leg_optimization!');
else
    load('../leg_analysis/slider_crank_geometry_1D/data/model_parameter_optimized_80.mat');
end

%% Calculate moment of inertia

% Assumptions:
% Pitch - Leg is fully extended (max inertia in y direction)
% Roll  - Leg is fully extended (max inertia in x direction)
% Yaw   - Theta = 0 (max inertia in z direction)

% Helper angle for pushrod inertia around zz
alpha = asin(mp_opt.length_crank / mp_opt.length_pushrod);

% Body
I_xx_b = mp_opt.inertia_body;       % assume body to be cube
I_yy_b = mp_opt.inertia_body;
I_zz_b = mp_opt.inertia_body;

% Crank
I_xx_c = mp_opt.inertia_crank;      % assuming leg is fully extended (worst case for roll)
I_yy_c = mp_opt.inertia_crank;      % assuming leg is fully extended (worst case for pitch)
I_zz_c = mp_opt.inertia_crank;      % assuming leg is at theta = 0 (worst case for yaw)

% Pushrod
I_xx_p = mp_opt.inertia_pushrod;    % assuming leg is fully extended (worst case for roll)
I_yy_p = mp_opt.inertia_pushrod;    % assuming leg is fully extended (worst case for pitch)
I_zz_p = sin(alpha) * mp_opt.inertia_pushrod;    % assuming leg is at theta = 0 (worst case for yaw)

% Foot
I_xx_f = mp_opt.inertia_foot;       % assuming leg is fully extended (worst case for roll)
I_yy_f = mp_opt.inertia_foot;       % assuming leg is fully extended (worst case for pitch)
I_zz_f = 0;                         % assuming leg is at theta = 0 (worst case for yaw)

% Parallel axis theorem to move inertia to CoG of body

I_xx_c = I_xx_c + mp_opt.mass_crank * (mp_opt.length_crank / 2)^2;
I_xx_p = I_xx_p + mp_opt.mass_pushrod * (mp_opt.length_crank + mp_opt.length_pushrod / 2)^2;
I_xx_f = I_xx_f + mp_opt.mass_foot * (mp_opt.length_crank + mp_opt.length_pushrod + mp_opt.length_foot / 2)^2;

I_yy_c = I_xx_c;    % robot in extended leg config has the same ineratia around pitch and roll axis
I_yy_p = I_xx_p;
I_yy_f = I_xx_p;

I_zz_c = I_zz_c + mp_opt.mass_crank * (mp_opt.length_crank / 2)^2;
I_zz_p = I_zz_p + mp_opt.mass_pushrod * (mp_opt.length_crank / 2)^2;

% Total moment of inertia
I_xx_tot = I_xx_b + I_xx_c + I_xx_p + I_xx_f;
I_yy_tot = I_yy_b + I_yy_c + I_yy_p + I_yy_f;
I_zz_tot = I_zz_b + I_zz_c + I_zz_p + I_zz_f;

%% Torque requirements during flight phase

% Assumptions:
% Pitch - 120 deg turn in 0.5s
% Roll  - 80 deg turn in 0.5s
% Yaw   - no requirements

t = 0.35;                           % flight time
d_pitch = deg2rad(180);             % desired pitch angle difference during flight phase
d_roll = deg2rad(120);              % desired roll angle difference during flight phase

% Pitch
req_acc_pitch = d_pitch / t^2;      % required angular acceleration pitch [rad/s^2] (1/2 * acc * t^2 = d_angle / 2)
req_torque_pitch = I_yy_tot * req_acc_pitch;

% Roll
req_acc_roll = d_roll / t^2;        % required angular acceleration roll [rad/s^2]
req_torque_roll = I_xx_tot * req_acc_roll;


%% Torque requirements fall recovery

l_b = 0.08;         % length of rollover bar [m]

g_acc = 9.81;

l_d = mp_opt.length_pushrod - mp_opt.length_crank;
l_l = l_d + mp_opt.length_foot;
l_ff = mp_opt.length_foot / 2;
l_fb = l_l;
l_fr = mp_opt.length_foot + mp_opt.length_pushrod / 2;
l_fc = l_l + mp_opt.length_crank / 2;
phi = atan(l_b / l_l);

req_torque_standup = (mp_opt.mass_crank * l_fc + mp_opt.mass_pushrod * l_fr + ...
        mp_opt.mass_body * l_fb + mp_opt.mass_foot * l_ff) * g_acc * cos(phi);


%% Required thrust forces

l_p = 0.1;
l_r = 0.08;

req_thrust_pitch_flight = (req_torque_pitch / l_p) / 9.81

req_thrust_pitch_standup = (req_torque_standup / (l_p + l_l)) / 9.81

req_thrust_roll_flight = (req_torque_roll / l_r) / 9.81
    
% phi_alt = atan(mp_opt.length_crank / (mp_opt.length_)
