%% Thruster optimization
% Author:   Alex Dietsche
% Date: 22/05/20

close all; clear; clc;

%% Model parameters
load('data/model_parameter_optimized_80.mat');

length_thruster_pitch = 0.1;
length_thruster_roll = 0.08;
length_thruster_yaw = 0.05;


%% PITCH thruster requirements for fall recovery
l_b = sym('l_b');                                       % radius of rollover bar

l_d = mp_opt.length_pushrod - mp_opt.length_crank;      % length difference
l_l = l_d + mp_opt.length_foot;                         % length leg retracted
l_ff = mp_opt.length_foot / 2;                          % length CoM foot
l_fb = l_l;                                             % length CoM body
l_fr = mp_opt.length_foot + mp_opt.length_pushrod / 2;  % length CoM pushrod
l_fc = l_l + mp_opt.length_crank / 2;                   % length CoM crank

phi = atan(l_b / l_l);                                  % angle of robot lying in rollover bar

m_c = mp_opt.mass_crank;                                % mass of crank
m_r = mp_opt.mass_pushrod;                              % mass of pushrod
m_b = mp_opt.mass_body;                                 % mass of body
m_f = mp_opt.mass_foot;                                 % mass of foot

g = 8.91;                                               % gravity

min_rollover_bar_length = 0.0;
max_rollover_bar_lenght = 0.2;
rollover_bar_lengths = linspace(min_rollover_bar_length, max_rollover_bar_lenght);
T_fun = (m_c * l_fc + m_r * l_fr + m_b * l_fb + m_f * l_ff) * cos(phi) * g;

% Rollover bar length
rollover_bar_length = 0.06;

T_required = eval(subs(T_fun, l_b, rollover_bar_length));
thrust_required_fall_recovery = (T_required / (length_thruster_pitch +  l_l)) / g; % [kg]

% torque requirements for various rollover bar lengths
T = eval(subs(T_fun, l_b, rollover_bar_lengths));
thrusts = (T / (length_thruster_pitch +  l_l)) / g;

% visualize
plot(rollover_bar_lengths, thrusts, 'LineWidth', 3);
hold on
plot(rollover_bar_length, thrust_required_fall_recovery, 'or', 'LineWidth', 5);
xlabel('Rollover bar length [m]')
ylabel('Required Thrust [kg]')
title('Torque for fall recovery')
grid on

%% thruster requirements for jumping

% 180 degrees turn with leg fully extended with convervative jump height
jump_height = 0.3;
delta_phi_pitch = pi;           % 180 degrees
delta_phi_roll = 2 * pi / 3;    % 120 degrees
delta_phi_yaw = pi / 2;         % 90 degrees

flight_time = 2 * sqrt(2 * g * jump_height) / g;

% for visualization
jump_heights = linspace(0.2, 1.5);
flight_times = 2 * sqrt(2 * g * jump_heights) / g;

figure
plot(jump_heights, flight_times, 'LineWidth', 3);
hold on
plot(jump_height, flight_time, 'ro', 'LineWidth', 5);
xlabel('Jump height [m]')
ylabel('Flight time [s]')
title('Flight times')
grid on

phi_dot_dot_pitch = 4 * delta_phi_pitch / (flight_time)^2;
phi_dot_dot_roll = 4 * delta_phi_roll / (flight_time)^2;
phi_dot_dot_yaw = 4 * delta_phi_yaw / (flight_time)^2;

I_pitch_roll = mp_opt.inertia_body + (mp_opt.inertia_crank + mp_opt.mass_crank * (mp_opt.length_crank / 2)^2) + ...
    (mp_opt.inertia_pushrod + mp_opt.mass_pushrod * (mp_opt.length_crank + mp_opt.length_pushrod / 2)^2) + ...
    (mp_opt.inertia_foot + mp_opt.mass_foot * (mp_opt.length_crank + mp_opt.length_pushrod + mp_opt.length_foot / 2)^2);

I_yaw = mp_opt.inertia_body + (mp_opt.inertia_crank + mp_opt.mass_crank * (mp_opt.length_crank / 2)^2) + ...
    (mp_opt.inertia_pushrod + mp_opt.mass_pushrod * (mp_opt.length_pushrod / 2)^2);

T_required_flight_pitch = I_pitch_roll * phi_dot_dot_pitch;
T_required_flight_roll = I_pitch_roll * phi_dot_dot_roll;
T_required_flight_yaw = I_yaw * phi_dot_dot_roll;

thrust_required_flight_pitch = (T_required_flight_pitch / (length_thruster_pitch)) / g;
thrust_required_flight_roll = ((T_required_flight_roll / (length_thruster_roll)) / g) / 2;
thrust_required_flight_yaw = ((T_required_flight_yaw / (length_thruster_yaw)) / g) / 2;

%% Results
disp('################### RESULTS #####################');
disp('MODEL PARAMETERS');
disp(['PITCH MOUNTING LENGTH: ', num2str(length_thruster_pitch), ' [m]']);
disp(['ROLL MOUNTING LENGTH: ', num2str(length_thruster_roll), ' [m]']);
disp(['YAW MOUNTING LENGTH: ', num2str(length_thruster_yaw), ' [m]']);
fprintf('\n');
disp('FALL RECOVERY PARAMETERS');
disp(['ROLLOVER BAR LENGTH: ', num2str(rollover_bar_length), ' [m]']);
fprintf('\n');
disp('FLIGHT STABILIZATION PARAMETERS');
disp(['JUMP HEIGHT: ', num2str(jump_height), ' [m]']);
disp(['FLIGHT TIME: ', num2str(flight_time), ' [s]']);
disp(['DELTA PHI PITCH: ', num2str(rad2deg(delta_phi_pitch)), ' [deg]']);
disp(['DELTA PHI ROLL: ', num2str(rad2deg(delta_phi_roll)), ' [deg]']);
disp(['DELTA PHI YAW: ', num2str(rad2deg(delta_phi_yaw)), ' [deg]']);
fprintf('\n');
disp('FALL RECOVERY');
disp(['== PITCH THRUST FALL RECOVERY: ', num2str(thrust_required_fall_recovery), '[kg]']);
fprintf('\n');
disp('FLIGHT STABILIZATION');
disp(['== PITCH THRUST FLIGHT: ', num2str(thrust_required_flight_pitch), '[kg]']);
disp(['== ROLL THRUST FLIGHT: ', num2str(thrust_required_flight_roll), '[kg]']);
disp(['== YAW THRUST FLIGHT: ', num2str(thrust_required_flight_yaw), '[kg]']);
disp('##################### END #######################');

