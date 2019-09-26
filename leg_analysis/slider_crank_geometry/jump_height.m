%% Jump height
% Author:   Alex Dietsche
% Date: 1/09/19

close all; clear; clc

mass = 1;
length_crank = 0.15;
length_pushrod = 0.35;
number_cells = 4;
motor_type = 0;

thetas = linspace(-pi/2, pi/2, 1000);
jacobians = zeros(1000, 0);

% i = 1;
% for theta = thetas
%     jacobians(i) = get_jacobian(theta, length_crank, length_pushrod);
%     i = i + 1;
% end
% 
% plot(thetas, jacobians);


% R = 0.175;               % [Ohm]
% kv = 105;                % [RPM/V]
% kt = 0.091;              % [Nm/A] 
% t_max = 2;              % [Nm]
% mass_motor = 0.555;      % [kg]
% gear_reduction = 9;      % []
% u_cell = 3.8;
% n_cell = 6.3;
% 
% theta_dots = linspace(0, 300, 1000);
% motor_torques = zeros(1000, 0);
% j = 1;
% for theta_dot = theta_dots
%     motor_torques(j) = motor_torque(5, theta_dot, u_cell, n_cell, R, kv, kt, t_max);
%     j = j + 1;
% end
% 
% theta_dots = theta_dots * 60 / (2* pi);
% 
% theta_dots = theta_dots / gear_reduction;
% motor_torques = motor_torques * gear_reduction;
% 
% plot(theta_dots, motor_torques);

j_height = get_jump_height(mass, length_crank, length_pushrod, number_cells, motor_type)