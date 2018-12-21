%% Analyse joint forces
% Author:   Alex Dietsche
% Date: 19/12/2018

%% Parameters

close all; clear; clc

tau = 0.2; % torque applied throughout the stroke [Nm]
i = 25; % transmission []

if ~isfile('../data/leg_geometry_stroke.csv') 
    disp('-- Did not find leg_geometry file')
    disp('-- Running leg_kinemtics script...')
    run('../leg_kinematics/leg_kinematics.m')
    close all; clear; clc
    disp('-- leg_kinematics script finished.')
end

leg_geometry = csvread('../data/leg_geometry_stroke.csv', 1, 0);
A = [leg_geometry(:, 1), leg_geometry(:, 2)];
B = [leg_geometry(:, 3), leg_geometry(:, 4)];
C = [leg_geometry(:, 5), leg_geometry(:, 6)];
D = [leg_geometry(:, 7), leg_geometry(:, 8)];
F = [leg_geometry(:, 9), leg_geometry(:, 10)];
G = [leg_geometry(:, 11), leg_geometry(:, 12)];
H = [leg_geometry(:, 13), leg_geometry(:, 14)];
K = [leg_geometry(:, 15), leg_geometry(:, 16)];
L = [leg_geometry(:, 17), leg_geometry(:, 18)];
M = [leg_geometry(:, 19), leg_geometry(:, 20)];
P_0 = [leg_geometry(:, 21), leg_geometry(:, 22)];

theta_1 = leg_geometry(:, 23);
theta_2 = leg_geometry(:, 24);
theta_3 = leg_geometry(:, 25);
theta_4 = leg_geometry(:, 26);
theta_5 = leg_geometry(:, 27);
theta_6 = leg_geometry(:, 28);
theta_7 = leg_geometry(:, 29);

%% Calculate joint forces

syms F_f F_d F_bx F_by F_cx F_cy F_lx F_ly F_k F_ax F_ay f;

theta_1 = theta_1(1800);
theta_2 = theta_2(1800);
theta_3 = theta_3(1800);
theta_4 = theta_4(1800);
theta_5 = theta_5(1800);
theta_6 = theta_6(1800);
theta_7 = theta_7(1800);

F_x = F(1800, 1);
F_y = F(1800, 2);
D_x = D(1800, 1);
D_y = D(1800, 2);
C_x = C(1800, 1) - L(1800, 1); 
C_y = C(1800, 2) - L(1800, 2);
H_x = H(1800, 1) - L(1800, 1);
H_y = H(1800, 2) - L(1800, 2);
G_x = G(1800, 1) - L(1800, 1);
G_y = G(1800, 2) - L(1800, 2);
K_x = K(1800, 1) - A(1800, 1);
K_y = K(1800, 2) - A(1800, 2);
C_ax = C(1800, 1) - A(1800, 1);
C_ay = C(1800, 2) - A(1800, 2);
M_x = M(1800, 1) - L(1800, 1);
M_y = M(1800, 2) - L(1800, 2);
P_x = P_0(1800, 1) - L(1800, 1);
P_y = P_0(1800, 2) - L(1800, 2);

% crank
eqn_crank_1 = - i * tau - F_f * cos(theta_2) * F_y + F_f * sin(theta_2) * F_x - ...
    F_d * cos(theta_7) * D_y + F_d * sin(theta_7) * D_x == 0;
eqn_crank_2 = F_f * cos(theta_2) + F_d * cos(theta_7) + F_bx == 0;
eqn_crank_3 = F_f * sin(theta_2) + F_d * sin(theta_7) + F_by == 0;

% shin
eqn_shin_1 = -F_cx * C_y + F_cy * C_x + F_f * cos(theta_2) * H_y - ...
    F_f * sin(theta_2) * H_x + F_d * cos(theta_7) * G_y - ... 
    F_d * sin(theta_7) * G_x == 0; 
eqn_shin_2 = F_lx + F_cx - F_f * cos(theta_2) - F_d * cos(theta_7) == 0;
eqn_shin_3 = F_ly + F_cy - F_f * sin(theta_2) - F_d * sin(theta_7) == 0;

% rod
eqn_rod_1 = -F_k * cos(theta_5) * K_y + F_k * sin(theta_5) * K_x + ... 
    F_cx * C_ay - F_cy * C_ax == 0;
eqn_rod_2 = F_ax + F_k * cos(theta_5) - F_cx == 0;
eqn_rod_3 = F_ay + F_k * sin(theta_5) - F_cy == 0;


% foot
eqn_foot_1 = F_k * cos(theta_5) * M_y - F_k * sin(theta_5) * M_x + ...
    P_x * f == 0;
eqn_foot_2 = -F_lx - F_k * cos(theta_5) == 0;
eqn_foot_3 = -F_ly - F_k * sin(theta_5) + f == 0;

% solve equations
eqns = [ eqn_crank_1, eqn_crank_2, eqn_crank_3, ...
    eqn_shin_1, eqn_shin_2, eqn_shin_3, ...
    eqn_rod_1, eqn_rod_2, eqn_rod_3, ...
    eqn_foot_1, eqn_foot_2, eqn_foot_3];

vars = [F_f F_d F_bx F_by F_cx F_cy F_lx F_ly F_k F_ax F_ay f];

sol = solve(eqns, vars);





