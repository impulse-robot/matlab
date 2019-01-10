%% Analyse joint forces
% Author:   Alex Dietsche
% Date: 19/12/2018

%% Parameters

close all; clear; clc

tau = 0.2; % torque applied throughout the stroke [Nm]
i = 25; % transmission []
theta_0 = 3.0; % starting output angle [rad]
take_off_extension = 0.13; % extension at which no more torque is produced [m]
number_of_evaluations = 60;

if ~isfile('../data/leg_geometry_stroke.csv') 
    disp('-- Did not find leg_geometry file')
    disp('-- Running leg_kinemtics script...')
    run('../leg_kinematics/leg_kinematics.m')
    close all; clear; clc
    disp('-- leg_kinematics script finished.')
end

leg_geometry = csvread('../data/leg_geometry_stroke.csv', 1, 0);

leg_geometry(:, 22) = -leg_geometry(:, 22);

% ignore entries high rpm 
[start_val, idx_start] = min(abs(leg_geometry(:, 23) - theta_0));
leg_geometry = leg_geometry(idx_start:end, :);
[end_val, idx_end] = min(abs(leg_geometry(:, 22) - (leg_geometry(1, 22) + take_off_extension)));
leg_geometry = leg_geometry(1:idx_end, :);

% downsample entries
number_of_entries = size(leg_geometry, 1);
idx = 1:number_of_entries;
idxq = linspace(min(idx), max(idx), number_of_evaluations);
leg_geometry = interp1(idx, leg_geometry, idxq, 'linear');


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

solutions = zeros(number_of_evaluations, 12);

for j = 1:number_of_evaluations

    theta_1_curr = theta_1(j);
    theta_2_curr = theta_2(j);
    theta_3_curr = theta_3(j);
    theta_4_curr = theta_4(j);
    theta_5_curr = theta_5(j);
    theta_6_curr = theta_6(j);
    theta_7_curr = theta_7(j);

    F_x = F(j, 1);
    F_y = F(j, 2);
    D_x = D(j, 1);
    D_y = D(j, 2);
    C_x = C(j, 1) - L(j, 1); 
    C_y = C(j, 2) - L(j, 2);
    H_x = H(j, 1) - L(j, 1);
    H_y = H(j, 2) - L(j, 2);
    G_x = G(j, 1) - L(j, 1);
    G_y = G(j, 2) - L(j, 2);
    K_x = K(j, 1) - A(j, 1);
    K_y = K(j, 2) - A(j, 2);
    C_ax = C(j, 1) - A(j, 1);
    C_ay = C(j, 2) - A(j, 2);
    M_x = M(j, 1) - L(j, 1);
    M_y = M(j, 2) - L(j, 2);
    P_x = P_0(j, 1) - L(j, 1);
    P_y = P_0(j, 2) - L(j, 2);

    % crank
    eqn_crank_1 = - i * tau - F_f * cos(theta_2_curr) * F_y + F_f * sin(theta_2_curr) * F_x - ...
        F_d * cos(theta_7_curr) * D_y + F_d * sin(theta_7_curr) * D_x == 0;
    eqn_crank_2 = F_f * cos(theta_2_curr) + F_d * cos(theta_7_curr) + F_bx == 0;
    eqn_crank_3 = F_f * sin(theta_2_curr) + F_d * sin(theta_7_curr) + F_by == 0;

    % shin
    eqn_shin_1 = -F_cx * C_y + F_cy * C_x + F_f * cos(theta_2_curr) * H_y - ...
        F_f * sin(theta_2_curr) * H_x + F_d * cos(theta_7_curr) * G_y - ... 
        F_d * sin(theta_7_curr) * G_x == 0; 
    eqn_shin_2 = F_lx + F_cx - F_f * cos(theta_2_curr) - F_d * cos(theta_7_curr) == 0;
    eqn_shin_3 = F_ly + F_cy - F_f * sin(theta_2_curr) - F_d * sin(theta_7_curr) == 0;

    % rod
    eqn_rod_1 = -F_k * cos(theta_5_curr) * K_y + F_k * sin(theta_5_curr) * K_x + ... 
        F_cx * C_ay - F_cy * C_ax == 0;
    eqn_rod_2 = F_ax + F_k * cos(theta_5_curr) - F_cx == 0;
    eqn_rod_3 = F_ay + F_k * sin(theta_5_curr) - F_cy == 0;

    % foot
    eqn_foot_1 = F_k * cos(theta_5_curr) * M_y - F_k * sin(theta_5_curr) * M_x + ...
        P_x * f == 0;
    eqn_foot_2 = -F_lx - F_k * cos(theta_5_curr) == 0;
    eqn_foot_3 = -F_ly - F_k * sin(theta_5_curr) + f == 0;

    % solve equations
    eqns = [ eqn_crank_1, eqn_crank_2, eqn_crank_3, ...
        eqn_shin_1, eqn_shin_2, eqn_shin_3, ...
        eqn_rod_1, eqn_rod_2, eqn_rod_3, ...
        eqn_foot_1, eqn_foot_2, eqn_foot_3];

    vars = [F_f F_d F_bx F_by F_cx F_cy F_lx F_ly F_k F_ax F_ay f];
    sol = solve(eqns, vars);
    solutions(j, :) = [eval(sol.F_f), eval(sol.F_d), eval(sol.F_bx), ... 
        eval(sol.F_by), eval(sol.F_cx), eval(sol.F_cy), eval(sol.F_lx), ...
        eval(sol.F_ly), eval(sol.F_k), eval(sol.F_ax), eval(sol.F_ay), ...
        eval(sol.f)];
    
end

% calculate resulting forces in joints
F_F = solutions(:, 1);
F_H = - F_F;
F_D = solutions(:, 2);
F_G = - F_D;
F_B = sqrt(solutions(:, 3).^2 + solutions(:, 4).^2);
F_C = sqrt(solutions(:, 5).^2 + solutions(:, 6).^2);
F_L = sqrt(solutions(:, 7).^2 + solutions(:, 8).^2);
F_K = solutions(:, 9);
F_M = - F_K;
F_A = sqrt(solutions(:, 10).^2 + solutions(:, 11).^2);
F_f = solutions(:, 12);

% calculate max forces in joints
[F_F_max_abs, F_F_max_idx] = max(abs(F_F));
[F_H_max_abs, F_H_max_idx] = max(abs(F_H));
[F_D_max_abs, F_D_max_idx] = max(abs(F_D));
[F_G_max_abs, F_G_max_idx] = max(abs(F_G));
[F_B_max_abs, F_B_max_idx] = max(abs(F_B));
[F_C_max_abs, F_C_max_idx] = max(abs(F_C));
[F_L_max_abs, F_L_max_idx] = max(abs(F_L));
[F_K_max_abs, F_K_max_idx] = max(abs(F_K));
[F_M_max_abs, F_M_max_idx] = max(abs(F_M));
[F_A_max_abs, F_A_max_idx] = max(abs(F_A));
[F_f_max_abs, F_f_max_idx] = max(abs(F_f));

F_F_max = F_F(F_F_max_idx);
F_H_max = F_H(F_H_max_idx);
F_D_max = F_D(F_D_max_idx);
F_G_max = F_G(F_G_max_idx);
F_B_max = F_B(F_B_max_idx);
F_C_max = F_C(F_C_max_idx);
F_L_max = F_L(F_L_max_idx);
F_K_max = F_K(F_K_max_idx);
F_M_max = F_M(F_M_max_idx);
F_A_max = F_A(F_A_max_idx);
F_f_max = F_f(F_f_max_idx);

% visualize forces
figure('name', 'Crank Forces')
subplot(3, 1, 1);
plot(P_0(:, 2), F_F, 'LineWidth', 3);
hold on;
scatter(P_0(F_F_max_idx, 2), F_F_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_F_max), 'N']};
text(P_0(F_F_max_idx, 2), F_F_max, txt, 'FontSize', 12);
title('Joint F')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(3, 1, 2);
plot(P_0(:, 2), F_D, 'LineWidth', 3);
hold on;
scatter(P_0(F_D_max_idx, 2), F_D_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_D_max), 'N']};
text(P_0(F_D_max_idx, 2), F_D_max, txt, 'FontSize', 12);
title('Joint D')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(3, 1, 3);
plot(P_0(:, 2), F_B, 'LineWidth', 3);
hold on;
scatter(P_0(F_B_max_idx, 2), F_B_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_B_max), 'N']};
text(P_0(F_B_max_idx, 2), F_B_max, txt, 'FontSize', 12);
title('Joint B')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

figure('name', 'Shin Forces');
subplot(4, 1, 1);
plot(P_0(:, 2), F_G, 'LineWidth', 3);
hold on;
scatter(P_0(F_G_max_idx, 2), F_G_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_G_max), 'N']};
text(P_0(F_G_max_idx, 2), F_G_max, txt, 'FontSize', 12);
title('Joint G')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(4, 1, 2);
plot(P_0(:, 2), F_H, 'LineWidth', 3);
hold on;
scatter(P_0(F_H_max_idx, 2), F_H_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_H_max), 'N']};
text(P_0(F_H_max_idx, 2), F_H_max, txt, 'FontSize', 12);
title('Joint H')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(4, 1, 3);
plot(P_0(:, 2), F_C, 'LineWidth', 3);
hold on;
scatter(P_0(F_C_max_idx, 2), F_C_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_C_max), 'N']};
text(P_0(F_C_max_idx, 2), F_C_max, txt, 'FontSize', 12);
title('Joint C')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(4, 1, 4);
plot(P_0(:, 2), F_L, 'LineWidth', 3);
hold on;
scatter(P_0(F_L_max_idx, 2), F_L_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_L_max), 'N']};
text(P_0(F_L_max_idx, 2), F_L_max, txt, 'FontSize', 12);
title('Joint L')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;


figure('name', 'Rod Forces')
subplot(3, 1, 1);
plot(P_0(:, 2), F_A, 'LineWidth', 3);
hold on;
scatter(P_0(F_A_max_idx, 2), F_A_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_A_max), 'N']};
text(P_0(F_A_max_idx, 2), F_A_max, txt, 'FontSize', 12);
title('Joint A')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(3, 1, 2);
plot(P_0(:, 2), F_K, 'LineWidth', 3);
hold on;
scatter(P_0(F_K_max_idx, 2), F_K_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_K_max), 'N']};
text(P_0(F_K_max_idx, 2), F_K_max, txt, 'FontSize', 12);
title('Joint K')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(3, 1, 3);
plot(P_0(:, 2), F_C, 'LineWidth', 3);
hold on;
scatter(P_0(F_C_max_idx, 2), F_C_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_C_max), 'N']};
text(P_0(F_C_max_idx, 2), F_C_max, txt, 'FontSize', 12);
title('Joint C')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;


figure('name', 'Foot Forces')
subplot(3, 1, 1);
plot(P_0(:, 2), F_M, 'LineWidth', 3);
hold on;
scatter(P_0(F_M_max_idx, 2), F_M_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_M_max), 'N']};
text(P_0(F_M_max_idx, 2), F_M_max, txt, 'FontSize', 12);
title('Joint M')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(3, 1, 2);
plot(P_0(:, 2), F_L, 'LineWidth', 3);
hold on;
scatter(P_0(F_L_max_idx, 2), F_L_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_L_max), 'N']};
text(P_0(F_L_max_idx, 2), F_L_max, txt, 'FontSize', 12);
title('Joint L')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

subplot(3, 1, 3);
plot(P_0(:, 2), F_f, 'LineWidth', 3);
hold on;
scatter(P_0(F_f_max_idx, 2), F_f_max, 50, 'LineWidth', 2);
txt = {'Max Force:', ['F\_max = ', num2str(F_f_max), 'N']};
text(P_0(F_L_max_idx, 2), F_f_max, txt, 'FontSize', 12);
title('Foot')
ylabel('Force [N]');
xlabel('Stroke [m]');
grid on;

