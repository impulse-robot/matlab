%% Analyse leg kinematics
% Author:   Alex Dietsche
% Date: 10/11/2018

%% Parameters
% (from Design Exploration and Kinematic Tuning of a Power
%  Modulating Jumping Monopod paper)
close all; clear; clc

A = [-0.024330, -0.015363];
B = [0, 0]; % origin
C = [0.038243, 0.011044];
D = [0.008843, -0.001726];
F = [-0.007718, 0.000153];
G = [0.064137, 0.0245630];
H = [0.043186, 0.0247990];
K = [0.029413, 0.008778];
L = [-0.04621, -0.067287];
M = [-0.056208, -0.067095];
P_0 = [0.00014, -0.07271];

% Link lengths
a_1 = norm(F-B);
a_2 = norm(D-B);
b_1 = norm(L-G);
b_2 = norm(C-H);
b_3 = norm(L-C);
c_1 = norm(K-A);
c_2 = norm(C-A);
d_1 = norm(L-M);
d_2 = norm(P_0-L);
l_0 = norm(B-A);
l_1 = norm(H-F);
l_2 = norm(M-K);
l_3 = norm(G-D);
link_lengths = [a_1, a_2, b_1, b_2, b_3, c_1, c_2, d_1, d_2, ... 
    l_0, l_1, l_2, l_3];

% Angles
alpha_1 = acos(dot((D-B), (F-B)) / (norm(D-B) * norm(F-B)));
beta_1 = acos(dot((C-H), (L-C)) / (norm(C-H) * norm(L-C)));
beta_2 = acos(dot((C-H), (L-G)) / (norm(C-H) * norm(L-G)));
gamma_1 = acos(dot((K-A), (C-A)) / (norm(K-A) * norm(C-A)));
delta_1 = acos(dot((M-L), (L-P_0)) / (norm(M-L) * norm(L-P_0)));
epsilon_0 = atan(A(2) / A(1));
link_angles = [alpha_1, beta_1, beta_2, gamma_1, delta_1, epsilon_0];

% Initial angle
t_1 = atan2(F(2), F(1));
l = H-F;
t_2_0 = atan2(l(2), l(1));
l = C-H;
t_3_0 = atan2(l(2), l(1));
l = M-L;
t_4_0 = atan2(l(2), l(1));
l = K-M;
t_5_0 = atan2(l(2), l(1));
l = A-K;
t_6_0 = atan2(l(2), l(1));
l = G-D;
t_7_0 = atan2(l(2), l(1));

% Loop closure equations
syms t_1 t_2 t_3 t_4 t_5 t_6 t_7
eqns = [
    a_1 * cos(t_1) + l_1 * cos(t_2) + b_2 * cos(t_3) + ... 
    c_2 * cos(t_6 - gamma_1) + l_0 * cos(epsilon_0) == 0
    a_1 * sin(t_1) + l_1 * sin(t_2) + b_2 * sin(t_3) + ... 
    c_2 * sin(t_6 - gamma_1) + l_0 * sin(epsilon_0) == 0
    a_1 * cos(t_1) + l_1 * cos(t_2) + b_2 * cos(t_3) + ... 
    b_3 * cos(t_3 - beta_1) + d_1 * cos(t_4) + l_2 * cos(t_5) + ...
    c_1 * cos(t_6) + l_0 * cos(epsilon_0) == 0
    a_1 * sin(t_1) + l_1 * sin(t_2) + b_2 * sin(t_3) + ... 
    b_3 * sin(t_3 - beta_1) + d_1 * sin(t_4) + l_2 * sin(t_5) + ... 
    c_1 * sin(t_6) + l_0 * sin(epsilon_0) == 0
    a_2 * cos(t_1 + alpha_1) + l_3 * cos(t_7) + ...
    b_1 * cos(t_3 - beta_2) + d_1 * cos(t_4) + l_2 * cos(t_5) + ... 
    c_1 * cos(t_6) + l_0 * cos(epsilon_0) == 0
    a_2 * sin(t_1 + alpha_1) + l_3 * sin(t_7) + ... 
    b_1 * sin(t_3 - beta_2) + d_1 * sin(t_4) + l_2 * sin(t_5) + ...
    c_1 * sin(t_6) + l_0 * sin(epsilon_0) == 0
    ];

vars = [t_2 t_3 t_4 t_5 t_6 t_7];
init_guess = [t_2_0, t_3_0, t_4_0, t_5_0, t_6_0, t_7_0];


n = 30;  % number of evaluations
upper_angle_limit = 3.05; % 3.122
lower_angle_limit = -0.95; % 1.5
input_angles = linspace(upper_angle_limit, lower_angle_limit, n);
joint_angles = zeros(n, 7);

disp('-- Solving loop closure equations...')

tic

for i = 1:numel(input_angles)
    % solve loop closure equations
    input_angle = input_angles(i);
    [theta_2, theta_3, theta_4, theta_5, theta_6, theta_7] = ... 
        vpasolve(subs(eqns, t_1, input_angle), vars, init_guess);
    
    % check if no solution was found
    if (isempty(theta_2) || isempty(theta_3) || isempty(theta_4) || ... 
            isempty(theta_5) || isempty(theta_6) || isempty(theta_7))
        error('No solution found for input angle: %.3f', input_angle)
    end
    
    % store solution
    joint_angles(i, :) = [input_angle, theta_2, theta_3, theta_4, ...
        theta_5, theta_6, theta_7];
    
    % update initial guess for next iteration
    init_guess(1) = double(theta_2);
    init_guess(2) = double(theta_3);
    init_guess(3) = double(theta_4);
    init_guess(4) = double(theta_5);
    init_guess(5) = double(theta_6);
    init_guess(6) = double(theta_7);
    
end

t = toc;

fprintf('-- Finished in %.2fs \n', t)

% calculate pivots
pivots = calculate_pivots(joint_angles, link_lengths, link_angles);
y = pivots(:, 11, 2);

% visualize pivots
visualize_leg(pivots)

% approximate jacobian
jacobian = diff(y) ./ diff(input_angles.');


% write forward kinematics to file
forward_kinematics_file = 'forward_kinematics.csv';
fk_header = ['theta',',', 'y'];
fid = fopen(forward_kinematics_file, 'w');
fprintf(fid, '%s\r', fk_header);
fclose(fid);
dlmwrite(forward_kinematics_file, [input_angles.', y],'-append','delimiter',',','roffset', 0,'coffset',0);

% write jacobian to file
differential_kinematics_file = 'differential_kinematics.csv';
dk_header = ['theta', ',', 'J(theta)'];
fid = fopen(differential_kinematics_file, 'w');
fprintf(fid, '%s\r', dk_header);
fclose(fid);
dlmwrite(differential_kinematics_file, [input_angles(1:end-1).', jacobian],'-append','delimiter',',','roffset', 0,'coffset',0);

% write leg geometry to file
leg_geometry_file = 'leg_geometry.csv';
geometry_header = ['x', ',', 'y'];
fid = fopen(leg_geometry_file, 'w');
fprintf(fid, '%s\r', geometry_header);
fclose(fid);
dlmwrite(leg_geometry_file, [A; B; C; D; F; G; H; K; L; M; P_0],'-append','delimiter',',','roffset', 0,'coffset',0);
