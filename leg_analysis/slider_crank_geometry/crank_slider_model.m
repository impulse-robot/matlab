%% Leg optimization
% Author:   Alex Dietsche
% Date: 1/09/19

close all; clear; clc;


%% Model parameters

safety_factor = 8;
motor_torque = 18;

% body
m_b = 0.8;                          % [kg] mass of body (without battery)
l_b = 0.06;                         % [m] body assumed to be a cube, side length
I_b = 1 / 12 * m_b * (2 * l_b^2);   % [kg m^2]
        

% foot 
l_f = 0.05;                         % [m] length of foot
r_f = 0.007;                        % [m] radius of cylindrical foot

% the foot is assumed to be a solid cylinder
[m_f, I_f] = get_cylinder_mass_inertia(l_f, r_f);    % [kg] mass of foot, [kg m^2] moment of interia


% links
l_r = 0.165;                        % [m] length of pushrod
l_c = 0.155;                        % [m] length of crank

[r_o, r_i] = get_crank_dimensions(l_c, motor_torque, safety_factor);
[m_c, I_c] = get_link_mass_inertia(l_c, r_o, r_i);    % [kg] mass of crank, [kg m^2] moment of interia

% we use the same diameters for the pushrod (same parts, cheaper
% manufacturing)
[m_r, I_r] = get_link_mass_inertia(l_r, r_o, r_i);    % [kg] mass of pushrod, [kg m^2] moment of interia                   


m_total = m_f + m_r + m_c + m_b;

%% Forward kinematics
syms phi theta phi_dot theta_dot

% Positions in inertial frame I
I_r_f = [-l_f/2 * sin(phi); 
        l_f/2 * cos(phi)];

I_r_r = [-l_c/2 * cos(theta) * cos(phi) - (l_f + l_r/2 * sqrt(1 - (l_c/l_r * cos(theta))^2)) * sin(phi);
        -l_c/2 * cos(theta) * sin(phi) + (l_f + l_r/2 * sqrt(1 - (l_c/l_r * cos(theta))^2)) * cos(phi)];
    
I_r_c = [-l_c/2 * cos(theta) * cos(phi) - (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c/2 * sin(theta)) * sin(phi);
        -l_c/2 * cos(theta) * sin(phi) + (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c/2 * sin(theta)) * cos(phi)];

I_r_b = [-(l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c * sin(theta)) * sin(phi);
        (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c * sin(theta)) * cos(phi)];

    
%% Jacobians



% Position jacobians of links
J_P_f = [-l_f / 2 * cos(phi), 0 ; 
        -l_f / 2 * sin(phi), 0];

J_P_r = [l_c / 2 * cos(theta) * sin(phi) - (l_f + l_r / 2 * sqrt(1 - (l_c / l_r * cos(theta))^2)) * cos(phi), -l_c / 2 * sin(theta) * cos(phi) - (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (2 * l_r * sqrt(1 - (l_c / l_r * cos(theta))^2));
        -l_c / 2 * cos(theta) * cos(phi) - (l_f + l_r / 2 * sqrt(1 - (l_c / l_r * cos(theta))^2)) * sin(phi), l_c / 2 * sin(theta) * sin(phi) - (l_c^2 * sin(theta) * cos(theta) * cos(phi)) / (2 * l_r * sqrt(1 - (l_c / l_r * cos(theta))^2))];
    
J_P_c = [l_c / 2 * cos(theta) * sin(phi) - (l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c / 2 * sin(theta)) * cos(phi), -l_c / 2 * sin(theta) * cos(phi) - (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c / 2  * cos(theta) * sin(phi);
        -l_c / 2 * cos(theta) * cos(phi) - (l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c / 2 * sin(theta)) * sin(phi), l_c / 2 * sin(theta) * sin(phi) - (l_c^2 * sin(theta) * cos(theta) * cos(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c / 2  * cos(theta) * cos(phi)];
    
J_P_b = [-(l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c * sin(theta)) * cos(phi), (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c * cos(theta) * sin(phi);
        -(l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c * sin(theta)) * sin(phi), (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c * cos(theta) * cos(phi)];
    

% Rotation jacobians of links
J_R_f = [1 0];

J_R_r = [1, -(l_c / l_r * sin(theta)) / (sqrt(1 - (l_c / l_r * cos(theta))^2))];

J_R_c = [1 1];

J_R_b = [1 0];


%% Lagrange Method
% Kinetic energy
q = [phi; theta];
q_dot = [phi_dot; theta_dot];

T = 1 / 2 * q_dot.' * (J_P_f.' * m_f * J_P_f + J_R_f.' * I_f * J_R_f + J_P_r.' * m_r * J_P_r + J_R_r.' * I_r * J_R_r + J_P_c.' * m_c * J_P_c + J_R_c.' * I_c * J_R_c + J_P_b.' * m_b * J_P_b + J_R_c + J_R_b.' * I_b * J_R_b) * q_dot;

T_test = double(subs(T, [phi, theta, phi_dot, theta_dot] , [0, 0, 1, 0]));

% Potential energy
g_acc = 9.81;                         % [m/s^2]
U = (I_r_f(2) * m_f + I_r_r(2) * m_r + I_r_c(2) * m_c + I_r_b(2) * m_b) * g_acc;

L = T - U;

d_L_d_phi_dot = diff(L, phi_dot);
d_L_d_theta_dot = diff(L, theta_dot);
d_L_d_q_dot = [d_L_d_phi_dot; d_L_d_theta_dot];

d_L_d_phi = diff(L, phi);
d_L_d_theta = diff(L, theta);
d_L_d_q = [d_L_d_phi; d_L_d_theta];

% d_L_d_t = 



