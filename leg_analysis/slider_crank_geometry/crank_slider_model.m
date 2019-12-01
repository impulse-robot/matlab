%% Slider Crank Model
% Author:   Alex Dietsche
% Date: 1/09/19

close all; clear; clc;


%% Model parameters
global L_F L_R L_C M_F M_R M_C M_B I_F I_R I_C I_B

safety_factor = 8;
motor_torque = 18;

% body
M_B = 0.8;                          % [kg] mass of body (without battery)
L_B = 0.06;                         % [m] body assumed to be a cube, side length
I_B = 1 / 12 * m_b * (2 * l_b^2);   % [kg m^2]
        

% foot 
L_F = 0.05;                         % [m] length of foot
R_F = 0.007;                        % [m] radius of cylindrical foot

% the foot is assumed to be a solid cylinder
[M_F, I_F] = get_cylinder_mass_inertia(l_f, r_f);    % [kg] mass of foot, [kg m^2] moment of interia


% links
L_R = 0.165;                        % [m] length of pushrod
L_C = 0.155;                        % [m] length of crank

[r_o, r_i] = get_crank_dimensions(l_c, motor_torque, safety_factor);
[M_C, I_C] = get_link_mass_inertia(l_c, r_o, r_i);    % [kg] mass of crank, [kg m^2] moment of interia

% we use the same diameters for the pushrod (same parts, cheaper
% manufacturing)
[M_R, I_R] = get_link_mass_inertia(l_r, r_o, r_i);    % [kg] mass of pushrod, [kg m^2] moment of interia                   


m_total = M_F + M_R + M_C + M_B;

%% Forward kinematics
syms phi theta phi_dot theta_dot real
% syms l_f l_r l_c real positive
% syms m_f m_r m_c m_b real positive
% syms I_f I_r I_c I_b real positive

% phi = sym('phi', 'real');
% theta = sym('theta', 'real');
% phi_dot = sym('phi_dot', 'real');
% theta_dot  = sym('theta_dot', 'real');
% l_f  = sym('l_f', 'real');
% I_f  = sym('I_f', 'real');
% m_f  = sym('m_f', 'real');
% l_r  = sym('l_r', 'real');
% I_r  = sym('I_r', 'real');
% m_r  = sym('m_r', 'real');
% l_c  = sym('l_c', 'real');
% I_c  = sym('I_c', 'real');
% m_c  = sym('m_c', 'real');
% I_b  = sym('I_b', 'real');
% m_b  = sym('m_b', 'real');


% Positions in inertial frame I
I_r_f = [-l_f/2 * sin(phi); 
        l_f/2 * cos(phi)];

I_r_r = [-l_c/2 * cos(theta) * cos(phi) - (l_f + l_r/2 * sqrt(1 - (l_c/l_r * cos(theta))^2)) * sin(phi);
        -l_c/2 * cos(theta) * sin(phi) + (l_f + l_r/2 * sqrt(1 - (l_c/l_r * cos(theta))^2)) * cos(phi)];
    
I_r_c = [-l_c/2 * cos(theta) * cos(phi) - (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c/2 * sin(theta)) * sin(phi);
        -l_c/2 * cos(theta) * sin(phi) + (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c/2 * sin(theta)) * cos(phi)];

I_r_b = [-(l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c * sin(theta)) * sin(phi);
        (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c * sin(theta)) * cos(phi)];

% J = jacobian([I_r_f])
    
    
%% Jacobians


% Position jacobians of links
% J_P_f_old = [-l_f / 2 * cos(phi), 0 ; 
%         -l_f / 2 * sin(phi), 0];
%     
% J_P_r_old = [l_c / 2 * cos(theta) * sin(phi) - (l_f + l_r / 2 * sqrt(1 - (l_c / l_r * cos(theta))^2)) * cos(phi), -l_c / 2 * sin(theta) * cos(phi) - (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (2 * l_r * sqrt(1 - (l_c / l_r * cos(theta))^2));
%         -l_c / 2 * cos(theta) * cos(phi) - (l_f + l_r / 2 * sqrt(1 - (l_c / l_r * cos(theta))^2)) * sin(phi), l_c / 2 * sin(theta) * sin(phi) - (l_c^2 * sin(theta) * cos(theta) * cos(phi)) / (2 * l_r * sqrt(1 - (l_c / l_r * cos(theta))^2))];
% 
%     
% J_P_c_old = [l_c / 2 * cos(theta) * sin(phi) - (l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c / 2 * sin(theta)) * cos(phi), -l_c / 2 * sin(theta) * cos(phi) - (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c / 2  * cos(theta) * sin(phi);
%         -l_c / 2 * cos(theta) * cos(phi) - (l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c / 2 * sin(theta)) * sin(phi), l_c / 2 * sin(theta) * sin(phi) - (l_c^2 * sin(theta) * cos(theta) * cos(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c / 2  * cos(theta) * cos(phi)];
%     
% J_P_b_old = [-(l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c * sin(theta)) * cos(phi), (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c * cos(theta) * sin(phi);
%         -(l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c * sin(theta)) * sin(phi), (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c * cos(theta) * cos(phi)];
     
    
J_P_f = jacobian([I_r_f(1); I_r_f(2)], [phi, theta]);  
J_P_r = jacobian([I_r_r(1); I_r_r(2)], [phi, theta]);  
J_P_c = jacobian([I_r_c(1); I_r_c(2)], [phi, theta]);  
J_P_b = jacobian([I_r_b(1); I_r_b(2)], [phi, theta]);  


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

% T_test = double(subs(T, [phi, theta, phi_dot, theta_dot] , [0, 0, 1, 0]));

% Potential energy
g_acc = 9.81;                         % [m/s^2]
U = (I_r_f(2) * m_f + I_r_r(2) * m_r + I_r_c(2) * m_c + I_r_b(2) * m_b) * g_acc;

L = T - U;

% d_L_d_phi_dot = diff(L, phi_dot);
% d_L_d_theta_dot = diff(L, theta_dot(t));
% d_L_d_q_dot = [d_L_d_phi_dot; d_L_d_theta_dot(t)];

% d_L_d_phi = diff(L, phi);
% d_L_d_theta = diff(L, theta);
% d_L_d_q = [d_L_d_phi; d_L_d_theta];

% d_L_d_t = 

X = {phi, phi_dot, theta, theta_dot};
Q_i = {0, 0};
Q_e = {0, 0};
R = 0;
par = {};
% par = {l_f, l_r, l_c, m_f, m_r, m_c, m_b, I_f, I_r, I_c, I_b };
VF = EulerLagrange(L, X, Q_i, Q_e, R, par, 'm', 'crank_slider_equations_of_motion');


%% Solve DE


t_span = [0 2];
Y0 = [0.1, 0, pi/2 , 0];
[t, Y] = ode45(@crank_slider_equations_of_motion, t_span, Y0);


plot(t, Y(1))
