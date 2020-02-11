%% Slider Crank Model
% Author:   Alex Dietsche
% Date: 1/09/19

close all; clear; clc;


%% Model parameters
global L_F L_R L_C L_B R_F R_O M_F M_R M_C M_B I_F I_R I_C I_B

safety_factor = 3;
motor_torque = 18;

% body
M_B = 0.9;                          % [kg] mass of body (without battery)
L_B = 0.06;                         % [m] body assumed to be a cube, side length
I_B = 1 / 12 * M_B * (2 * L_B^2);   % [kg m^2]
        
% foot 
L_F = 0.05;                         % [m] length of foot
R_F = 0.007;                        % [m] radius of cylindrical foot

% the foot is assumed to be a solid cylinder
[M_F, I_F] = get_cylinder_mass_inertia(L_F, R_F);    % [kg] mass of foot, [kg m^2] moment of interia

% links
L_R = 0.165;                        % [m] length of pushrod
L_C = 0.155;                        % [m] length of crank

[R_O, r_i] = get_crank_dimensions(L_C, motor_torque, safety_factor);
[M_C, I_C] = get_link_mass_inertia(L_C, R_O*2, r_i*2);    % [kg] mass of crank, [kg m^2] moment of interia

% we use the same diameters for the pushrod (same parts, cheaper
% manufacturing)
[M_R, I_R] = get_link_mass_inertia(L_R, R_O*2, r_i*2);    % [kg] mass of pushrod, [kg m^2] moment of interia                   


m_total = M_F + M_R + M_C + M_B;

%% Generate Forward kinematics
syms tau thrust phi theta phi_dot theta_dot real
syms l_f l_r l_c real positive
% syms m_f m_r m_c m_b real positive
% syms I_f I_r I_c I_b real positive


% Positions in inertial frame I
% I_r_f = [-L_F/2 * sin(phi); 
%         L_F/2 * cos(phi)];
% 
% I_r_r = [-L_C/2 * cos(theta) * cos(phi) - (L_F + L_R/2 * sqrt(1 - (L_C/L_R * cos(theta))^2)) * sin(phi);
%         -L_C/2 * cos(theta) * sin(phi) + (L_F + L_R/2 * sqrt(1 - (L_C/L_R * cos(theta))^2)) * cos(phi)];
%     
% I_r_c = [-L_C/2 * cos(theta) * cos(phi) - (L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2) + L_C/2 * sin(theta)) * sin(phi);
%         -L_C/2 * cos(theta) * sin(phi) + (L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2) + L_C/2 * sin(theta)) * cos(phi)];
% 
% I_r_b = [-(L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2) + L_C * sin(theta)) * sin(phi);
%         (L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2) + L_C * sin(theta)) * cos(phi)];

I_r_f = [-l_f/2 * sin(phi); 
        l_f/2 * cos(phi)];

I_r_r = [-l_c/2 * cos(theta) * cos(phi) - (l_f + l_r/2 * sqrt(1 - (l_c/l_r * cos(theta))^2)) * sin(phi);
        -l_c/2 * cos(theta) * sin(phi) + (l_f + l_r/2 * sqrt(1 - (l_c/l_r * cos(theta))^2)) * cos(phi)];
    
I_r_c = [-l_c/2 * cos(theta) * cos(phi) - (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c/2 * sin(theta)) * sin(phi);
        -l_c/2 * cos(theta) * sin(phi) + (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c/2 * sin(theta)) * cos(phi)];

I_r_b = [-(l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c * sin(theta)) * sin(phi);
        (l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c * sin(theta)) * cos(phi)];

    
    
%% Generate Jacobians


% Position jacobians of links
% J_P_f_old = [-l_f / 2 * cos(phi), 0 ; 
%         -l_f / 2 * sin(phi), 0];
%     
% J_P_r_old = [l_c / 2 * cos(theta) * sin(phi) - (l_f + l_r / 2 * sqrt(1 - (l_c / l_r * cos(theta))^2)) * cos(phi), l_c / 2 * sin(theta) * cos(phi) - (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (2 * l_r * sqrt(1 - (l_c / l_r * cos(theta))^2));
%         -l_c / 2 * cos(theta) * cos(phi) - (l_f + l_r / 2 * sqrt(1 - (l_c / l_r * cos(theta))^2)) * sin(phi), l_c / 2 * sin(theta) * sin(phi) + (l_c^2 * sin(theta) * cos(theta) * cos(phi)) / (2 * l_r * sqrt(1 - (l_c / l_r * cos(theta))^2))];
% 
%     
% J_P_c_old = [l_c / 2 * cos(theta) * sin(phi) - (l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c / 2 * sin(theta)) * cos(phi), l_c / 2 * sin(theta) * cos(phi) - (l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) - l_c / 2  * cos(theta) * sin(phi);
%         -l_c / 2 * cos(theta) * cos(phi) - (l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c / 2 * sin(theta)) * sin(phi), l_c / 2 * sin(theta) * sin(phi) + (l_c^2 * sin(theta) * cos(theta) * cos(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c / 2  * cos(theta) * cos(phi)];
%      
% J_P_b_old = [-(l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c * sin(theta)) * cos(phi), (-l_c^2 * sin(theta) * cos(theta) * sin(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) - l_c * cos(theta) * sin(phi);
%         -(l_f + l_r * sqrt(1 - (l_c / l_r * cos(theta))^2) + l_c * sin(theta)) * sin(phi), (l_c^2 * sin(theta) * cos(theta) * cos(phi)) / (l_r * sqrt(1 - (l_c / l_r * cos(theta))^2)) + l_c * cos(theta) * cos(phi)];
%      
    
J_P_f = jacobian([I_r_f(1); I_r_f(2)], [phi, theta]);  
J_P_r = jacobian([I_r_r(1); I_r_r(2)], [phi, theta]);  
J_P_c = jacobian([I_r_c(1); I_r_c(2)], [phi, theta]);  
J_P_b = jacobian([I_r_b(1); I_r_b(2)], [phi, theta]);  


% Rotation jacobians of links
J_R_f = [1 0];

J_R_r = [1, -(L_C / L_R * sin(theta)) / (sqrt(1 - (L_C / L_R * cos(theta))^2))];

J_R_c = [1 1];

J_R_b = [1 0];


%% Lagrange Method
% Kinetic energy
q = [phi; theta];
q_dot = [phi_dot; theta_dot];

T = 1 / 2 * q_dot.' * (J_P_f.' * M_F * J_P_f + J_R_f.' * I_F * J_R_f + J_P_r.' * M_R * J_P_r + J_R_r.' * I_R * J_R_r + J_P_c.' * M_C * J_P_c + J_R_c.' * I_C * J_R_c + J_P_b.' * M_B * J_P_b + J_R_c + J_R_b.' * I_B * J_R_b) * q_dot;

% Potential energy
g_acc = 9.81;                         % [m/s^2]
U = (I_r_f(2) * M_F + I_r_r(2) * M_R + I_r_c(2) * M_C + I_r_b(2) * M_B) * g_acc;

L = T - U;

X = {phi, phi_dot, theta, theta_dot};
Q_i = {0, 0};
Q_e = {thrust, tau};
R = 0;
par = {};
VF = EulerLagrange(L, X, Q_i, Q_e, R, par, 'm', 'crank_slider_equations_of_motion');


%% Solve DE

t_span = [0 10];
Y0 = [0, 0, pi/2 - 0.001, 0];

time_step = 0.0001;

% initial conditions

t0 = 0;
t = [t0];
t_last = t0;

Q0 = [0; 0; -pi/4; 0];
Q = [Q0];
Q_last = Q0;

body_velocity_last = 0;

% pid gains
kp_thrust = 500;
kd_thrust = 15;

while (true)
    
    % control input
    thrusterForce = -kp_thrust * Q_last(1) - kd_thrust * Q_last(2);
    motorTorque = 8;
    
    t_current = t_last + time_step;
    t_last = t_current;
    t = [t, t_current];
    
    Q_current = Q_last + time_step * crank_slider_equations_of_motion(t_current, Q_last, thrusterForce, motorTorque);
    theta_dot_last = Q_last(4);
    Q_last = Q_current;
    Q = [Q, Q_current];
    
    body_velocity_current = norm(get_jacobian_positional_body(Q_current(1), Q_current(3)) * [Q_current(2); Q_current(4)]);
    if ((body_velocity_current - body_velocity_last < -0.00001) && (body_velocity_current > 0.1))
        break
    end

%     if (Q_current(3) > pi/2)
%         break
%     end

    body_velocity_last = body_velocity_current;
    
end


% interpolate for uniform timestep
t_int = t(1):0.01:t(end);
Q_int = interp1(t.', Q.', t_int.');
visualize_motion(t_int.', Q_int);


