%% Leg optimization
% Author:   Alex Dietsche
% Date: 7/01/2020

close all; clear; clc;

%% CONFIG

% visualize jump height for different configurations and jump curves for
% optimal configuration
visualize = true;


% will run optimization, if false will load optimization results if there
% are any
run_optimization = false;

% calculate joint forces for optimal configuration
calc_joint_forces = true;

% store optimization results
store_results = false;

%% Model Parameters
length_body = 0.08;
length_foot = 0.05;
radius_foot = 0.007;
max_leg_extension = 0.25;
mass_body_empty = 0.6;
safety_factor = 3;


%% Define symbolic expressions
syms tau theta theta_dot real
syms l_f l_r l_c real positive
syms m_f m_r m_c m_b real positive
syms I_f I_r I_c I_b real positive
syms t real positive


%% Forward kinematics

% Positions in inertial frame I
I_r_f = [0;
    l_f/2];

I_r_r = [-l_c/2 * cos(theta);
    l_f + l_r/2 * sqrt(1 - (l_c/l_r * cos(theta))^2)];

I_r_c = [-l_c/2 * cos(theta);
    l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c/2 * sin(theta)];

I_r_b = [0;
    l_f + l_r * sqrt(1 - (l_c/l_r * cos(theta))^2) + l_c * sin(theta)];


%% Jacobians

% Positional jacobians
J_P_f = jacobian([I_r_f(1); I_r_f(2)], theta);
J_P_r = jacobian([I_r_r(1); I_r_r(2)], theta);
J_P_c = jacobian([I_r_c(1); I_r_c(2)], theta);
J_P_b = jacobian([I_r_b(1); I_r_b(2)], theta);

jacobian_body = matlabFunction(J_P_b(2), 'vars', {'theta', 'l_c', 'l_r'});

% Rotational jacobians
J_R_f = 0;
J_R_r = -(l_c / l_r * sin(theta)) / (sqrt(1 - (l_c / l_r * cos(theta))^2));
J_R_c = 1;
J_R_b = 1;


%% Lagrange method

% Generalized coordinates
q = theta;
q_dot = theta_dot;

% Kinetic energy
T = 1 / 2 * (J_P_r.' * m_r * J_P_r + ...
    J_P_c.' * m_c * J_P_c + ...
    J_P_b.' * m_b * J_P_b + ...
    J_R_r.' * I_r * J_R_r + ...
    J_R_c.' * I_c * J_R_c) * q_dot^2;

% Potential energy
g_acc = 9.81;
U = (I_r_r(2) * m_r + I_r_c(2) * m_c + I_r_b(2) * m_b) * g_acc;

% Lagrange equation
L = T - U;

% create time dependent symfun
qt = symfun('theta(t)', t);
qt_dot = symfun('theta_dot(t)', t);

L_q = diff(L, q);
L_qdot = diff(L, q_dot);

L_q(t) = subs(L_q, [q, q_dot], [qt, qt_dot]);
L_qdot(t) = subs(L_qdot, [q, q_dot], [qt, qt_dot]);

L_qdot_t(t) = diff(L_qdot, t);

L_terms = L_qdot_t - L_q;

L_terms = simplify(subs(L_terms, qt_dot, diff(qt, t)));

% Define equation of motion
EoM = L_terms == tau;


%% Optimization parameters
N = 80;                      % square root of crank-pushrod combinations []
battery_types = [0 1 2];
motor_types = [0 1];


% generate crank - pushrod combinations for optimization
length_crank_min = 0.02;    % [m]
length_crank_max = 0.25;    % [m]

length_pushrod_min = 0.03;  % [m]
length_pushrod_max = 0.3;   % [m]

length_combinations = zeros(N^2, 2);
crank_lengths_lin = linspace(length_crank_min, length_crank_max, N)';
crank_lengths_square = repmat(crank_lengths_lin, 1, N)';
crank_lengths = crank_lengths_square(:)';

length_combinations(:,1) = crank_lengths;


pushrod_lengths_lin = linspace(length_pushrod_min, length_pushrod_max, N)';
pushrod_lengths_square = repmat(pushrod_lengths_lin, 1, N);
pushrod_lengths = pushrod_lengths_square(:)';

M = size(motor_types, 2);
B = size(battery_types, 2);

jump_heights_square = zeros(N, N, B, M);
jump_heights_valid = zeros(N, N, B, M);


if (run_optimization)
    
    %% Optimization
    
    ppm = ParforProgressbar(N, 'showWorkerProgress', true);
    
    parfor (i = 1:N, 6)
        for j = 1:N
            for m = 1:M
                for b = 1:B
                    crank_length = crank_lengths_square(j, i);
                    pushrod_length = pushrod_lengths_square(j, i);
                    if (pushrod_length <= crank_length)
                        % non valid geometry
                        jump_heights_square(j, i, b, m) = 0;
                    else
                        
                        % valid geometry
                        mp_local = get_model_parameters(crank_length, pushrod_length, length_body, ...
                            length_foot, radius_foot, (m-1), (b-1), ...
                            mass_body_empty, safety_factor, -1);
                        
                        % max torque function
                        get_max_torque = @(theta_dot) motor_torque(mp_local.motor_max_torque / mp_local.motor_gear_reduction, ...
                            theta_dot * mp_local.motor_gear_reduction, mp_local.battery_u_cell, mp_local.battery_n_cell, ...
                            mp_local.motor_R, mp_local.motor_KV, mp_local.motor_KT, mp_local.motor_max_torque / mp_local.motor_gear_reduction) ...
                            * mp_local.motor_gear_reduction;
                        
                        
                        % Equation of motion
                        EoM_local = subs(EoM, [l_f, l_r, l_c, m_f, m_r, m_c, m_b, I_f, I_r, I_c, I_b], ...
                            [mp_local.length_foot, mp_local.length_pushrod, mp_local.length_crank, mp_local.mass_foot, ...
                            mp_local.mass_pushrod, mp_local.mass_crank, mp_local.mass_body, mp_local.inertia_foot, ...
                            mp_local.inertia_pushrod, mp_local.inertia_crank, mp_local.inertia_body]);
                        
                        % Transform to system of 1. order ODE
                        [V] = odeToVectorField(EoM_local);
                        
                        % Transfrom to matlab function
                        dydtdt = matlabFunction(V(2), 'vars', {'t', 'Y', 'tau'});
                        crank_slider_ode = @(t, Y) [Y(2); dydtdt(t, Y, get_max_torque(Y(2)))];
                        
                        % Define termination event
                        opts = odeset('Events', @angle_limit);
                        
                        % Solve ODE
                        theta_0 = -pi/2;
                        theta_dot_0 = 0;
                        sol = ode15s(crank_slider_ode, [0 1], [theta_0 theta_dot_0], opts);
                        
                        % Results
                        t = sol.x;
                        theta = sol.y(1, :);
                        theta_dot = sol.y(2, :);
                        
                        % Calculate body velocity
                        body_velocity = jacobian_body(theta, mp_local.length_crank, mp_local.length_pushrod) .* theta_dot;
                        
                        % Detect take off when body reaches max velocity
                        [max_velocity, ~] = max(body_velocity);
                        jump_height = 1 / (2*g_acc) * max_velocity^2;
                        
                        jump_heights_square(j, i, b, m) = jump_height;
                        
                    end
                end
            end
        end
        % Increase progress bar
        ppm.increment();
        
    end
    
    delete(ppm);
    
    
    %% Optimized model parameters
    jump_heights_valid = get_valid_jump_heights(jump_heights_square, crank_lengths_square, pushrod_lengths_square, max_leg_extension);
    
    mp_opt = get_optimal_model_parameter(jump_heights_valid, crank_lengths_square, pushrod_lengths_square, length_body, length_foot, radius_foot, mass_body_empty, safety_factor);
    
    if (store_results)
        save(strcat('model_parameter_optimized_', num2str(N), '.mat'), 'mp_opt');
        save(strcat('jump_heights_square_N_', num2str(N),'.mat'), 'jump_heights_square');
        save(strcat('jump_heights_valid_N_', num2str(N),'.mat'), 'jump_heights_valid');
    end
    
    print_optimal_model_parameter(mp_opt);
    
    
else   % not running optimization
    
    % load optimized data
    disp('Reading optimized model parameters from file');
    load(strcat('model_parameter_optimized_', num2str(N), '.mat'), 'mp_opt');
    
    disp('Reading jump heights from file');
    load(strcat('jump_heights_square_N_', num2str(N),'.mat'), 'jump_heights_square');
    
    % Shorter leg extension
    jump_heights_valid = get_valid_jump_heights(jump_heights_square, crank_lengths_square, pushrod_lengths_square, max_leg_extension);
    mp_opt = get_optimal_model_parameter(jump_heights_valid, crank_lengths_square, pushrod_lengths_square, length_body, length_foot, radius_foot, mass_body_empty, safety_factor);
    
    print_optimal_model_parameter(mp_opt);
    
    
    
end

%% Run optimal model for visualization



% visualize jump heights for different model configs
visualize_jump_heights(crank_lengths_square, pushrod_lengths_square, jump_heights_square, max_leg_extension, mp_opt);


% max torque function
get_max_torque = @(theta_dot) motor_torque(mp_opt.motor_max_torque / mp_opt.motor_gear_reduction, ...
    theta_dot * mp_opt.motor_gear_reduction, mp_opt.battery_u_cell, mp_opt.battery_n_cell, ...
    mp_opt.motor_R, mp_opt.motor_KV, mp_opt.motor_KT, mp_opt.motor_max_torque / mp_opt.motor_gear_reduction) ...
    * mp_opt.motor_gear_reduction;


% Equation of motion
EoM_opt = subs(EoM, [l_f, l_r, l_c, m_f, m_r, m_c, m_b, I_f, I_r, I_c, I_b], ...
    [mp_opt.length_foot, mp_opt.length_pushrod, mp_opt.length_crank, mp_opt.mass_foot, ...
    mp_opt.mass_pushrod, mp_opt.mass_crank, mp_opt.mass_body, mp_opt.inertia_foot, ...
    mp_opt.inertia_pushrod, mp_opt.inertia_crank, mp_opt.inertia_body]);

% Transform to system of 1. order ODE
[V] = odeToVectorField(EoM_opt);

% Transfrom to matlab function
dydtdt = matlabFunction(V(2), 'vars', {'t', 'Y', 'tau'});
crank_slider_ode = @(t, Y) [Y(2); dydtdt(t, Y, get_max_torque(Y(2)))];

% Define termination event
opts = odeset('Events', @angle_limit);

% Solve ODE
theta_0 = -pi/2;
theta_dot_0 = 0;
sol = ode15s(crank_slider_ode, [0 1], [theta_0 theta_dot_0], opts);

% Results
t = sol.x;
theta = sol.y(1, :);
theta_dot = sol.y(2, :);

% Calculate body velocity
body_velocity = jacobian_body(theta, mp_opt.length_crank, mp_opt.length_pushrod) .* theta_dot;

% Calculate torque
torque = arrayfun(get_max_torque, theta_dot);

% Detect take off when body reaches max velocity
[max_velocity, idx_max_velocity] = max(body_velocity);
jump_height_opt = 1 / (2*g_acc) * max_velocity^2;

% Cut solution arrays
t = t(1:idx_max_velocity);
theta = theta(1:idx_max_velocity);
theta_dot = theta_dot(1:idx_max_velocity);
body_velocity = body_velocity(1:idx_max_velocity);
torque = torque(1:idx_max_velocity);
power = torque .* theta_dot;
foot_force = 1./jacobian_body(theta, mp_opt.length_crank, mp_opt.length_pushrod) .* torque;

% Interpolate solution
t_lin = linspace(t(1), t(end), 300);
y_lin = deval(sol, t_lin);
theta_lin = y_lin(1, :);
theta_dot_lin = y_lin(2, :);
body_velocity_lin = jacobian_body(theta_lin, mp_opt.length_crank, mp_opt.length_pushrod) .* theta_dot_lin;
torque_lin = arrayfun(get_max_torque, theta_dot_lin);
power_lin = torque_lin .* theta_dot_lin;
foot_force_lin = 1./jacobian_body(theta_lin, mp_opt.length_crank, mp_opt.length_pushrod) .* torque_lin;



if (calc_joint_forces)

    [joint_forces_crank_pushrod, joint_forces_body_crank, ...
        joint_forces_body_linear_guide, joint_forces_pushrod_linear_guide, ...
        joint_forces_body_external] = ...
        get_joint_forces(t_lin, torque_lin, theta_lin, mp_opt);
    
    y_upper_limit = 300;
    y_lower_limit = -300;
    
    figure('name', 'Joint Forces')
    subplot(3, 2, 1);
    plot(t_lin, joint_forces_body_crank, 'LineWidth', 3);
    hold on;
    title('Joint Forces on Body by Crank');
    ylabel('F [N]');
    xlabel('time [s]');
    ylim([y_lower_limit, y_upper_limit]);
    grid on;
    
    subplot(3, 2, 2)
    plot(t_lin, joint_forces_crank_pushrod, 'LineWidth', 3);
    hold on;
    title('Joint Forces on Crank by Pushrod');
    ylabel('F [N]');
    xlabel('time [s]');
    ylim([y_lower_limit, y_upper_limit]);
    grid on;
    
    subplot(3, 2, 3)
    plot(t_lin, joint_forces_body_linear_guide, 'LineWidth', 3);
    hold on;
    title('Joint Forces on Body by Linear Guide');
    ylabel('F [N]');
    xlabel('time [s]');
    ylim([y_lower_limit, y_upper_limit]);
    grid on;
    
    subplot(3, 2, 4)
    plot(t_lin, joint_forces_pushrod_linear_guide, 'LineWidth', 3);
    hold on;
    title('Joint Forces on Pushrod by Linear Guide');
    ylabel('F [N]');
    xlabel('time [s]');
    ylim([y_lower_limit, y_upper_limit]);
    grid on;
    
    subplot(3, 2, 5)
    plot(t_lin, joint_forces_body_external, 'LineWidth', 3);
    hold on;
    title('Joint Forces on Body by External');
    ylabel('F [N]');
    xlabel('time [s]');
    ylim([y_lower_limit, y_upper_limit]);
    grid on;
    
    
end

if (store_results)
    % limit foot forces
    foot_force_lin(foot_force_lin>1e6)=1e6;
    foot_forces = [t_lin; foot_force_lin].';
    save(strcat('foot_forces_N_', num2str(N), '.mat'), 'foot_forces');
    
    
    
end

if (visualize)
    visualize_jump_analysis(t, t_lin, theta, theta_lin, theta_dot, theta_dot_lin, ...
        body_velocity, body_velocity_lin, torque, torque_lin, power, power_lin, ...
        foot_force, foot_force_lin, true);
    
    visualize_motion(t_lin, theta_lin, mp_opt);
    
end
