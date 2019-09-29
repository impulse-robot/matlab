function [jump_height] = get_jump_height(mass, length_crank, length_pushrod, number_cells, motor_type, visualize)
%get_jump_height - calculates the jump height

% Inputs:
%    mass - mass without battery and motor              [rad]
%    length_crank - length of crank                     [m]
%    length_pushrod - length of push rod                [m]
%    number_cells - number of cells of the battery      []
%    motor_type - motor type (0 = A80-9, 1 = A80-6)     []
%
% Outputs:
%    jump_height - jump height as definded in docs      [m]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

% time step
t_d = 0.00001;    % [s]
gravity = 9.81; % [m/s^2]
max_acceleration = 200; % [m/s^2]
has_contact = true;

% motor type
if (motor_type == 0)         % A80-9
    R = 0.175;               % [Ohm]
    kv = 105;                % [RPM/V]
    kt = 0.091;              % [Nm/A] 
    t_max = 2;               % [Nm] (max motor torque)
    mass_motor = 0.555;      % [kg]
    gear_reduction = 9;      % []
    
elseif (motor_type == 1)    % A80-6
    R = 0.175;               % [Ohm]
    kv = 105;                % [RPM/V]
    kt = 0.091;              % [Nm/A]
    t_max = 2;               % [Nm]
    mass_motor = 0.462;      % [kg]
    gear_reduction = 6;      % []
end
    
% battery
% (https://www.swaytronic.ch/LiPo-Akku---Swaytronic/LiPo-Akku-6S-22-2V-251/75c-150c-616/sway-fpv-lipo-6s-22-2v-450mah-75c-150c-xt30.html)
u_cell = 3.8;                % [V]
n_cell = number_cells;       % []
mass_battery = 0.082;        % [kg]

% total mass
mass_total = mass + mass_battery + mass_motor;

% max leg extension
max_extension = length_crank + length_pushrod;

% Initial angle fully retracted leg
theta_0 = -pi/2 + 0.4;
theta_dot_0 = 0;


% initial condition  
z_0 = -forward_kinematics(theta_0, length_crank, length_pushrod);
z_dot_0 = 0;
z_dot_dot_0 = 0;

t = [0];                    % time [s]
z = [z_0];                  % vertical position [m]
z_dot = [z_dot_0];          % vertical velocity [m/s]
z_dot_dot = [z_dot_dot_0];  % vertical acceleration [m/s^2]
theta = [theta_0];          % output shaft angle [rad]
theta_dot = [theta_dot_0];  % output shaft angular velocity [rad/s]
jacobian = [0];             % jacobain
foot_force = [0];           % force exerted on the ground [N]
joint_torque = [0];         % joint torque [Nm]
joint_torque_desired = [0]; % desired joint torque [Nm]
power = [0];                % power consumption [W]             

i = 1;

while true
    
    if (i > 100000)
        disp('-- [get_jump_height] WARNING: Exceeded max iteration');
        break
    end
   
    
    jacobian_next = get_jacobian(theta(i), length_crank, length_pushrod);
    

    
    % account for small jacobians at start and finish
%     if (abs(jacobian_next) < 0.01)
%         jacobian_next = -0.01;
%     end
    
    % solve differential equations
    if (has_contact)
        % calculate desired torque based on max acceleration
        foot_force_desired = mass_total * max_acceleration;
        joint_torque_desired_next = -foot_force_desired * jacobian_next;
        motor_torque_desired = joint_torque_desired_next / gear_reduction;
        
        % run through motor model
        joint_torque_next = motor_torque(motor_torque_desired, theta_dot(i) * gear_reduction, u_cell, n_cell, R, kv, kt, t_max) * gear_reduction;
        foot_force_next = joint_torque_next / jacobian_next;
    else
        joint_torque_desired_next = 0;
        joint_torque_next = 0;
        foot_force_next = 0;
    end
    
    z_dot_dot_next =  -foot_force_next / mass_total - gravity;
    z_dot_next = z_dot(i) + t_d * z_dot_dot_next;
    z_next = z(i) + t_d * z_dot_next;
    
    % check for take off
    if (z_next > max_extension)
        if (has_contact == true)
%             disp('-- [get_jump_height] INFO: Take off');
        end
        take_off_time = i * t_d;
        has_contact = false;
    end
    
    % update joint angle and velocity
    if (has_contact)
        theta_dot_next = -z_dot_next / jacobian_next;
        
        z_f = - z_next;  % tranform body position to leg extension in base frame
        theta_next = inverse_kinematics(z_f, length_crank, length_pushrod);
    else
        theta_dot_next = 0;
        theta_next = theta(i);
    end
    
    % calculate output power
    power_next = joint_torque_next * theta_dot_next;
    
    % store results
    t = [t, i * t_d];
    joint_torque = [joint_torque, joint_torque_next];
    joint_torque_desired = [joint_torque_desired, joint_torque_desired_next];
    power = [power, power_next];
    foot_force = [foot_force, foot_force_next];
    jacobian = [jacobian, jacobian_next];
    z_dot_dot = [z_dot_dot, z_dot_dot_next];
    z_dot = [z_dot, z_dot_next];
    z = [z, z_next];
    theta_dot = [theta_dot, theta_dot_next];
    theta = [theta, theta_next];
    

    if (z_dot(i) < 0)
        break
    end
    
    i = i + 1;
    
end

%% visualization
if (visualize)
    % jump height
    figure('name', 'Jump Height Analysis')
    subplot(4, 1, 1);
    plot(t, z, 'LineWidth', 3);
    hold on;
    title('Jump Height')
    ylabel('z [m]');
    xlabel('time [s]');
    grid on;

    subplot(4, 1, 2)
    plot(t, z_dot, 'LineWidth', 3);
    hold on;
    title('Velocity')
    ylabel('v [m/s]');
    xlabel('time [s]');
    grid on;

    subplot(4, 1, 3)
    plot(t, foot_force, 'LineWidth', 3);
    title('Foot force')
    ylabel('F [N]');
    xlabel('time [s]');
    grid on;
    
    subplot(4, 1, 4)
    plot(t, z_dot_dot, 'LineWidth', 3);
    title('Acceleration')
    ylabel('a [m/s^2]');
    xlabel('time [s]');
    grid on;   
    

    % actuator
    figure('name', 'Actuator Analysis')
    subplot(4, 1, 1)
    plot(t, theta, 'LineWidth', 3);
    hold on;
    title('Angular Position')
    ylabel('theta [rad]');
    xlabel('time [s]');
    grid on;

    subplot(4, 1, 2)
    plot(t, theta_dot, 'LineWidth', 3);
    hold on;
    title('Angular Velocoity')
    ylabel('theta_dot [rad/s]');
    xlabel('time [s]');
    grid on;

    subplot(4, 1, 3)
    plot(t, joint_torque, 'LineWidth', 3);
    hold on;
    title('Joint Torque')
    ylabel('tau [Nm]');
    xlabel('time [s]');
    grid on;

    subplot(4, 1, 4)
    plot(t, power, 'LineWidth', 3);
    hold on;
    title('Power')
    ylabel('P [W]');
    xlabel('time [s]');
    grid on;
    
   % testing
    figure('name', 'Testing')
    plot(t, joint_torque_desired, 'LineWidth', 3);
    hold on;
    title('Desired joint torque')
    ylabel('Torque [Nm]');
    xlabel('time [s]');
    grid on;
    
end

jump_height = z(end);


end
