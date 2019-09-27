function [jump_height] = get_jump_height(mass, length_crank, length_pushrod, number_cells, motor_type)
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
t_d = 0.0001;    % [s]
gravity = 9.81; % [m/s^2]
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

% Initial angle fully retracted leg
theta_0 = -pi/2;
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
foot_force = [0];           % force exerted on the ground [N]
power = [0];                % power consumption [W]             

i = 1;

while true
    
    if (i > 100000)
        break
    end
   
    
    jacobian = get_jacobian(theta(i), length_crank, length_pushrod);
    % account for small jacobians at start and finish
    if (jacobian < 0.01)
        jacobian = 0.01;
    end
    
    
    % solve differential equations
    if (has_contact)        
        foot_force_current = motor_torque(20, theta_dot(i) * gear_reduction, u_cell, n_cell, R, kv, kt, t_max) * gear_reduction / jacobian;
    else
        foot_force_current = 0;
    end
    
    z_dot_dot_next =  foot_force_current / mass_total - gravity;
    z_dot_next = z_dot(i) + t_d * z_dot_dot_next;
    z_next = z(i) + t_d * z_dot_next;
    
    
    
    if (has_contact)
        theta_dot_next = z_dot_next / jacobian;
        
        z_f = - z_next;  % tranform body position to leg extension in base frame
        theta_next = inverse_kinematics(z_f, length_crank, length_pushrod);
    else
        theta_dot_next = 0;
        theta_next = theta(i);
    end
    
    % store results
    t = [t, i * t_d];
    z_dot_dot = [z_dot_dot, z_dot_dot_next];
    z_dot = [z_dot, z_dot_next];
    z = [z, z_next];
    theta_dot = [theta_dot, theta_dot_next];
    theta = [theta, theta_next];
    
    if (theta_next > pi / 2)
        has_contact = false;
    end
    
    if (z_dot(i) < 0)
        break
    end
    
    i = i + 1;
    
end


plot(t, z);

jump_height = z(end);

end
