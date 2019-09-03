function [jump_height] = get_jump_height(mass, length_crank, length_pushrod, length_eccentricity, number_cells, motor_type)
%get_jump_height - calculates the jump height

% Inputs:
%    mass - mass without battery and motor              [rad]
%    length_crank - length of crank                     [m]
%    length_pushrod - length of push rod                [m]
%    length_eccentricity - eccentric off set            [m]
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
t_d = 0.001;    % [s]

% motor type
if (motor_type == 0)         % A80-9
    R = 0.175;               % [Ohm]
    kv = 105;                % [RPM/V]
    kt = 0.091;              % [Nm/A] 
    t_max = 18;              % [Nm]
    mass_motor = 0.555;      % [kg]
    gear_reduction = 9;      % []
    
else if (motor_type == 1)    % A80-6
    R = 0.175;               % [Ohm]
    kv = 105;                % [RPM/V]
    kt = 0.091;              % [Nm/A]
    t_max = 12;              % [Nm]
    mass_motor = 0.462;      % [kg]
    gear_reduction = 6;      % []
    end
    
% battery
% (https://www.swaytronic.ch/LiPo-Akku---Swaytronic/LiPo-Akku-6S-22-2V-251/75c-150c-616/sway-fpv-lipo-6s-22-2v-450mah-75c-150c-xt30.html)
u_cell = 3.8;                % [V]
n_cell = number_cells;       % []
mass_battery = 0.082;        % [kg]

% Initial angle fully retracted leg
theta_0 = -pi/2 + asin(le/lr);
theta_dot_0 = 0;


% initial condition  
z_0 = forward_kinematics(theta_0, length_crank, length_pushrod, length_eccentricity);
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
    
    % solve differential equations
    z_dot_dot_next = 
    
    
    
    if (z_dot(j) < 0)
        break
    end
    
end



end
