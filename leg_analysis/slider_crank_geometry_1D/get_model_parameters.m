function [model_parameters] = get_model_parameters(length_crank,length_pushrod, length_body, length_foot, radius_foot, motor_type, battery_type, mass_body_empty, safety_factor, max_jump_height)
%GET_MODEL_PARAMETERS - Calculate model parameters
%   Input:
%       length_crank        - length of crank       [m]
%       length_pushrod      - length of pushrod     [m]
%       length_lbody        - body length (body assymed to be cube) [m]
%       length_foot         - length of foot        [m]
%       radius_foot         - radius of foot        [m]
%       motor_type          - motor type            [0 - A80-9, 1 - A80-6]
%       battery_type        - battery type          [0 - 4s, 1 - 6s, 2 -8s]
%       body_mass_empty     - body mass (w/o motor, battery and leg)    [kg]
%       safety_factor       - safety factor for carbon type dimensions  []
%       max_jump_height     - max jump height       [m]


% Motor type
if (motor_type == 0)         % A80-9
    R = 0.175;               % [Ohm]
    kv = 105;                % [RPM/V]
    kt = 0.091;              % [Nm/A] 
    t_max = 2;               % [Nm] (max motor torque)
    mass_motor = 0.555;      % [kg]
    gear_reduction = 9;      % []
    max_torque = t_max * gear_reduction; % [Nm]
    
elseif (motor_type == 1)    % A80-6
    R = 0.175;               % [Ohm]
    kv = 105;                % [RPM/V]
    kt = 0.091;              % [Nm/A]
    t_max = 2;               % [Nm]
    mass_motor = 0.462;      % [kg]
    gear_reduction = 6;      % []
    max_torque = t_max * gear_reduction; % [Nm]
end


% Battery type
u_cell = 3.8;
if (battery_type == 0)       % 4s
    n_cell = 4;
    mass_battery = 0.096; % https://www.swaytronic.ch/LiPo-Akku---Swaytronic/LiPo-Akku-4S-14-8V-249/60C---120C-284/sway-fpv-lipo-4s-14-8v-850mah-60c-120c-xt30.html
    
elseif (battery_type == 1)   % 6s
    n_cell = 6;
    mass_battery = 0.156; % https://www.swaytronic.ch/LiPo-Akku---Swaytronic/LiPo-Akku-6S-22-2V-251/35C---70C-295/swaytronic-lipo-6s-22-2v-950mah-35c-70c-ec3.html
    
elseif (battery_type == 2)   % 8s
    n_cell = 8;
    mass_battery = 2 * 0.057; % 2x https://www.swaytronic.ch/LiPo-Akku---Swaytronic/LiPo-Akku-4S-14-8V-249/60C---120C-284/sway-fpv-lipo-4s-14-8v-450mah-60c-120c-xt30.html
end


% body
mass_body = mass_body_empty + mass_motor + mass_battery;
inertia_body = 1 / 12 * mass_body * (2 * length_body^2);   % [kg m^2]

% the foot is assumed to be a solid cylinder
[mass_foot, inertia_foot] = get_cylinder_mass_inertia(length_foot, radius_foot);    % [kg] mass of foot, [kg m^2] moment of interia


[radius_link_outer, radius_link_inner] = get_crank_dimensions(length_crank, max_torque, safety_factor);
[mass_crank, inertia_crank] = get_link_mass_inertia(length_crank, radius_link_outer*2, radius_link_inner*2);    % [kg] mass of crank, [kg m^2] moment of interia

% we use the same diameters for the pushrod (same parts, cheaper
% manufacturing)
[mass_pushrod, inertia_pushrod] = get_link_mass_inertia(length_pushrod, radius_link_outer*2, radius_link_inner*2);    % [kg] mass of pushrod, [kg m^2] moment of interia                   

mass_total = mass_foot + mass_pushrod + mass_crank + mass_body;

%% Model parameters
model_parameters = struct;

% Geometric parameters
model_parameters.length_pushrod = length_pushrod;
model_parameters.length_crank = length_crank;
model_parameters.length_foot = length_foot;
model_parameters.length_body = length_body;
model_parameters.radius_link_outer = radius_link_outer;
model_parameters.radius_foot = radius_foot;

% Mass / Intertia parameters
model_parameters.mass_pushrod = mass_pushrod;
model_parameters.mass_crank = mass_crank;
model_parameters.mass_foot = mass_foot;
model_parameters.mass_body = mass_body;
model_parameters.mass_total = mass_total;

model_parameters.inertia_pushrod = inertia_pushrod;
model_parameters.inertia_crank = inertia_crank;
model_parameters.inertia_foot = inertia_foot;
model_parameters.inertia_body = inertia_body;

% Powertrain parameters
% motor
model_parameters.motor_R = R;
model_parameters.motor_KV = kv;
model_parameters.motor_KT = kt;
model_parameters.motor_gear_reduction = gear_reduction;
model_parameters.motor_max_torque = max_torque;

% battery
model_parameters.battery_n_cell = n_cell;
model_parameters.battery_u_cell = u_cell;

% Max jump height
model_parameters.max_jump_height = max_jump_height;

end

