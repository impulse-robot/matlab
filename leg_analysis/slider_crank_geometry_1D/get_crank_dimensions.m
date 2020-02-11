function [outer_diameter, inner_diameter] = get_crank_dimensions(length, motor_torque, safety)
% get_crank_dimensions - Calculates the dimension of the crank link to 
%                        endure the motor torque with the given safety
%                        factor
%
% Inputs:
%    length - length of link                            [m]
%    motor_torque - output torque of motor              [Nm]
%    safety - safety factor                             []
%
% Outputs:
%    outer_diameter - outer diameter of carbon tube     [m]
%    inner_diameter - inner diameter of carbon tube     [m]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

% carbon fiber properties
compressive_strength = 425 * 10^6;          % [N/m^2]
shear_strength = 90 * 10^6;                 % [N/m^2]

% thickness is assumed to be 1mm
t = 1 * 10^-3;                              % [m]

% carbon tube thickness based on bending stress
syms r_o
eqn_bending_stress = compressive_strength == (motor_torque * r_o) /  (pi / 4 * (r_o^4 - (r_o - t)^4)) * safety;
S = solve(eqn_bending_stress, r_o);
r_o_bending_stress = max(double(vpa(S)));

% carbon tube thickness based on shear stress
shear_force = motor_torque / length;
eqn_shear_stress = shear_strength == (4 * shear_force) / (3 * pi * (r_o^2 - (r_o - t)^2)) * (r_o^2 + r_o * (r_o - t) + (r_o - t)^2) / (r_o^2 + (r_o - t)^2) * safety;
S = solve(eqn_shear_stress, r_o);
r_o_shear_stress = max(double(vpa(S)));

r_o = max(r_o_bending_stress, r_o_shear_stress);

outer_diameter = r_o;
inner_diameter = r_o - t;


end