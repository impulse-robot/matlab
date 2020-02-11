function [mass,inertia] = get_cylinder_mass_inertia(length, radius)
%GET_CYLINDER_MASS_INERTIA - Calculate the mass and rotational moment of
%intertia for a aluminium cylinder
%
% Inputs:
%    length - length of cylinder                [m]
%    radius - radius of cylinder                [m]
%
% Outputs:
%    mass  - total mass of cylinder             [kg]
%    inertia - rotational moment of inertia     [kg m^2]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

aluminium_density = 2700;                                   % [kg/m^3]

mass = pi * radius^2 * length * aluminium_density;
inertia = 1 / 12 * mass * length^2;

end

