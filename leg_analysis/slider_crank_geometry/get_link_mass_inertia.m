function [mass, inertia] = get_link_mass_inertia(length, outer_diameter, inner_diameter)
% get_link_mass - Calculates the mass and momenent of inertia of link based in its length
%
% Inputs:
%    length - length of link                            [m]
%    outer_diameter - outer diameter of carbon tube     [m]
%    inner_diameter - inner diameter of carbon tube     [m]
%
% Outputs:
%    mass  - total mass of link                         [kg]
%    inertia - rotational moment of inertia             [kg m^2]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

%% mass and inertia of carbon tube
carbon_density = 1650;          % [kg/m^3]

mass_carbon_tube = pi * (outer_diameter^2 - inner_diameter^2) * length * carbon_density;
inertia_carbon_tube = 1 /12 * mass_carbon_tube * ( 3 * (outer_diameter^2 + inner_diameter^2) + length^2);

%% mass and inertia of aluminium joints
% the shape of the aluminium joints is assumed to be a tube with a cover
aluminium_density = 2700;                                   % [kg/m^3]
thickness_joint = 2 * 10^-3;                                % [m]
length_joint = 15 * 10^-3;                                  % [m]
thickness_cover = 10 * 10^-3;                                % [m]
inner_diameter_joint = outer_diameter;
outer_diameter_joint = inner_diameter_joint + thickness_joint;
mass_joint = (pi * (outer_diameter_joint^2 - inner_diameter_joint^2) * length_joint + pi * outer_diameter^2 * thickness_cover) * aluminium_density;

% the joints are assumed to be point masses at the tip of the carbon tube
inertia_joint = mass_joint * (length/2)^2;

mass = mass_carbon_tube + 2 * mass_joint;
inertia = inertia_carbon_tube + 2 * inertia_joint;

end