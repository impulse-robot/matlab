%% Analyse joint forces
% Author:   Alex Dietsche
% Date: 20/01/2019

%% Parameters

close all; clear; clc

if (~isfile('../data/leg_geometry_stroke.csv'))
    disp('-- Did not find leg_geometry file')
    disp('-- Running leg_kinemtics script...')
    run('../leg_kinematics/leg_kinematics.m')
    close all; clear; clc
    disp('-- leg_kinematics script finished.')
end

leg_geometry_stroke = csvread('../data/leg_geometry_stroke.csv', 1, 0);

ind = 1;

pivots = zeros(11, 2);

pivots(1, :) = [leg_geometry_stroke(ind, 1), leg_geometry_stroke(ind, 2)]; % A
pivots(2, :) = [leg_geometry_stroke(ind, 3), leg_geometry_stroke(ind, 4)]; % B
pivots(3, :) = [leg_geometry_stroke(ind, 5), leg_geometry_stroke(ind, 6)]; % C
pivots(4, :) = [leg_geometry_stroke(ind, 7), leg_geometry_stroke(ind, 8)]; % D
pivots(5, :) = [leg_geometry_stroke(ind, 9), leg_geometry_stroke(ind, 10)]; % F
pivots(6, :) = [leg_geometry_stroke(ind, 11), leg_geometry_stroke(ind, 12)]; % G
pivots(7, :) = [leg_geometry_stroke(ind, 13), leg_geometry_stroke(ind, 14)]; % H
pivots(8, :) = [leg_geometry_stroke(ind, 15), leg_geometry_stroke(ind, 16)]; % K
pivots(9, :) = [leg_geometry_stroke(ind, 17), leg_geometry_stroke(ind, 18)]; % L
pivots(10, :) = [leg_geometry_stroke(ind, 19), leg_geometry_stroke(ind, 20)]; % M
pivots(11, :) = [leg_geometry_stroke(ind, 21), leg_geometry_stroke(ind, 22)]; % P

figure('Name', 'Leg Visualization')
grid on;



visualize_configuration(pivots);


