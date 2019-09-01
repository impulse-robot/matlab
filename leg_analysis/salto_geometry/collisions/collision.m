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

pivots_stroke = csvread('../data/leg_geometry_stroke.csv', 1, 0);
pivots_cad = csvread('../data/leg_geometry.csv', 1, 0);

leg_slider();
