%% Leg optimization
% Author:   Alex Dietsche
% Date: 1/09/19

close all; clear; clc;


%% Optimization parameters
N = 60;                      % square root of crank-pushrod combinations []
mass = 1.0;                 % robot mass without the motor and battery [kg]
length_crank = 0.168;        % [m]
length_pushrod = 0.176;       % [m]
number_cells = 6;           % number of battery cells []
motor_type = 0;             % [0 - A80-9, 1 - A80-6]

max_leg_extension = 0.35;   % [m]

test_jump_height = get_jump_height(mass, length_crank, length_pushrod, number_cells, motor_type, true);


%% generate crank - pushrod combinations for optimization
length_crank_min = 0.02;    % [m]
length_crank_max = 0.25;     % [m]

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

%% Optimization
jump_heights_square = zeros(N, N);
jump_heights_valid = zeros(N, N);

for i = 1:N
    parfor (j = 1:N, 10)
        crank_length = crank_lengths_square(j, i);
        pushrod_length = pushrod_lengths_square(j, i);
        if (pushrod_length < crank_length)
            jump_heights_square(i, j) = 0;
        else
            jump_heights_square(i, j) = get_jump_height(mass, crank_length, pushrod_length, number_cells, motor_type, false);
        end
    
        if (crank_length + pushrod_length < max_leg_extension)
            jump_heights_valid(i, j) = jump_heights_square(i, j);
        else
            jump_heights_valid(i, j) = 0;
        end
    end
    progress = i / N;
    disp(['-- [leg_optimization] INFO: ', num2str(progress * 100), '% finished']);
end


%% Valid leg geometries
[j_max, i_max] = find(jump_heights_valid == max(jump_heights_valid, [], 'all'));
max_jump_height = jump_heights_valid(j_max, i_max);
crank_length_opt = crank_lengths_square(j_max, i_max);
pushrod_length_opt = pushrod_lengths_square(j_max, i_max);

%% Visualization
surf(crank_lengths_square, pushrod_lengths_square, jump_heights_square);
zlabel('Jump Height [m]');
xlabel('Crank Length [m]');
ylabel('Pushrod Length [m]');
axis vis3d
hold on

scatter3(crank_length_opt, pushrod_length_opt, max_jump_height, 60, 'filled', 'r');
hold on
max_jump_height_annotation_line_1 = ['Max Jump Height: ' num2str(max_jump_height) 'm'];
max_jump_height_annotation_line_2 = ['Crank length: ' num2str(crank_length_opt) 'm'];
max_jump_height_annotation_line_3 = ['Pushrod length: ' num2str(pushrod_length_opt) 'm'];

text(crank_length_opt, pushrod_length_opt, max_jump_height + 0.4, {max_jump_height_annotation_line_1; max_jump_height_annotation_line_2; max_jump_height_annotation_line_3}, 'Fontsize', 15);
hold on


limit_x_square = [0.0 0.0; max_leg_extension max_leg_extension];
limit_y_square = [max_leg_extension max_leg_extension; 0.0, 0.0];
limit_z_square = [0.0 2; 0.0 2];

surf(limit_x_square, limit_y_square, limit_z_square, 'FaceColor', [0.9100    0.4100    0.1700], 'FaceAlpha', 0.5);


colorbar