%% Leg optimization
% Author:   Alex Dietsche
% Date: 1/09/19

close all; clear; clc;


%% Optimization parameters
N = 5;                      % square root of crank-pushrod combinations []
mass = 1.5;                 % robot mass without the motor and battery [kg]
length_crank = 0.25;        % [m]
length_pushrod = 0.4;       % [m]
length_offset = 0.005;      % [m]
number_cells = 6;           % number of battery cells []
motor_type = 1;             % [0 - A80-9, 1 - A80-6]

% generate crank - pushrod combinations for optimization
length_crank_min = 0.03;    % [m]
length_crank_max = 0.4;     % [m]

length_pushrod_min = 0.03;  % [m]
length_pushrod_max = 0.5;   % [m]

length_combinations = zeros(N^2, 2);
crank_lengths_lin = linspace(length_crank_min, length_crank_max, N)';
crank_lengths_square = repmat(crank_lengths_lin, 1, N)';
crank_lengths = crank_lengths_square(:)';

length_combinations(:,1) = crank_lengths;

pushrod_lengths_square = zeros(N, N);

i = 1;
for crank_length = crank_lengths_lin'
    pushrod_lengths = linspace(crank_length + length_offset, length_pushrod_max, N);
    pushrod_lengths_square(:, i) = pushrod_lengths';
    start_index = 1 + (i-1) * N;
    end_index = i * N;
    length_combinations(start_index:end_index , 2) = pushrod_lengths;
    i = i + 1;
end

%% Optimization
jump_heights_square = zeros(N, N);

for i = 1:N
    for j = 1:N
        crank_length = crank_lengths_square(j, i);
        pushrod_length = pushrod_lengths_square(j, i);
        jump_heights_square(i, j) = get_jump_height(mass, crank_length, pushrod_length, number_cells, motor_type, false);
    end
    progress = i / N;
    disp(['-- [leg_optimization] INFO: ', num2str(progress * 100), '% finished']);
end


%% Visualization
surf(crank_lengths_square, pushrod_lengths_square, jump_heights_square);
zlabel('Jump Height [m]');
xlabel('Crank Length [m]');
ylabel('Pushrod Length [m]');
axis vis3d
colorbar