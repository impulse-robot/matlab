function [] = visualize_jump_heights(crank_lengths_square, pushrod_lengths_square, jump_heights_square, max_leg_extension, mp_opt)
%VISUALIZE_JUMP_HEIGHTS - visualize jump height for different model
%parameters
%   Detailed explanation goes here

max_jump_height = mp_opt.max_jump_height;
n_cell = mp_opt.battery_n_cell;
m_reduction = mp_opt.motor_gear_reduction;
crank_length_opt = mp_opt.length_crank;
pushrod_length_opt = mp_opt.length_pushrod;

figure('name', 'Leg Geometry optimization')
subplot(3, 2, 1);
surf(crank_lengths_square(:, :), pushrod_lengths_square(:, :), jump_heights_square(:, :, 1, 1));
title('Motor: A80-9 / Battery: 4s-Lipo');
zlabel('Jump Height [m]');
xlabel('Crank Length [m]');
ylabel('Pushrod Length [m]');
axis vis3d
hold on

if(n_cell == 4 && m_reduction == 9)
    scatter3(crank_length_opt, pushrod_length_opt, max_jump_height, 60, 'filled', 'r');
    
    hold on
    max_jump_height_annotation_line_1 = ['Max Jump Height: ' num2str(max_jump_height) 'm'];
    max_jump_height_annotation_line_2 = ['Crank length: ' num2str(crank_length_opt) 'm'];
    max_jump_height_annotation_line_3 = ['Pushrod length: ' num2str(pushrod_length_opt) 'm'];
    
    text(crank_length_opt, pushrod_length_opt, max_jump_height + 0.4, {max_jump_height_annotation_line_1; max_jump_height_annotation_line_2; max_jump_height_annotation_line_3}, 'Fontsize', 15);
    hold on
end

limit_x_square = [0.0 0.0; max_leg_extension max_leg_extension];
limit_y_square = [max_leg_extension max_leg_extension; 0.0, 0.0];
limit_z_square = [0.0 2; 0.0 2];

surf(limit_x_square, limit_y_square, limit_z_square, 'FaceColor', [0.9100    0.4100    0.1700], 'FaceAlpha', 0.5);

colorbar

subplot(3, 2, 2);
surf(crank_lengths_square(:, :), pushrod_lengths_square(:, :), jump_heights_square(:, :, 1, 2));
title('Motor: A80-6 / Battery: 4s-Lipo');
zlabel('Jump Height [m]');
xlabel('Crank Length [m]');
ylabel('Pushrod Length [m]');
axis vis3d
hold on

if(n_cell == 4 && m_reduction == 6)
    scatter3(crank_length_opt, pushrod_length_opt, max_jump_height, 60, 'filled', 'r');
    hold on
    max_jump_height_annotation_line_1 = ['Max Jump Height: ' num2str(max_jump_height) 'm'];
    max_jump_height_annotation_line_2 = ['Crank length: ' num2str(crank_length_opt) 'm'];
    max_jump_height_annotation_line_3 = ['Pushrod length: ' num2str(pushrod_length_opt) 'm'];
    
    text(crank_length_opt, pushrod_length_opt, max_jump_height + 0.4, {max_jump_height_annotation_line_1; max_jump_height_annotation_line_2; max_jump_height_annotation_line_3}, 'Fontsize', 15);
    hold on
end

limit_x_square = [0.0 0.0; max_leg_extension max_leg_extension];
limit_y_square = [max_leg_extension max_leg_extension; 0.0, 0.0];
limit_z_square = [0.0 2; 0.0 2];

surf(limit_x_square, limit_y_square, limit_z_square, 'FaceColor', [0.9100    0.4100    0.1700], 'FaceAlpha', 0.5);

colorbar

subplot(3, 2, 3);
surf(crank_lengths_square(:, :), pushrod_lengths_square(:, :), jump_heights_square(:, :, 2, 1));
title('Motor: A80-9 / Battery: 6s-Lipo');
zlabel('Jump Height [m]');
xlabel('Crank Length [m]');
ylabel('Pushrod Length [m]');
axis vis3d
hold on

if(n_cell == 6 && m_reduction == 9)
    scatter3(crank_length_opt, pushrod_length_opt, max_jump_height, 60, 'filled', 'r');
    hold on
    max_jump_height_annotation_line_1 = ['Max Jump Height: ' num2str(max_jump_height) 'm'];
    max_jump_height_annotation_line_2 = ['Crank length: ' num2str(crank_length_opt) 'm'];
    max_jump_height_annotation_line_3 = ['Pushrod length: ' num2str(pushrod_length_opt) 'm'];
    
    text(crank_length_opt, pushrod_length_opt, max_jump_height + 0.4, {max_jump_height_annotation_line_1; max_jump_height_annotation_line_2; max_jump_height_annotation_line_3}, 'Fontsize', 15);
    hold on
end

limit_x_square = [0.0 0.0; max_leg_extension max_leg_extension];
limit_y_square = [max_leg_extension max_leg_extension; 0.0, 0.0];
limit_z_square = [0.0 2; 0.0 2];

surf(limit_x_square, limit_y_square, limit_z_square, 'FaceColor', [0.9100    0.4100    0.1700], 'FaceAlpha', 0.5);

colorbar

subplot(3, 2, 4);
surf(crank_lengths_square(:, :), pushrod_lengths_square(:, :), jump_heights_square(:, :, 2, 2));
title('Motor: A80-6 / Battery: 6s-Lipo');
zlabel('Jump Height [m]');
xlabel('Crank Length [m]');
ylabel('Pushrod Length [m]');
axis vis3d
hold on

if(n_cell == 6 && m_reduction == 6)
    scatter3(crank_length_opt, pushrod_length_opt, max_jump_height, 60, 'filled', 'r');
    hold on
    max_jump_height_annotation_line_1 = ['Max Jump Height: ' num2str(max_jump_height) 'm'];
    max_jump_height_annotation_line_2 = ['Crank length: ' num2str(crank_length_opt) 'm'];
    max_jump_height_annotation_line_3 = ['Pushrod length: ' num2str(pushrod_length_opt) 'm'];
    
    text(crank_length_opt, pushrod_length_opt, max_jump_height + 0.4, {max_jump_height_annotation_line_1; max_jump_height_annotation_line_2; max_jump_height_annotation_line_3}, 'Fontsize', 15);
    hold on
end

limit_x_square = [0.0 0.0; max_leg_extension max_leg_extension];
limit_y_square = [max_leg_extension max_leg_extension; 0.0, 0.0];
limit_z_square = [0.0 2; 0.0 2];

surf(limit_x_square, limit_y_square, limit_z_square, 'FaceColor', [0.9100    0.4100    0.1700], 'FaceAlpha', 0.5);

colorbar

subplot(3, 2, 5);
surf(crank_lengths_square(:, :), pushrod_lengths_square(:, :), jump_heights_square(:, :, 3, 1));
title('Motor: A80-9 / Battery: 8s-Lipo');
zlabel('Jump Height [m]');
xlabel('Crank Length [m]');
ylabel('Pushrod Length [m]');
axis vis3d
hold on

if(n_cell == 8 && m_reduction == 9)
    scatter3(crank_length_opt, pushrod_length_opt, max_jump_height, 60, 'filled', 'r');
    hold on
    max_jump_height_annotation_line_1 = ['Max Jump Height: ' num2str(max_jump_height) 'm'];
    max_jump_height_annotation_line_2 = ['Crank length: ' num2str(crank_length_opt) 'm'];
    max_jump_height_annotation_line_3 = ['Pushrod length: ' num2str(pushrod_length_opt) 'm'];
    
    text(crank_length_opt, pushrod_length_opt, max_jump_height + 0.4, {max_jump_height_annotation_line_1; max_jump_height_annotation_line_2; max_jump_height_annotation_line_3}, 'Fontsize', 15);
    hold on
end

limit_x_square = [0.0 0.0; max_leg_extension max_leg_extension];
limit_y_square = [max_leg_extension max_leg_extension; 0.0, 0.0];
limit_z_square = [0.0 2; 0.0 2];

surf(limit_x_square, limit_y_square, limit_z_square, 'FaceColor', [0.9100    0.4100    0.1700], 'FaceAlpha', 0.5);

colorbar

subplot(3, 2, 6);
surf(crank_lengths_square(:, :), pushrod_lengths_square(:, :), jump_heights_square(:, :, 3, 2));
title('Motor: A80-6 / Battery: 8s-Lipo');
zlabel('Jump Height [m]');
xlabel('Crank Length [m]');
ylabel('Pushrod Length [m]');
axis vis3d
hold on

if(n_cell == 8 && m_reduction == 6)
    scatter3(crank_length_opt, pushrod_length_opt, max_jump_height, 60, 'filled', 'r');
    hold on
    max_jump_height_annotation_line_1 = ['Max Jump Height: ' num2str(max_jump_height) 'm'];
    max_jump_height_annotation_line_2 = ['Crank length: ' num2str(crank_length_opt) 'm'];
    max_jump_height_annotation_line_3 = ['Pushrod length: ' num2str(pushrod_length_opt) 'm'];
    
    text(crank_length_opt, pushrod_length_opt, max_jump_height + 0.4, {max_jump_height_annotation_line_1; max_jump_height_annotation_line_2; max_jump_height_annotation_line_3}, 'Fontsize', 15);
    hold on
end

limit_x_square = [0.0 0.0; max_leg_extension max_leg_extension];
limit_y_square = [max_leg_extension max_leg_extension; 0.0, 0.0];
limit_z_square = [0.0 2; 0.0 2];

surf(limit_x_square, limit_y_square, limit_z_square, 'FaceColor', [0.9100    0.4100    0.1700], 'FaceAlpha', 0.5);


colorbar


% Visualize optimal battery and motor jump heights

if (n_cell == 4)
    b_opt = 1;
elseif (n_cell == 6)
    b_opt = 2;
elseif (n_cell == 8)
    b_opt = 3;
end

if (m_reduction == 9)
    m_opt = 1;
elseif (m_reduction == 6)
    m_opt = 2;
end

plot_title = strcat('Motor: A80-', num2str(m_reduction), ' / Battery: ', num2str(n_cell), 's-Lipo');

figure('name', 'Jump Height optimal Battery / Motor')
surf(crank_lengths_square(:, :), pushrod_lengths_square(:, :), jump_heights_square(:, :, b_opt, m_opt));
title(plot_title);
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

end


