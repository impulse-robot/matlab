function [] = visualize_configuration(pivots)
%VISUALIZE_CONFIGURATION - visualize the current leg configuration
% Syntax: visualize_configuration(pivots)
%
% Inputs:
%    pivot points - pivots points stored in a 11x2 array
%
% Outputs:
%    None
%
% Example: 
%    visualize_configuration(pivots)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

A = pivots(1, :);
B = pivots(2, :);
C = pivots(3, :);
D = pivots(4, :);
F = pivots(5, :);
G = pivots(6, :);
H = pivots(7, :);
K = pivots(8, :);
L = pivots(9, :);
M = pivots(10, :);
P_0 = pivots(11, :);

% input_angle = atan2(F(2), F(1));
% end_effector = P_0;

crank = [[B(1); D(1); F(1)], [B(2); D(2); F(2)]];
shin = [[H(1); G(1); L(1); C(1)], [H(2); G(2); L(2); C(2)]];
foot = [[M(1); L(1); P_0(1)], [M(2); L(2); P_0(2)]];
rod = [[A(1); K(1); C(1)], [A(2); K(2); C(2)]];
crank_link_upper = [[F(1); H(1)], [F(2); H(2)]];
crank_link_lower = [[D(1); G(1)], [D(2); G(2)]];
achilles_link = [[K(1); M(1)], [K(2); M(2)]];

% plot terniary links
cla;
crank_area = fill(crank(:, 1), crank(:, 2), 'b', 'LineWidth', 3);
set(crank_area,'facealpha',.3)
hold on
shin_area = fill(shin(:, 1), shin(:, 2), 'b', 'LineWidth', 3);
set(shin_area,'facealpha',.3)
hold on
foot_area = fill(foot(:, 1), foot(:, 2), 'b', 'LineWidth', 3);
set(foot_area,'facealpha',.3)
hold on
rod_area = fill(rod(:, 1), rod(:, 2), 'b', 'LineWidth', 3);
set(rod_area,'facealpha',.3)
hold on

% plot links
plot(crank_link_upper(:, 1), crank_link_upper(:, 2), 'b', 'LineWidth', 4);
hold on
plot(crank_link_lower(:, 1), crank_link_lower(:, 2), 'b', 'LineWidth', 4);
hold on
plot(achilles_link(:, 1), achilles_link(:, 2), 'b', 'LineWidth', 4);
hold on

% plot pivots
scatter(pivots(:, 1), pivots(:, 2), 75, 'k', 'LineWidth', 4);
hold on


title('Leg Kinematics')
xlabel('x[m]')
ylabel('y[m]')
axis equal
xlim([-0.17, 0.17])
grid on

end

