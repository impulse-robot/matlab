%% Jump height
% Author:   Alex Dietsche
% Date: 1/09/19

close all; clear; clc

N = 1000;

M = 18;
n = linspace(0, 300, N);
Ucell = 3.7;
ncell = 6;
M_motor = [];

for i = 1:N
    M_motor = [M_motor, motor_torque(M, n(i), Ucell, ncell)];
end

rpm = (n * 60 / (2 * pi)) / 9;
M_motor = 9 * M_motor;

plot(rpm , M_motor);
grid on;


le = 0.25;
lc = 0.3;
lr = 1;

theta_0 = -pi/2 + asin(le/lr);

thetas = linspace(theta_0, theta_0 + pi, N);
jacobians = [];

for i = 1:N
    jacobians = [jacobians, get_jacobian(thetas(i), lc, lr, le)];
end

% plot(thetas, jacobians);
% grid on;