function [] = visualize_joint_forces(theta, joint_forces_crank_pushrod, ...
        joint_forces_crank_body, ...
        joint_forces_body_crank, joint_forces_body_linear_guide, ...
        joint_forces_pushrod_linear_guide, joint_forces_pushrod_crank, ...
        joint_forces_linear_guide_body, joint_forces_linear_guide_lower_joint, ...
        worst_case)
%VISUALIZE_JOINT_FORCES - visualize joint forces in inertial frame

if (worst_case)
    y_upper_limit = 600;
    y_lower_limit = -600;
else
    y_upper_limit = 300;
    y_lower_limit = -300;
end


figure('name', 'Joint Forces')
subplot(2, 2, 1);


% Forces on body
subplot(2, 4, 1);
plot(theta, joint_forces_body_crank, 'LineWidth', 3);
hold on;
title('Joint Forces Body at Motor joint');
ylabel('F [N]');
xlabel('theta [rad]');
ylim([y_lower_limit, y_upper_limit]);
grid on;
legend('x', 'y');

subplot(2, 4, 5);
plot(theta, joint_forces_body_linear_guide, 'LineWidth', 3);
hold on;
title('Joint Forces Body at linear guide joint');
ylabel('F [N]');
xlabel('theta [rad]');
ylim([y_lower_limit, y_upper_limit]);
grid on;
legend('x', 'y');

% Forces on crank
subplot(2, 4, 2);
plot(theta, joint_forces_crank_body, 'LineWidth', 3);
hold on;
title('Joint Forces Crank upper joint');
ylabel('F [N]');
xlabel('theta [rad]');
ylim([y_lower_limit, y_upper_limit]);
grid on;
legend('x', 'y');

subplot(2, 4, 6);
plot(theta, joint_forces_crank_pushrod, 'LineWidth', 3);
hold on;
title('Joint Forces Crank at lower joint');
ylabel('F [N]');
xlabel('theta [rad]');
ylim([y_lower_limit, y_upper_limit]);
grid on;
legend('x', 'y');

% Forces on pushrod
subplot(2, 4, 3);
plot(theta, joint_forces_pushrod_crank, 'LineWidth', 3);
hold on;
title('Joint Forces Pushrod at upper joint');
ylabel('F [N]');
xlabel('theta [rad]');
ylim([y_lower_limit, y_upper_limit]);
grid on;
legend('x', 'y');

subplot(2, 4, 7);
plot(theta, joint_forces_pushrod_linear_guide, 'LineWidth', 3);
hold on;
title('Joint Forces Pushrod at lower joint');
ylabel('F [N]');
xlabel('theta [rad]');
ylim([y_lower_limit, y_upper_limit]);
grid on;
legend('x', 'y');

% Forces on linear guide
subplot(2, 4, 4);
plot(theta, joint_forces_linear_guide_body, 'LineWidth', 3);
hold on;
title('Joint Forces Linear Guide at linear joint');
ylabel('F [N]');
xlabel('theta [rad]');
ylim([y_lower_limit, y_upper_limit]);
grid on;
legend('x', 'y');

subplot(2, 4, 8);
plot(theta, joint_forces_linear_guide_lower_joint, 'LineWidth', 3);
hold on;
title('Joint Forces Linear guide at lower joint');
ylabel('F [N]');
xlabel('theta [rad]');
ylim([y_lower_limit, y_upper_limit]);
grid on;
legend('x', 'y');



end

