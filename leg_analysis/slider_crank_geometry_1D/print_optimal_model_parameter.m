function [] = print_optimal_model_parameter(mp_opt)
%PRINT_OPTIMAL_MODEL_PARAMETER - print optimal model parameter to console

if (mp_opt.motor_gear_reduction == 9)
    motor_opt = 'A80-9';
elseif (mp_opt.motor_gear_reduction == 6)
    motor_opt = 'A80-6';
end

if (mp_opt.battery_n_cell == 4)
    battery_opt = '4s';
elseif (mp_opt.battery_n_cell == 6)
    battery_opt = '6s';
elseif (mp_opt.battery_n_cell == 8)
    battery_opt = '8s';
end

disp('################### RESULTS #####################');
fprintf('\n');
disp(['== OPTIMAL CRANK LENGTH: ', num2str(mp_opt.length_crank), ' [m]']);
disp(['== OPTIMAL PUSHROD LENGTH: ', num2str(mp_opt.length_pushrod), ' [m]']);
disp(['== OPTIMAL MOTOR TYPE: ', motor_opt]);
disp(['== OPTIMAL BATTERY TYPE: ', battery_opt, '-LiPo']);
disp(['== THEORETICAL JUMP HEIGHT: ', num2str(mp_opt.max_jump_height), ' [m]']);
fprintf('\n');
disp('##################### END #######################');

end

