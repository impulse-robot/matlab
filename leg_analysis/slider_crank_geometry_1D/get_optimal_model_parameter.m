function [mp_opt] = get_optimal_model_parameter(jump_heights_valid, crank_lengths_square, pushrod_lengths_square, length_body, length_foot, radius_foot, mass_body_empty, safety_factor)
%GET_OPTIMAL_MODEL_PARAMETER - get optimal model parameter based on
%calculated jumping heights for different model parameters

    max_jump_height = max(jump_heights_valid(:));
    [j_max, i_max, b_max, m_max] = ind2sub(size(jump_heights_valid), find(jump_heights_valid(:) == max_jump_height));
    crank_length_opt = crank_lengths_square(j_max, i_max);
    pushrod_length_opt = pushrod_lengths_square(j_max, i_max);
    motor_type_opt = m_max - 1;
    battery_type_opt = b_max - 1;
    
    % Store results
    mp_opt = get_model_parameters(crank_length_opt, pushrod_length_opt, length_body, ...
        length_foot, radius_foot, motor_type_opt, ...
        battery_type_opt, mass_body_empty, safety_factor, max_jump_height);
end

