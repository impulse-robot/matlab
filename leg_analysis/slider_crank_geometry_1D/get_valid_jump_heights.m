function [jump_heights_valid] = get_valid_jump_heights(jump_heights, crank_lengths_square, pushrod_lengths_square, max_leg_extension)
%GET_VALID_JUMP_HEIGHTS - return jump heights for leg configurations that
%fulfill the max_leg_extension criterion

jump_heights_valid = jump_heights;

N = size(jump_heights, 1);

for j = 1:N
    for i = 1:N
        if (crank_lengths_square(j, i) + pushrod_lengths_square(j, i) > max_leg_extension)
            jump_heights_valid(j, i, :, :) = 0;
        end
    end
end

end

