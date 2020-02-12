function [joint_forces_crank_pushrod, joint_forces_crank_body, ...
        joint_forces_body_crank, joint_forces_body_linear_guide, ...
        joint_forces_pushrod_linear_guide, joint_forces_pushrod_crank, ...
        joint_forces_linear_guide_body, joint_forces_linear_guide_pushrod, ...
        joint_forces_linear_guide_ground] = ...
        get_joint_forces(t, torque, theta, mp_opt, worst_case)
%GET_JOINT_FORCES - calculate joint forces
%   Detailed explanation goes here

syms M f_bx f_ey f_cx f_cy f_fx f_fy f_wx f_gx f_gy real
syms th al l_g real
syms l_c l_r

% f_ey is the constraining force for the static analysis of the robot
% in reality this force will accelerate the body of the robot

l_g = l_r * sqrt(1 - (l_c/l_r * cos(th))^2) + l_c * sin(th);
al = asin(l_c/l_r * cos(th));

eqns = [f_bx == -f_cx, f_ey == -f_cy, ...
    M == f_cy * l_c * cos(th) - f_cx * l_c * sin(th), ...
    f_fx == f_cx, f_fy == f_cy, ...
    f_cx * l_r * cos(al) == - f_cy * l_r * sin(al), ...
    f_wx == f_bx + f_fx - f_gx, f_fy == f_gy, ...
    -M + l_g * f_bx - l_g * f_wx == 0];

eqns = subs(eqns, [l_c, l_r], [mp_opt.length_crank, mp_opt.length_pushrod]);
s = solve(eqns, [f_bx f_ey f_cx f_cy f_fx f_fy f_wx f_gx f_gy], 'ReturnConditions', true);

F_bx = matlabFunction(s.f_bx);
F_ey = matlabFunction(s.f_ey);
F_cx = matlabFunction(s.f_cx);
F_cy = matlabFunction(s.f_cy);
F_fx = matlabFunction(s.f_fx);
F_fy = matlabFunction(s.f_fy);
F_gy = matlabFunction(s.f_gy);

joint_forces_crank_pushrod = zeros(2, size(theta, 2));          % force exerted on crank by pushrod (in intertial frame)
joint_forces_crank_body = zeros(2, size(theta, 2));             % force exerted on crank by body (in intertial frame)
joint_forces_body_crank = zeros(2, size(theta, 2));             % force exerted on body by crank (in intertial frame)
joint_forces_body_linear_guide = zeros(2, size(theta, 2));      % force exerted on body by linear guide (in intertial frame)
joint_forces_pushrod_linear_guide = zeros(2, size(theta, 2));   % force exerted on pushrod by linear guide (in intertial frame)
joint_forces_pushrod_crank = zeros(2, size(theta, 2));          % force exerted on pushrod by linear guide (in intertial frame)
joint_forces_linear_guide_body = zeros(2, size(theta, 2));      % force exerted on linear guide by body (in intertial frame)
joint_forces_linear_guide_pushrod = zeros(2, size(theta, 2));   % force exerted on linear guide by pushrod (in intertial frame)
joint_forces_linear_guide_ground = zeros(2, size(theta, 2));    % force exerted on linear guide by ground (in intertial frame)

for i = 1:size(t, 2)
    
    if (worst_case)
        torque_current = mp_opt.motor_max_torque;
    else
        torque_current = torque(i);
    end
    
    condition_with_values = subs(s.conditions, [M, th], [torque_current, theta(i)]);
    if(isAlways(condition_with_values))
        
        joint_forces_crank_pushrod(1, i) = F_cx(torque_current, theta(i));
        joint_forces_crank_pushrod(2, i) = F_cy(torque_current, theta(i));
        joint_forces_body_crank(1, i) = -F_bx(torque_current, theta(i));
        joint_forces_body_crank(2, i) = -F_ey(torque_current, theta(i));
        joint_forces_body_linear_guide(1, i) = -F_bx(torque_current, theta(i));
        joint_forces_body_linear_guide(2, i) = 0;
        joint_forces_pushrod_linear_guide(1, i) = F_fx(torque_current, theta(i));
        joint_forces_pushrod_linear_guide(2, i) = F_fy(torque_current, theta(i));
        joint_forces_linear_guide_ground(1, i) = 0;
        joint_forces_linear_guide_ground(2, i) = F_gy(torque_current, theta(i));
        
    else
        disp('WARNING condition for solution of joint forces not fulfilled!');
        disp(['Torque: ', num2str(torque_current), ', Theta: ' , num2str(theta(i))]);
    end
    
    
    
    
end

joint_forces_crank_body = -joint_forces_body_crank;
joint_forces_pushrod_crank = -joint_forces_crank_pushrod;
joint_forces_linear_guide_body = -joint_forces_body_linear_guide;
joint_forces_linear_guide_pushrod = -joint_forces_pushrod_linear_guide;


end

