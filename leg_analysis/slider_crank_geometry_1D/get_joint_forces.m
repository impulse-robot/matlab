function [joint_forces_crank_pushrod,joint_forces_body_crank, ...
    joint_forces_body_linear_guide, joint_forces_pushrod_linear_guide, ...
    joint_forces_body_external] = ...
    get_joint_forces(t, torque, theta, mp_opt)
%GET_JOINT_FORCES - calculate joint forces
%   Detailed explanation goes here

syms M f_bx f_ey f_cx f_cy f_fx f_fy f_wx f_gx f_gy real
syms th al l_g real
syms l_c l_r

% f_ey is the constraining force for the static analysis of the robot
% in reality this force will accelerate the body of the robot

l_g = l_r * sqrt(1 - (l_c/l_r * cos(th))^2) + l_c/2 * sin(th);
al = asin(l_c/l_r * cos(th));

eqns = [f_bx == -f_cx, f_ey == -f_cy, ...
    M == f_cy * l_c * cos(th) - f_cx * l_c * sin(th), ...
    f_fx == f_cx, f_fy == f_cy, ...
    f_cx * l_r * cos(al) == - f_cy * l_r * sin(al), ...
    f_wx == f_bx + f_fx - f_gx, f_fy - f_gy == 0, ...
    -M + l_g * f_bx - l_g * f_wx == 0];

eqns = subs(eqns, [l_c, l_r], [mp_opt.length_crank, mp_opt.length_pushrod]);
s = solve(eqns, [f_bx f_ey f_cx f_cy f_fx f_fy f_wx f_gx f_gy], 'ReturnConditions', true);

% TODO
% check equation, joint_force_body_external should be 0, but it is not

F_bx = matlabFunction(s.f_bx);
F_ey = matlabFunction(s.f_ey);
F_cx = matlabFunction(s.f_cx);
F_cy = matlabFunction(s.f_cy);
F_fx = matlabFunction(s.f_fx);
F_fy = matlabFunction(s.f_fy);
F_wx = matlabFunction(s.f_wx);

joint_forces_crank_pushrod = zeros(2, size(theta, 2));          % force exerted on crank by pushrod (in intertial frame)
joint_forces_body_crank = zeros(2, size(theta, 2));             % force exerted on body by crank (in intertial frame)
joint_forces_body_linear_guide = zeros(2, size(theta, 2));      % force exerted on body by linear guide (in intertial frame)
joint_forces_pushrod_linear_guide = zeros(2, size(theta, 2));   % force exerted on pushrod by linear guide (in intertial frame)
joint_forces_body_external = zeros(1, size(theta, 2));

for i = 1:size(t, 2)
    condition_with_values = subs(s.conditions, [M, th], [torque(i), theta(i)]);
    if(isAlways(condition_with_values))
%         joint_forces_crank_pushrod(1, i) = eval(subs(s.f_cx, [M, th], [torque(i), theta(i)]));
%         joint_forces_crank_pushrod(2, i) = eval(subs(s.f_cy, [M, th], [torque(i), theta(i)]));
%         joint_forces_body_crank(1, i) = -eval(subs(s.f_bx, [M, th], [torque(i), theta(i)]));
%         joint_forces_body_crank(2, i) = -eval(subs(s.f_ey, [M, th], [torque(i), theta(i)]));
%         joint_forces_body_linear_guide(1, i) = -eval(subs(s.f_bx, [M, th], [torque(i), theta(i)]));
%         joint_forces_body_linear_guide(2, i) = 0;
%         joint_forces_pushrod_linear_guide(1, i) = eval(subs(s.f_fx, [M, th], [torque(i), theta(i)]));
%         joint_forces_pushrod_linear_guide(2, i) = eval(subs(s.f_fy, [M, th], [torque(i), theta(i)]));

        joint_forces_crank_pushrod(1, i) = F_cx(torque(i), theta(i));
        joint_forces_crank_pushrod(2, i) = F_cy(torque(i), theta(i));
        joint_forces_body_crank(1, i) = -F_bx(torque(i), theta(i));
        joint_forces_body_crank(2, i) = -F_ey(torque(i), theta(i));
        joint_forces_body_linear_guide(1, i) = -F_bx(torque(i), theta(i));
        joint_forces_body_linear_guide(2, i) = 0;
        joint_forces_pushrod_linear_guide(1, i) = F_fx(torque(i), theta(i));
        joint_forces_pushrod_linear_guide(2, i) = F_fy(torque(i), theta(i));
        joint_forces_body_external(i) = F_wx(torque(i), theta(i));
        
    else
        disp('WARNING condition for solution of joint forces not fulfilled!');
        disp(['Torque: ', num2str(torque(i)), ', Theta: ' , num2str(theta(i))]);
    end
end

end

