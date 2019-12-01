function [] = visualize_motion(t,Q)
%VISUALIZE_MOTION Visualize motion of robot
%   Visualization of the robot 
%
%   Inputs:
%       t - k x 1 vector containing time
%
%       Q - K x 4 [phi, phi_dot, theta, theta_dot] vector containing all
%       state variables for different time steps
%

global L_F L_R L_C M_F M_R M_C M_B I_F I_R I_C I_B

for i = 1 : size(t, 1)
    t_current = t(i);
    q_current = Q(i);
    
    
    
    
    
end

end

