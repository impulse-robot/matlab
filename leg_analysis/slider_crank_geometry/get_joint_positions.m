function [joint_positions] = get_joint_positions(q)
%GET_JOINT_POSITIONS Get joint positions of robot for visualization
%   Calculate joint positions based on generalized coordinates q
%
%   Inputs:
%       q  - generalized coordinates (4 x 1 matrix) [phi; phi_dot; theta; theta_dot]
%
%
%   Ouput:
%       joint_positions - joint positions [FT; RF; CR; MC] where
%                         where FT : Foot tip , RF: Rod-Foot joint, 
%                         CR: crank-rod joint, MC: motor center
%

global L_F L_R L_C 

FT_sym = [0; 0];

RF_sym = [-L_F * sin(phi); 
    L_F * cos(phi)];

CR_sym = [-L_C * cos(theta) * cos(phi) - (L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2)) * sin(phi);
        -L_C * cos(theta) * sin(phi) + (L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2)) * cos(phi)];
    
CR_test = [-L_C * cos(theta) * cos(phi) - (L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2) + L_C * sin(theta)) * sin(phi);
        -L_C * cos(theta) * sin(phi) + (L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2) + L_C * sin(theta)) * cos(phi)];

MC_sym = [-(L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2) + L_C * sin(theta)) * sin(phi);
        (L_F + L_R * sqrt(1 - (L_C/L_R * cos(theta))^2) + L_C * sin(theta)) * cos(phi)];
  
FT = double(FT_sym);
RF = double(subs(RF_sym, [phi, theta] , [q(1), q(3)]));
CR = double(subs(CR_sym, [phi, theta] , [q(1), q(3)]));
MC = double(subs(MC_sym, [phi, theta] , [q(1), q(3)]));




end

