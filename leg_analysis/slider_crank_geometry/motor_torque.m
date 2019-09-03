function [t_actual] = joint_torque(t_desired, angular_velocity, u_cell, n_cell, R, kv, kt, max_torque)
% Inputs: 
% t_desired - desired torque                                    [Nm]
% angular_velocity - Angular velocity of output shaft           [rad/s]
% u_cell - voltage of single cell                               [V]
% n_cell - number of cells in battery                           []
% R - inner resistance of motor                                 [Ohm]
% kv - motor velocity constant                                  [RPM/V]
% kt - motor torque constant                                    [Nm/A]
% max_torque - maximum torque of motor at output shaft          [Nm]


U = Ucell*ncell*0.96;

% T Motors A80-9
R = 0.175;                      % [Ohm]
kv_ = 105;                      % [RPM/V]
kv = kv_ * 2 * pi / 60;         % [rad/s / V]
ke = 1 / kv;                    % [V / rad/s]
kt = 0.091;                     % [Nm/A]

        
% Maximales Drehmoment
if t_desired>max_torque
    t_desired = max_torque;
end

Ui = ke*n;

if Ui >= U
    t_actual = 0;
else
    Ud = U-Ui;
    I = t_desired/kt;
    Ur = R*I;
    
    if Ud > Ur
        t_actual = t_desired;
    else
        t_actual = kt*(Ud/R);
    end
end


%fprintf('M_max = %.2f, M = %.2f, U = %.2f, Ui = %.2f, n = %.2f \n',Mmax, Mkorr, U, Ui, n);

end