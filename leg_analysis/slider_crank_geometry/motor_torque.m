function [Mkorr] = motor_torque(M,n,Ucell,ncell)
% Beschreibung
% M: Drehmoment in Nm
% n: Drehzhal in rad/s
% Mkorr: Drehmoment in Nm (Mkorr == M oder Mkorr(n) < M)


U = Ucell*ncell*0.96;

% T Motors A80-9
R = 0.175;                      % [Ohm]
kv_ = 105;                      % [RPM/V]
kv = kv_ * 2 * pi / 60;         % [rad/s / V]
ke = 1 / kv;                    % [V / rad/s]
kt = 0.091;                     % [Nm/A]
Mmax = 2;                       % [Nm]

        
% Maximales Drehmoment
if M>Mmax
    M = Mmax;
end

Ui = ke*n;

if Ui >= U
    Mkorr = 0;
else
    Ud = U-Ui;
    I = M/kt;
    Ur = R*I;
    
    if Ud > Ur
        Mkorr = M;
    else
        Mkorr = kt*(Ud/R);
    end
end


%fprintf('M_max = %.2f, M = %.2f, U = %.2f, Ui = %.2f, n = %.2f \n',Mmax, Mkorr, U, Ui, n);

end