function [Mkorr] = drehmomentKorrektur(M,n,Ucell,ncell,type)
% Beschreibung
% M: Drehmoment in Nm
% n: Drehzhal in rad/s
% Mkorr: Drehmoment in Nm (Mkorr == M oder Mkorr(n) < M)


U = Ucell*ncell*0.96;

switch(type)
    case 1
        % F80 KV1900
        R = 0.036;
        kn = 1900/60*2*pi;  % [rad/(s*V)]
        kn = 1/kn;          % [s*V/rad]
        kt = kn;            % [Nm/A]
        Mmax = 40*kt;
        
    case 2
        % F40 PROII HV1600
        R = 0.082;
        kn = 1600/60*2*pi;  % [rad/(s*V)]
        kn = 1/kn;          % [s*V/rad]
        kt = kn;            % [Nm/A]
        Mmax = 40*kt;
    otherwise
        
end

% Maximales Drehmoment
if M>Mmax
    M = Mmax;
end

Ui = kn*n;
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


end