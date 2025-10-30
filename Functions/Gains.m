function [Kp,Ki,Kb] = Gains(wn,zita,Ai,Bi,Ci,D)
%GAINS Summary of this function goes here
%   Detailed explanation goes here

% CL control of hdot
Kb = zeros(2,4); Ki = zeros(2,4); Kp = zeros(2,4);                          % Gain Matrices

wns = wn^2; zwn = 2*zita*wn;
GN = [ Bi(2,1)*Ci(4,2),0,1; ...
    Ci(4,2)*( Ai(2,1)*Bi(1,1) - Ai(1,1)*Bi(2,1) ), 0 , zwn; ...
    0, Ci(4,2)*( Ai(2,1)*Bi(1,1) - Ai(1,1)*Bi(2,1) ), wns ];
RHS = [ Ai(1,1)+zwn;wns + Ai(1,2)*Ai(2,1); 0];

kcl = GN\RHS; Kp(1,4) = kcl(1); Ki(1,4) = kcl(2);

rLocusDef(Ai,Bi,Ci,4,1);                                                   % Kp14
Kit = zeros(4,4); Kit(1,4) = kcl(1); Ain = Ai - Bi*Kit*Ci;
rLocusDef(Ain,Bi,Ci,4,3); 
Kit = zeros(4,4); Kit(3,4) = kcl(2); Ain = Ain - Bi*Kit*Ci;                % Ki14
lam = eig(Ain);

end

