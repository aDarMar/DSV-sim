function [rl] = rLocusDef(Ai,Bi,Ci,iOut,iFor)

% Trasformazione in SISO
%iout = 4; % 1 - VIAS 3 - h
%iFor = 2; % 1,3 - CL 2,4 - T

CiSISO = Ci(iOut,:); DiSISO = zeros(1,1);
BiSISO = Bi(:,iFor);
lonsys = ss(Ai,BiSISO,CiSISO,DiSISO);
%r = rlocus(lonsys);

figure()
rl = rlocusplot(lonsys);
end