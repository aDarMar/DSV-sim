function [A,B,C,u0] = LongLinSys(x0,AC)
%UNTITLED2 Summary of this function goes here
%   x0 - Vettore delle varibili di stato longitudinali nella cond. di rif.
    % [Va,ga,h]
g = 9.81; gm = 1.4;
[T, a, p, rho] = atmosisa([0,x0(3)]);

CL = AC.m*g*2/(rho(2)*AC.Sw*x0(1)^2); % Trim CL
K = 1/(pi*AC.ARw*AC.e); % K

u0 = zeros(2,1); u0(1) = CL; u0(2) = 0.5*rho(2)*AC.Sw*x0(1)^2*( AC.CD0 + K*(CL-AC.CLmd)^2 );

A = zeros(3);
A(1,1) = -rho(2)*x0(1)*AC.Sw*(AC.CD0 + K*(CL - AC.CLmd)^2)/AC.m;% dfVa/dVa
A(1,2) = -g*cos(x0(2)); % dfVa/dga
A(2,1) = rho(2)*AC.Sw*CL/(2*AC.m) + g/(x0(1)^2)*cos(x0(2)); % dfga/dVa
A(2,2) = g/x0(1) * sin(x0(2)); % dfga/dga
A(3,1) = sin(x0(2));% dfh/dVa
A(3,2) = x0(1)*cos(x0(2));% dfh/dga

B = zeros(3,2);
B(1,1) = -rho(2)*x0(1)^2*AC.Sw*K*(CL - AC.CLmd)/AC.m; % dfVa/dCL
B(1,2) = 1/AC.m; % dfVa/dT
B(2,1) = rho(2)*x0(1)*AC.Sw/(2*AC.m); % dfGa/dCL

C = zeros(3,4);
M = x0(1)/a(2); MMS = (1+0.5*(gm-1)*M^2)^(1/(gm-1));
C(1,1) = a(1)/a(2) * p(2)/p(1) * M*MMS*( p(2)/p(1)*( MMS^gm - 1 ) + 1 )^(-1/gm);
C(1,1) = C(1,1)/ sqrt( 2/(gm-1)*( ( p(2)/p(1)*( MMS^gm - 1 ) + 1 )^(1-1/gm) -1 ) );% dgVIAS/dVa
C(2,1) = 1/a(2); % dgM/dVa
C(3,3) = 1; % dgh/dh
C(4,2) = A(3,2); % dghdot/dga

end

