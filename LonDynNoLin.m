function dxdt = LonDynNoLin(t,x,AC)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

[~, ~, ~, rho] = atmosisa(x(3)); g = 9.81;
q = 0.5*rho*x(1)^2;
K = 1/(pi*AC.ARw*AC.e);
dxdt = zeros(3,1);
[~,u] = IptFor(t);

L = q*AC.Sw*u(1);
D = q*AC.Sw*( AC.CD0 + K*(u(1) - AC.CLmd)^2 );

dxdt(1) = ( u(2) - D - AC.m*g*sin(x(2)) )/AC.m;
dxdt(2) = ( L - AC.m*g*cos(x(2)) )/(AC.m * x(1));
dxdt(3) = x(1)*sin(x(2));

end

