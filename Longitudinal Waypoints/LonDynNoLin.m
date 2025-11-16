function dxdt = LonDynNoLin(t,x,uct,AC)
%UNTITLED4 Summary of this function goes here
%   x - State vector [V,ga,h,m] [m/s,rad,m,kg]
%   uct - Input vector [CL,T]
%   AC - Aircraft class object ACclass

[a, ~, ~, rho] = atmosisa(x(3)); g = 9.81;
q = 0.5*rho*x(1)^2; 
M = x(1)/a; Re = AC.ReCalc(x(3),M);

dxdt = zeros(4,1);
%[~,u] = IptFor(t);
%uct = LongDynNL_cont; % TODO FINIRE E TOGLIERE COME INPUT
L = q*AC.Sw*uct(1);
T = uct(2);
%T = q*AC.Sw*u(2);
%CL = u(1)/(q*AC.Sw);
D = q*AC.Sw*AC.polar(M,Re,uct(1));

dxdt(1) = ( T - D - x(4)*g*sin(x(2)) )/x(4);
dxdt(2) = ( L - x(4)*g*cos( x(2)) )/(x(4) * x(1));
dxdt(3) = x(1)*sin(x(2));
dxdt(4) = 0; % Constant mass
end