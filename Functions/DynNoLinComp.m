function dxdt = DynNoLinComp(t,x,uct,AC,GEO,V)
%UNTITLED4 Summary of this function goes here
%   x - State vector [V,ga,h,m,CL,T,VD,Psi,phi,p] [m/s,rad,m,kg,-,N,kts,rad,rad,rad/s]
%   uct - Input vector [CL,T]
%   AC - Aircraft class object ACclass
%   GEO - struct or class ???? containing data abouth earth model
%   V   - CG velocity [m/s]
[a, ~, ~, rho] = atmosisa(x(3)); g = 9.81;
q = 0.5*rho*x(1)^2; 
M = x(1)/a; Re = AC.ReCalc(x(3),M);

dxdt = zeros(12,1);
%[~,u] = IptFor(t);
%uct = LongDynNL_cont; % TODO FINIRE E TOGLIERE COME INPUT
L = q*AC.Sw*uct(1);
T = uct(2);
%T = q*AC.Sw*u(2);
%CL = u(1)/(q*AC.Sw);
D = q*AC.Sw*AC.polar(M,Re,uct(1));

dxdt(1) = ( T - D - x(4)*g*sin(x(2)) )/x(4);                    %dVa
dxdt(2) = ( L*cos( x(9) ) - x(4)*g*cos( x(2)) )/(x(4) * x(1));  %dga
dxdt(3) = x(1)*sin(x(2));                                       %dh
dxdt(4) = 0; % Constant mass                                    %dm

dxdt(8) = ( L*sin( x(9) ) )/( x(4)*x(1)*cos(x(2)) );            % dPsi
dxdt(9) = x(10);
dxdt(10) = AC.Lp*x(10) + AC.Lda*u(3);

[Rmu,Dmu] = GEO.RadCurv(x(11));
dxdt(11) = V(1)/(Rmu+x(3));                                     % Geodetic Latitude
dxdt(12) = V(2)/( (Dmu+x(3))*cos(x(11)) );                      % Longitude

end