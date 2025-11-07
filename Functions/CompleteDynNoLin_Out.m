function y = CompleteDynNoLin_Out(x,Vw,gm)
%LongDynNoLin_Out Function that returns the state output
%   INPUT
%   - x: longitudinal state vector [Va,ga,ha]. It can be passed as a matrix
%   - Vw: wind speed along Vx,Vy in NED [m/s]
%   - gm: cp/cv ratio, specify if not 1.4
%   OUTPUT
%   - y: longitudinal output vector [VIAS,M,h,hdot,phi,p,psi,Vg,psiG] in 
%           [kts,-,ft,fpm,deg,deg,rad/s,m/s,rad]
% Tested comparing TAS <-> IAS with IAS2TAS
    a = nan(2,1); P = a; y = nan( 9,length( x(1,:) ) );
    [~,a(1),P(1),~] = atmosisa( 0 );
    if nargin < 2
            gm = 1.4;
    end
    
    for i = 1:length( x(1,:) )
        [~,a(2),P(2),~] = atmosisa( x(3,i) );
        y(2,i) = x(1,i)/a(2);
        y(1,i) = a(1) * sqrt( 2/(gm-1)*( ( P(2)/P(1)*( ...
            (1 + 0.5*(gm-1)*y(2,i)^2)^( gm/(gm-1) ) - 1 ) + 1 )^((gm-1)/gm) - 1 ) );
        y(1,i) = y(1,i)/0.5144; % m/s to kts
        y(3,i) = x(3,i)/0.305;  % m to ft
        y(4,i) = x(1,i)*sin(x(2,i))/0.305*60;                           %m/s to fpm
        y(5,i) = x(8)*180/pi;                                           % Heading Angle [deg]
        y(6,i) = x(10);                                                 % Roll speed [rad/s] DECIDERE SE TENERE
        y(7,i) = x(9)*180/pi;                                           % Bank angle [deg]

        y(8,i) = sqrt( ( Vw(1) + x(1)*cos(x(2))*cos(x(8)) )^2 + ...
            ( Vw(2) + x(1)*cos(x(2))*sin(x(8)) )^2 );                   % Ground Speed
        y(9,:) = atan2( ( Vw(2) + x(1)*cos(x(2))*sin(x(8)) ),...
            ( Vw(1) + x(1)*cos(x(2))*cos(x(8)) ) );                     % Ground Heading
    end
end

