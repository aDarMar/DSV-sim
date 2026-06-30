function y = CompleteDynNoLin_Out(x,Vw,gm)
%COMPLETEDYNNOLIN_OUT Calculates the state output vector.
%   This function transforms the full aircraft state vector into a vector
%   of output variables. 
%   INPUT
%   - x: Aircraft state vector. Can be a matrix, where the i-th column
%   represents the state vector at a time t.
%        The state vector components along a column are:
%        [Va, ga, ha, m, ICL, It, Vd, Psi, Phi, p, mu, lng]
%        where:
%        * Va: True Airspeed [m/s]
%        * ga: Flight Path Angle [rad]
%        * ha: Altitude [m]
%        * m: Aircraft Mass [kg]
%        * ICL, It, Vd: Auxiliary/Internal variables
%        * Psi: Heading Angle [rad]
%        * Phi: Bank Angle [rad]
%        * p: Roll rate [rad/s]
%        * mu: Geodetic Latitude Angle [rad] (not directly used in output)
%        * lng: Longitude [rad] (not directly used in output)
%   - Vw: wind speed along Vx,Vy in NED [m/s]
%   - gm: (optional) cp/cv ratio, specify if not 1.4
%   OUTPUT
%   - y: Output vector.
%        The output vector components are:
%        [VIAS, M, h, hdot, Phi_deg, p, Psi_deg, Vg, PsiG]
%        with units:
%        [kts, -, ft, fpm, deg, rad/s, deg, m/s, rad]
%        * VIAS: Indicated Airspeed [kts]
%        * M: Mach Number [-]
%        * h: Altitude [ft]
%        * hdot: Rate of climb/descent [fpm]
%        * Phi_deg: Bank Angle [deg]
%        * p: Roll rate [rad/s]
%        * Psi_deg: Heading Angle [deg]
%        * Vg: Ground Speed [m/s]
%        * PsiG: Ground Track Angle [rad]
% Tested comparing TAS <-> IAS with IAS2TAS
% ----------------------------------------------------------------------- %
    % Initialization of variables for atmospheric parameters
    a = nan(2,1); P = a; y = nan( 9,length( x(1,:) ) );
    [~,a(1),P(1),~] = atmosisa( 0 );
    % Assign default value for gm if not provided as input
    if nargin < 3
        gm = 1.4;
    end
    % Loop through the state matrix columns
    for i = 1:length( x(1,:) )
        % Calculation of atmospheric parameters at the current altitude
        [~,a(2),P(2),~] = atmosisa( x(3,i) );
    
        y(2,i) = x(1,i)/a(2);                       %/*$\leftarrow$ \eqref{eq:out:subeq:M}*/
        y(1,i) = a(1) * sqrt( 2/(gm-1)*( ( P(2)/P(1)*( ...
            (1 + 0.5*(gm-1)*y(2,i)^2)^( gm/(gm-1) ) - 1 ) + 1 )...
            ^((gm-1)/gm) - 1 ) );                   %/*$\leftarrow$ \eqref{eq:out:subeq:IAS}*/
        y(1,i) = y(1,i)/0.5144;                     % m/s to kts
        y(3,i) = x(3,i)/0.305;                      % Altitude m to ft
        y(4,i) = x(1,i)*sin(x(2,i))/0.305*60;       %/*$\leftarrow$ \eqref{eq:NAPE_h} in [ft]*/
        y(5,i) = x(8,i)*180/pi;                     % Heading Angle [deg]
        y(6,i) = x(10,i);                           % Roll rate (p) [rad/s]
        y(7,i) = x(9,i)*180/pi;                     % Bank angle [deg]
    
        y(8,i) = sqrt( ( Vw(1) + x(1,i)*cos(x(2,i))*cos(x(8,i)) )^2 + ...
            ( Vw(2) + x(1,i)*cos(x(2,i))*sin(x(8,i)) )^2 );                 % Ground Speed [m/s]
        y(9,:) = atan2( ( Vw(2) + x(1,i)*cos(x(2,i))*sin(x(8,i)) ),...
            ( Vw(1) + x(1,i)*cos(x(2,i))*cos(x(8,i)) ) );                   % Ground Heading [rad]
    end
end

