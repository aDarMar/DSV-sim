function [u,u_out] = LatDirControlOut(t,x,y,y_way,AC,GEO)
% LATDIRCONTROLOUT Computes the necessary control inputs for lateral-directional
% control, primarily the commanded roll angle, based on the target waypoint.
% INPUT
%  - t: Current time (scalar).
%  - x: Full state vector: [Va, ga, h, m, CL_i, T_i, Vd_i, Psi, Phi, p, mu, lng].
%  - y: Output vector: [VIAS, M, h, hdot, Phi, p, Psi, VGT, PsiGT].
%  - y_way: Target waypoint vector, including desired trajectory type and parameters.
%  - AC: Aircraft class object.
%  - GEO: Geographical data class object.
% OUTPUT
%  - u: Commanded roll moment [N*m] (or a signal proportional to it).
%  - u_out: Lateral output vector: [PhiC, PsiD, dR].
% -------------------- %
    k = 1; % Placeholder index for control gains (AC.lat)
    % Determine the commanded angles and distance error based on the route type
    u_out = nan(3,1);
    [PhiC,u_out(2),u_out(3)] = wayRoute(x,y,y_way,k,AC,GEO);
    % Calculate the control signal (commanded roll moment)
    % Uses a proportional control law: u = (wn^2) * Phi_c
    u = AC.lat(2)*PhiC;      
    % Store the commanded roll angle
    u_out(1) = PhiC;

end

function [PhiD,PsiD,dR] = wayRoute(x,y,y_way,k,AC,GEO)
% WAYROUTE Defines the commanded bank angle (PhiD) and commanded heading (PsiD)
% required to execute the lateral maneuver defined by the waypoint.
% INPUT
%  - x: Current state vector.
%  - y: Current output vector.
%  - y_way: Target waypoint vector containing trajectory definition.
%  - k: Placeholder for control gain index.
%  - AC: Aircraft class object.
%  - GEO: Geographical data class object.
% OUTPUT
%  - PhiD: Commanded roll angle [rad].
%  - PsiD: Commanded heading [rad].
%  - dR: Lateral distance error from the desired path [m].
% -------------------- %

% Decide the kind of command to use based on the waypoint ID (y_way(5))    
    if y_way(5) == 1 || y_way(5) == 2
    % TF and CF cases both follow a Rhumb line
        flg = 1;
    elseif y_way(5) == 3
    % RF
        flg = 2;
    else
    
    end
    % Heading difference between Ground Track and Aircraft Body heading
    dPsiw = y(9) - x(8);                                    % PsiGT - Psi [rad]
    Vi = sqrt( y(8)^2 + ( y(4)*0.305/60 )^2 );              % Aircraft Speed @ to Ground (CG Speed)
    % Calculate the turn radius for a fixed bank angle
    rt = AC.turnRadius(Vi,0.2967);                          % Radius of  curvature [m]
    switch flg
        case 1
        %% Rhumb Line Tracking
        % Gives the commanded heading to follow a Rhumb line. In this case a
        % desired heading is calculated.
        %   Psi_d = PsiGT_d - dPsi_w
        %   where 
        %   PsiGT_d = Kv - dPsi
        % In this case y_way contains:
        %   - y_way: [1:4,ID,Kv,mu(B),lng(B),Rx,Ry,Rz,mu(A),lng(A)]
            
            dR = rhumbLatDist(x,y_way,GEO);                         % distance from the desired rhumb line [m]
            PsiD = IntercePsi(dR,rt);                               % dPsi [rad]
            PsiD = y_way(6) - PsiD;                                 % PsiGT_d [rad]
            PsiD = PsiD - dPsiw;                                    % Psi_d [rad]
            % Checks if the aircraft should turn left or right
            err = HeadCapture(x(8),PsiD);                           % Minimum Heading Error [rad]
            % Converts the Heading error into bank angle error and 
            % Check if the heading error is too large, limiting the bank
            % angle. The theresold is set at 15°
            if abs(err) > 0.2618                                    % Heading error is > 15°
                PhiD = 0.5236*sign(err);                            % PhiD is set to 30°
            else
                PhiD = AC.lat(3)*err;                               % PhiD [rad] 
            end
        case 2
        %% Circular Arch Tracking
        % ELIMINARE
        % y_way = [---,ID,mu(end),lng(end),Rx(end),
                % Ry(end),Rz(end),rt,psi(end),turn,mu(ctr),lng(ctr),Rx(ctr)
                % Ry(ctr),Rz(ctr),Phic ]
            dR = circleLatDist(x,y_way,GEO);
            PsiD = IntercePsi(dR,y_way(11));    % dPsiD
            
            PhiD = AC.lat(3,k)*PsiD;            % dPhiD
            PhiD = y_way(19) - PhiD;            % PhiD = Phic - dPhi
    end
end

function dR = circleLatDist(x,y_way,GEO)
% ELIMINARE
% y_way = [---,ID,mu(end),lng(end),Rx(end),
                % Ry(end),Rz(end),rt,psi(end),turn,mu(ctr),lng(ctr),Rx(ctr)
                % Ry(ctr),Rz(ctr) ]
% Calcolare vettoer posizione
    Rc = GEO.LatLon2Vec(x(11),x(12),0);
    dR = norm(Rc - y_way(16:18),2); % Distanbe between aircraft and circle center
    dR = y_way(13)*(y_way(11) - dR); % istance from the circonference, positive if right. y_way(1)ì0) = 1 if right turn -1 if left
% Trasformare vettore poizione in NED
% Distanza Vettore-Centro

    
end


function PsiD = IntercePsi(dR,rt)
% INTERCEPSI Calculates the commanded heading angle required to intercept 
% the desired path based on the lateral distance error (dR).
% INPUT
%  - dR: Lateral distance error from the path [m].
%  - rt: Radius of curvature (turn radius) [m].
% OUTPUT
%  - PsiD: Commanded interception angle Psi_c_GT [rad].
% -------------------- %
    PsiD = 0.5*pi*dR/rt; % PsiD_GT in [rad]
    if abs(PsiD) > 0.25*pi
        % Maximum interception sngle at 45deg
        PsiD = 0.25*pi*sign(dR);
    end
end

function dR = rhumbLatDist(x,y_way,GEO)
% RHUMBLATDIST Evaluates the lateral distance from the current position to 
% the desired rhumb line (straight track).
% INPUT
%  - x: Current state vector (containing Lat/Lon at x(11)/x(12)).
%  - y_way: Target waypoint vector (containing target rhumb line parameters).
%  - GEO: Geographical data class object.
% OUTPUT
%  - dR: Lateral distance from the desired rhumb line [m]. 
%        Positive indicates the aircraft is on the right side of the path 
%        relative to the desired course.
% -------------------- %
    % Rhumb line from current position to waypoint
    %[Rnu,m] = GEO.rhumbLine([x(11),y_way(6)],[x(12),y_way(7)]); % Rhumb line passing from the current position to the ending waypoint
    
    %Intersection between commanded rhumb line and rhumb line perpendicular to the first    
    [lati,lngi] = rhxrh(x(11),x(12),y_way(6)-pi/2,...
        y_way(7),y_way(8),y_way(6),'radians'); 
    % Distance along the new rhumb lane between teh aircraft current
    % position and the desired rhumb lane: 
    R1 = GEO.LatLon2Vec(x(11),x(12),0);
    % Vector from current position to waypoint in ECEF
    dV = y_way(9:11) - R1;
    % Transfrom in local NED
    dV = GEO.NED2DIS(dV(:),'E2N',x(11),x(12));
    % Versor normal to the vector pointing to the waypoint
    n = [sin( y_way(6) ),-cos( y_way(6) )];
    dR = n*dV(1:2);
end

function err = HeadCapture(Psi,PsiD)
% HEADCAPTURE Calculates the minimum heading error required to capture 
% the desired heading Psi_c while considering the 360-degree wrap-around.
% INPUT
%  - Psi: Current aircraft heading [rad].
%  - PsiD: Desired aircraft heading [rad].
%
% OUTPUT
%  - err: Minimum signed heading error [rad].
% ----------------------------------------------------------------------- %
    if PsiD - Psi > 0
        eR = PsiD - Psi;
        eL = eR - 2*pi;
    else
        eR = PsiD - Psi + 2*pi;
        eL = PsiD - Psi;
    end
    err = eR;
    % Select the smallest absolute error
    if abs(eL) < abs(eR)
        err = eL;
    end

end