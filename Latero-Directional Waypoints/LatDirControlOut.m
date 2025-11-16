function [u,u_out] = LatDirControlOut(t,x,y,y_way,AC,GEO)
%LATDIRCONTROLOUT Summary of this function goes here
%   INPUT
%   - x: [Va.ga,h,m,CL,T,Vd,Psi,Phi,p,mu,lng]
%   - y: [VIAS,M,h,hdot,phi,p,psi,Vg,psiG]
%   - y_way: [VIAS,M,h,hdot,PTf, --- ]
%   OUTPUT
%   - u_out [PhiC,PsiC,dR]
%   - x_add: [ID,VIAS,dVd/dt,x(7),Kh] evaluated at previous successful iteration
%   - bounds: [vbound,hbound,V/g] in SI units
%   OUTPUT
%   - u: Roll moment [N*m]

    %u_out(1) = ZoneIdf(err,bounds);     % Identifies the zone in which the aircraft is                 Flag vector [ slow,fast,Low,High,lowenergy]                                                                 % Force Vector [Roll]
    k = 1; % TEMPORANEOOOO

    u_out = nan(3,1);
    [PhiC,u_out(2),u_out(3)] = wayRoute(x,y,y_way,k,AC,GEO);
    u = AC.lat(2)*PhiC;                 % wn^2*PhiD
    u_out(1) = PhiC;

end

function [PhiD,PsiD,dR] = wayRoute(x,y,y_way,k,AC,GEO)
%WAYROUTE: Function that defines the commanded bank angle to execute the
%commanded lateral manouver
%   INPUT
%   -   x
%   - y
%   - y_way
%   - k
%   OUTPUT
%   - PhiD: commanded roll angle [rad]
% Decide teh kind of command to use
    if y_way(5) == 1 || y_way(5) == 2
        % TF and CF cases both follow a rhumb line
        flg = 1;
    elseif y_way(5) == 3
        %% RF
        flg = 2;
    
    else
    
    end
    dPsiw = y(9) - x(8);                                    % PsiGT - Psi [rad]
    Vi = sqrt( y(8)^2 + ( y(4)*0.305/60 )^2 );              % Aircraft Speed @ to Ground (CG Speed)
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
            % Converts the Heading error into bank angle error: firt check
            % if the heading error is too big causing a too great bank
            % angle. The theresold is set at 15°
            if abs(err) > 0.2618                                         % Heading error is > 15°
                PhiD = 0.5236*sign(err);                                      % PhiD is set to 30°
            else
                PhiD = AC.lat(3)*err;                               % PhiD [rad] 
            end
        case 2
        %% Circular Arch Tracking
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
    PsiD = 0.5*pi*dR/rt; % PsiD_GT in [rad]
    if abs(PsiD) > 0.25*pi
        % Maximum interception sngle at 45deg
        PsiD = 0.25*pi*sign(dR);
    end
end

function dR = rhumbLatDist(x,y_way,GEO)
% RHUMBLATDIST: function that evaluates the distance from the current
% position to the desired rhumb line. The distance is evaluated along a
% second rhumb line that intersects the first and starts from the current
% position.
%   INPUT

%   OUTPUT
%       - dR: distance from teh desired rhumb lane along a rhumb lane that
%       connects the aircraft and intersects the rhumb lane [m]

    % Rhumb line from current position to waypoint
    %[Rnu,m] = GEO.rhumbLine([x(11),y_way(6)],[x(12),y_way(7)]); % Rhumb line passing from the current position to the ending waypoint
    
    %Intersection between commanded rhumb line and rhumb line perpendicular to the first    
    [lati,lngi] = rhxrh(x(11),x(12),y_way(6)-pi/2,...
        y_way(7),y_way(8),y_way(6),'radians'); 
    % Distance along the new rhumb lane between teh aircraft current
    % position and the desired rhumb lane
    [~,dR] = GEO.rhumbLine([x(11),lati],[x(12),lngi]);                         
    R1 = GEO.LatLon2Vec(x(11),x(12),0);
    dV = y_way(9:11) - R1;
    dV = GEO.NED2DIS(dV(:),'E2N',x(11),x(12));
    n = [sin( y_way(6) ),-cos( y_way(6) )];
    dR2 = n*dV(1:2);
    dR = dR*sign(dR2); % Positive if on the right
    %dR = dR2;
    % R1 = [m*cos(Rnu),m*sin(Rnu)]; % Distance vector in Mercator plane
    % % Test 1
    % n1 = [sin(y_way(5)),-cos(y_way(5))]; % Normal to to the rhumb line vector in Mercator plane
    % dR = R1*n1(:); % USARE MATLAB
    % % Test 2: intersecting rhumb lines
    % 
    % %lati = lati*pi/180; lngi = lngi*pi/180; % deg -> rad
    % [Rnu,m] = GEO.rhumbLine([x(11),lati],[x(12),lngi]); % Rhumb line passing from teh current position to the ending waypoint
    % 
    % [lattrk,lontrk] = track2("rh",y_way(6),y_way(7),y_way(8),y_way(9),GEO,'radians'); % Desired rhumb path
    % geoplot(lattrk*180/pi,lontrk*180/pi,'r','LineWidth',1); hold on
    % 
    % [lattrk,lontrk] = track2("rh",x(11),x(12),y_way(6),y_way(7),GEO,'radians'); % Rhumb line from current pos to waypoint
    % geoplot(lattrk*180/pi,lontrk*180/pi,'--b','LineWidth',1);
    % 
    % [lattrk,lontrk] = track2("rh",x(11),x(12),lati,lngi,GEO,'radians'); % Rhumb line from current pos to desired rhumb line
    % geoplot(lattrk*180/pi,lontrk*180/pi,'--k','LineWidth',0.75);


end

function err = HeadCapture(Psi,PsiD)

    if PsiD - Psi > 0
        eR = PsiD - Psi;
        eL = eR - 2*pi;
    else
        eR = PsiD - Psi + 2*pi;
        eL = PsiD - Psi;
    end
    err = eR;
    if abs(eL) < abs(eR)
        err = eL;
    end

end