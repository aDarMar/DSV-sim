function [x_way,y_way,ye,bounds,temp] = BuildWaypoint(GEO,AC,Vs,gas,hs,ms,ways,te,grFlg)
%BUILWAYPOINT Summary of this function goes here
%   OUTPUT
%   - y_way: [VIAS,M,h,hdot,PTf, --- ]. The first column is the initial
%           condition
%   - ye: initial state vector
    if length(Vs) == length(gas) && length(Vs) == length(ways) && ...
            length(Vs) == length(ms)
        nway = length(Vs);% It is actually number of waypoints + 1 because it includes the initial condition
    else
        error('Some vectors have different length');
    end
    x_way = nan(12,nway);
    y_way = nan(19,nway);
    
    
    for i = 1:nway
        x_way(:,i) = [Vs(i);gas(i);hs(i);ms(i);0;0;0;...
            0;0;0;ways(i).lat*pi/180;ways(i).lng*pi/180];
        y_way(1:9,i) = CompleteDynNoLin_Out(x_way(:,i),[0,0]); % Stores IAS,M,h,hdot]
    
        switch ways(i).ID
            case "T2F"
            % Track to Fix
            % y_way: [---,ID,Kv,mu(B),lng(B),Rx,Ry,Rz,mu(A),lng(A)] measured
            % in radiants
                y_way(5, i) = 1; % Manouver ID
                % Final Point Coordinates [rad]
                y_way(7, i) = ways(i).lat*pi/180; 
                y_way(8, i) = ways(i).lng*pi/180;
                % Initial Point Coordinates : NOT NECESSARY FOR THE SIMULATION
                y_way(12, i) = ways(i-1).lat*pi/180; 
                y_way(13, i) = ways(i-1).lng*pi/180;
                % Assigns the Azimuth angle
                [y_way(6, i), ~] = GEO.rhumbLine([y_way(12, i), y_way(7, i)],...
                    [y_way(13, i), y_way(8, i)]);
                y_way(6, i) = y_way(6, i); % Azimuth in [rad]
                % Position Vecotr of the Final Waypoint for Capture Halo
                Rc = GEO.LatLon2Vec(y_way(7,i), y_way(8,i), 0);
                y_way(9:11, i) = Rc(1:3);
            case "C2F"
            % Course to Fix
            % y_way: [---,ID,Kv,mu(B),lng(B),Rx,Ry,Rz,nan,nan]
                y_way(5, i) = 2; % Manouver ID
                % Final Point Coordinates [rad]
                y_way(7, i) = ways(i).lat*pi/180; 
                y_way(8, i) = ways(i).lng*pi/180; 
                % Azimuth [rad]
                y_way(6, i) = ways(i).Az*pi/180; 
                % Position Vecotr of the Final Waypoint for Capture Halo
                Rc = GEO.LatLon2Vec(y_way(7,i), y_way(8,i), 0);
                y_way(9:11, i) = Rc(1:3);
            case "R2F"
                % Radius to Fix: the route is build giving the coordinates
                % of teh center and the heading of teh termiantor. The
                % radius is obtained as the distance fromt the cenetr of
                % the previous waypoint/initial condition
                % y_way = [---,ID,mu(end),lng(end),Rx(end),
                % Ry(end),Rz(end),rt,psi(end),turn,mu(ctr),lng(ctr),Rx(ctr)
                % Ry(ctr),Rz(ctr),Phic ]
                y_way(5, i) = 3;                        % Manouver ID
                % Terminal heading
                y_way(12, i) = ways(i).PsiE*pi/180;
                % Center Coordinates
                y_way(14, i) = ways(i).lat*pi/180; 
                y_way(15, i) = ways(i).lng*pi/180; 
                % Obtaining the Center Vector in ECEF coordinates
                Rc = GEO.LatLon2Vec(ways(i).lat*pi/180, ways(i).lng*pi/180, 0);
                y_way(16:18, i) = Rc;
                % Left/Right Turn
                y_way(13, i) = sign(ways(i).t);
                % Terminator Point
                R = y_way(9:11,i-1)-y_way(16:18,i);     % Vector pointing from center to starting point
                y_way(11, i) = norm(R,2);               % Turn Radius
                
                [hed,hdf,dhdf] = deltaHead(...          % Delta Heading
                    y_way(13,i),y_way(12,i),R);
                % Rotate radius vector by delta Heding
                n = [cos(dhdf),-sin(dhdf),0;...
                    sin(dhdf),cos(dhdf),0;...
                    0,0,1]*R;                           % Rotates radius vector to final heading 
 
                
                n = GEO.NED2DIS(n,'N2E',y_way(14,i),y_way(15,i));
                y_way(8:10,i) = n + y_way(16:18,i);                             % Position Vector of Termination point: it is the sum of the vector identifying the center plus the rotated vector
                [y_way(6,i),y_way(7,i),h] = ecef2geodetic( GEO,y_way(8,i),...   % Latitue and Longitude of termiantor point
                    y_way(9,i),y_way(10,i),'radians' );                         % h may be negative i guess
                % Desired Bank Angle
                V = IAS2TAS(y_way(1,i-1),y_way(3,i-1),'ktsh');                  % NOTE: It is assumed no wind
                V = sqrt( V^2 - (y_way(4)*0.305/60)^2 );                        % Ground Speed
                y_way(19,i) = acos( 1/sqrt( V^2/( 9.81*y_way(11,i) ) + 1 ) );
                if y_way(19,i) > 0.5236
                    error('Cannot perform the turn with a bank agle no more than 30 degrees')
                end
                    
            otherwise
        % Skips the first element because it is the initial condition
                y_way(5,i) = 0;
                % Starting Point Coordinates [rad]
                y_way(7, i) = ways(i).lat*pi/180; 
                y_way(8, i) = ways(i).lng*pi/180; 
        end
    
    end
    %nway = nway-1;                                              % Remove the initial condition
    %y_way = y_way(:,2:end);                                     % The first column is the initial condition
    ye = x_way(:,1);                                            % Initial condition
    [~,~,~,ye(5:6)] = AC.LongLinSys(x_way(:,1),x_way(4,1));     % CL,T at t0 only for the first
    % Initial COndition
    %ye(7:end) = 0;  
    temp = [0,y_way(1,1),zeros(1,2),-1];                        % [ID,VIAS,dVd/dt,x(7),Kh]
    bounds = UpdateBounds( x_way(:,1) );                        % Longitudinal Bounds
    % Perform check to assign correct T0
    [u,temp,~] = LongControlOut(te,[x_way(1:4,1);0;0;0],...     % VEDERE SE Kh =/= -1 va bene
        y_way(1:4,1),temp,bounds,AC);
    temp(5) = -1;                                               % Initial Kh
    if temp(1) ~= 7
        % Zones other than 7 have maximum or idle thrust so it is set the
        % value they will have as initial condition to avoid starting with
        % 0 thrust
        ye(6) = u(2);
    end



    if grFlg
        fig = figure('Name','Waypoint Map');
        %ax = axes('Parent',fig); hold(ax,'on');
        plotMap(y_way,GEO);
    end
end

