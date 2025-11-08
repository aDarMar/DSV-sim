function [x_way,y_way,ye,bounds] = BuildWaypoint(GEO,AC,Vs,gas,hs,ms,ways,te,grFlg)
%BUILWAYPOINT Summary of this function goes here
%   y_way: [VIAS,M,h,hdot,PTf, --- ]

    if length(Vs) == length(gas) && length(Vs) == length(ways) && ...
            length(Vs) == length(ms)
        nway = length(Vs);
    else
        error('Some vectors have different length');
    end
    x_way = nan(12,nway);
    y_way = nan(12,nway);
    
    
    for i = 1:nway
        x_way(:,i) = [Vs(i);gas(i);hs(i);ms(i);0;0;0;...
            0;0;0;ways(i).lat*pi/180;ways(i).lng*pi/180];
        y_way(1:9,i) = CompleteDynNoLin_Out(x_way(:,i),[0,0]); % Stores IAS,M,h,hdot]
    
        switch ways(i).ID
            case "T2F"
            % Track to Fix
            % y_way: [---,ID,Kv,mu(B),lng(B),mu(A),lng(A),Rx,Ry]
                y_way(5, i) = 1; % Manouver ID
                % Final Point Coordinates
                y_way(7, i) = ways(i).lat; y_way(8, i) = ways(i).lng;
                % Initial Point Coordinates : NOT NECESSARY FOR THE SIMULATION
                y_way(9, i) = ways(i-1).lat; y_way(10, i) = ways(i-1).lng;
                % Assigns the Azimuth angle
                [y_way(6, i), ~] = GEO.rhumbLine([y_way(9, i), y_way(7, i)]*pi/180,...
                    [y_way(10, i), y_way(8, i)]*pi/180);
                y_way(6, i) = y_way(6, i)*180/pi; % Azimuth in [deg]
                % Position Vecotr of the Final Waypoint for Capture Halo
                Rc = GEO.LatLon2Vec(y_way(7,i)*pi/180, y_way(8,i)*pi/180, 0);
                y_way(11:12, i) = Rc(1:2);
            case "C2F"
            % Course to Fix
            % y_way: [---,ID,Kv,mu(B),lng(B),Rx,Ry,nan,nan]
                y_way(5, i) = 2; % Manouver ID
                % Final Point Coordinates
                y_way(7, i) = ways(i).lat; y_way(8, i) = ways(i).lng;
                % Azimuth
                y_way(6, i) = ways(i).Az;
                % Position Vecotr of the Final Waypoint for Capture Halo
                Rc = GEO.LatLon2Vec(y_way(7,i)*pi/180, y_way(8,i)*pi/180, 0);
                y_way(9:10, i) = Rc(1:2);
            case "R2F"
                % Radius to Fix
                % INCOMPLETOOO
                y_way(5, i) = 3;
                % Obtaining the Center Vector in ECEF coordinates
                Rc = GEO.LatLon2Vec(ways(i).latE*pi/180, ways(i).latE*pi/180, 0);
                y_way(6:7, i) = Rc(1:2);
                % Circle Radius
                y_way(8, i) = way(i).rt;
                % Terminal heading
                y_way(9, i) = way(i).PsiE*pi/180;
                % Left/Right Turn
                y_way(10, i) = 1;
                if way(i).turn == 'l'
                    y_way(10, i) = -1;
                end
            otherwise
        % Skips the first element because it is the initial condition
    
        end
    
    
    end
    ye = x_way(:,1);   % Initial condition
    [~,~,~,ye(5:6)] = AC.LongLinSys(x_way(:,1),x_way(4,1)); % CL,T at t0 only for the first
    % Initial COndition
    ye(7:end) = 0;  
    temp = [0,y_way(1,1),zeros(1,2),-1];                      % [ID,VIAS,dVd/dt,x(7),Kh]
    bounds = UpdateBounds( x_way(:,1) );                    % Longitudinal Bounds
    % Perform check to assign correct T0
    [u,temp,~] = LongControlOut(te,[x_way(1:4,1);0;0;0],...
        y_way(1:4,1),temp,bounds,AC);
    if temp(1) ~= 7
        % Zones other than 7 have maximum or idle thrust so it is set the
        % value they will have as initial condition to avoid starting with
        % 0 thrust
        ye(6) = u(2);
    end



    if grFlg
        fig = figure('Name','Waypoint Map');
        %ax = axes('Parent',fig); hold(ax,'on');
        for i =1:nway
            switch y_way(5,i) 
                case 1
                    [lattrk,lontrk] = track2("rh",y_way(9,i),y_way(10,i),...
                        y_way(7,i),y_way(8,i),GEO,'degrees');  % Desired rhumb path
                    geoplot(lattrk,lontrk,'r','LineWidth',1.4); hold on;
                    geoplot(lattrk(1),lontrk(1),'Marker','diamond','LineWidth',1.5);
                    geoplot(lattrk(end),lontrk(end),'Marker','diamond','LineWidth',1.5);
                case 2
                    [lattrk,lontrk] = track1("rh",y_way(7,i),y_way(8,i),...
                        y_way(6,i),100000,GEO,'degrees');
                    geoplot(lattrk,lontrk,'--b','LineWidth',1.4);
                    geoplot(y_way(7,i),y_way(8,i),'Marker','diamond','LineWidth',1.5);
            end
            
        end
    end
end

