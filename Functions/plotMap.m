function [outputArg1,outputArg2] = plotMap(y_way,GEO,ax)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if nargin < 3
    ax = geoaxes();
end
hold(ax,'on');
nway = length(y_way(1,:));
    for i =1:nway
        switch y_way(5,i) 
            case 0
                geoplot(ax,y_way(7,i)*180/pi,y_way(8,i)*180/pi,...
                    'Marker','diamond','LineWidth',1.5);
            case 1
                [lattrk,lontrk] = track2("rh",y_way(12,i)*180/pi,...
                    y_way(13,i)*180/pi,y_way(7,i)*180/pi,...
                    y_way(8,i)*180/pi,GEO,'degrees');       % Desired rhumb path
                geoplot(ax,lattrk,lontrk,':r','LineWidth',1.4); hold on;
                geoplot(ax,lattrk(1),lontrk(1),'Marker','diamond','LineWidth',1.5);
                geoplot(lattrk(end),lontrk(end),'Marker','diamond','LineWidth',1.5);
            case 2
                [lattrk,lontrk] = track1("rh",y_way(7,i)*180/pi,...
                    y_way(8,i)*180/pi,y_way(6,i)*180/pi,100000,GEO,'degrees');
                geoplot(ax,lattrk,lontrk,'--b','LineWidth',1.4);
                geoplot(ax,y_way(7,i)*180/pi,y_way(8,i)*180/pi,'Marker',...
                    'diamond','LineWidth',1.5);
                % TEMP FUNZIOAN SOLO SE NON E' IL PRIMO WAYPOINT
                [lai,lngi] = rhxrh(y_way(7,i-1),y_way(8,i-1),y_way(6,i-1),...
                    y_way(7,i),y_way(8,i),y_way(6,i),'radians');
                [lattrk,lontrk] = track2("rh",lai*180/pi,...
                    lngi*180/pi,y_way(7,i)*180/pi,...
                    y_way(8,i)*180/pi,GEO,'degrees');       % Desired rhumb path
                geoplot(ax,lattrk,lontrk,':r','LineWidth',1.4);
            case 3
                % Circle path
                % y_way = [---,ID,mu(end),lng(end),Rx(end),
                % Ry(end),Rz(end),rt,psi(end),turn,mu(ctr),lng(ctr),Rx(ctr)
                % Ry(ctr),Rz(ctr) ]
                geoplot(ax,y_way(6,i)*180/pi,y_way(7,i)*180/pi,...
                    'Marker','diamond','LineWidth',1.4);                    % Terminator Point
                geoplot(ax,y_way(14,i)*180/pi,y_way(15,i)*180/pi,...
                    'Marker','o','LineWidth',1.4);                          % Center Point
                % Circular Path
                Ri = y_way(9:11,i-1) - y_way(16:18,i);    % The vector must be pointing inside the node to get the correct sign for heading
                Ri = GEO.NED2DIS(Ri,'E2N',y_way(13,i),y_way(14,i)); % Position Vector in NED coordinates
                
                np = 10;
                [hd,hdf,dhdf] = deltaHead(y_way(13,i),y_way(12,i),Ri);
                hds = linspace(min(0,dhdf),max(0,dhdf),np);
                
                for ij = 1:np
                    ni = [cos(hds(ij)),-sin(hds(ij)),0;...
                    sin(hds(ij)),cos(hds(ij)),0;...
                    0,0,1]*Ri;%[y_way(11,i);0;0];             % Rotate radius vector
                    ni = GEO.NED2DIS(ni,'N2E',y_way(13,i),y_way(14,i)); 
                    Rv = ni + y_way(16:18,i);               % Position Vector in ECEF
                    
                    [lat(ij),lng(ij)] = ecef2geodetic(GEO,Rv(1),Rv(2),Rv(3));
                    
                end
            geoplot(ax,lat,lng,':r','LineWidth',1.5)
        end
        
    end
end

