close all; clear;clc

addpath('Classes');
addpath('Functions');
addpath('Latero-Directional Waypoints')
% Waypoints
lats = [35,20;45,34.7];
lng = [17,20.7;27,55.7];
flg = {'N','N','E','E'};

GEO = GeoClass();

cnv = GEO.LatLon2Rad([lats;lng],flg);
[Rv,m] = GEO.rhumbLineOLD(cnv(1:2),cnv(3:4));
[Rv1,m1] = GEO.rhumbLine(cnv(1:2),cnv(3:4));

Rv2 = azimuth("rh",cnv(1),cnv(3),cnv(2),cnv(4),GEO,'radians');
m2 = distance("rh",cnv(1),cnv(3),cnv(2),cnv(4),GEO,'radians');

y = nan(9,1); y(5) = Rv; y(6:7) = [cnv(2);cnv(4)];
y(8:9) = [cnv(1);cnv(3)];
% Current Position
x = nan(12,1);
x(11) = 36.5*pi/180; x(12) = 23.4*pi/180;
x(11) = 42.5*pi/180; x(12) = 20.4*pi/180;
wayRoute(x,y,GEO);

% Current Position is on the Rhumb path
[lati,lngi] = rhxrh(x(11),x(12),y(5)-pi/2,...
        y(6),y(7),y(5),'radians');
x(11) = lati; x(12) = lngi;
wayRoute(x,y,GEO);
%LatDirControlOut([],x,y,[],[],[],GEO)

%% DEBUG di wayRoute

function dR = wayRoute(x,y_way,GEO)
    % Rhumb Line Tracking
    dR = rhumbLatDist(x,y_way,GEO);

end

function dR = rhumbLatDist(x,y_way,GEO)

    % Rhumb line from current position to waypoint
    [Rnu,m] = GEO.rhumbLine([x(11),y_way(6)],[x(12),y_way(7)]); % Rhumb line passing from teh current position to the ending waypoint
    R1 = [m*cos(Rnu),m*sin(Rnu)]; % Distance vector in Mercator plane
    % Test 1
    n1 = [sin(y_way(5)),-cos(y_way(5))]; % Normal to to the rhumb line vector in Mercator plane
    dR = R1*n1(:); % USARE MATLAB
    % Test 2: intersecting rhumb lines
    [lati,lngi] = rhxrh(x(11),x(12),y_way(5)-pi/2,...
        y_way(6),y_way(7),y_way(5),'radians'); %Interectio nbetween commanded rhumb line and rhumb line perpendicular to the first
    %lati = lati*pi/180; lngi = lngi*pi/180; % deg -> rad
    [Rnu,m] = GEO.rhumbLine([x(11),lati],[x(12),lngi]); % Rhumb line passing from teh current position to the ending waypoint
    figure()
    [lattrk,lontrk] = track2("rh",y_way(6),y_way(7),y_way(8),y_way(9),GEO,'radians'); % Desired rhumb path
    geoplot(lattrk*180/pi,lontrk*180/pi,'r','LineWidth',1); hold on
    
    [lattrk,lontrk] = track2("rh",x(11),x(12),y_way(6),y_way(7),GEO,'radians'); % Rhumb line from current pos to waypoint
    geoplot(lattrk*180/pi,lontrk*180/pi,'--b','LineWidth',1);
    
    [lattrk,lontrk] = track2("rh",x(11),x(12),lati,lngi,GEO,'radians'); % Rhumb line from current pos to desired rhumb line
    geoplot(lattrk*180/pi,lontrk*180/pi,'--k','LineWidth',0.75);


end