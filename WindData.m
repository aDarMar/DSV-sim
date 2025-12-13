%close all; clearvars; clc
% Module that returns a Wind.mat file used to simulate wind presence in the
% 4DoF NAPE Model.
%   OUTPUT
%   - [lng,lat,h,Vwx,Vwy]: Vwx,Vwy are the wind components in the local NED
%       frame. lng, lat sono latitudine (geodetica) e quota in rad.
% Ref: https://www.unavco.org/software/geodetic-utilities/geoid-height-calculator/geoid-height-calculator.html
% ----------------------------------------------------------------------- %
lngsp = -180:10:180;
latsp = -90:5:90;
hsp = 0:100:6000;
[lng,lat,hsp] = meshgrid(lngsp,latsp,hsp);
lng = lng(:); lat = lat(:); hsp = hsp(:);
CHS = 'cst';
switch CHS
    case 'cst'
        Vxc = 1.2; Vyc = 2.1;
        w = [lng*0+Vxc,lng*0+Vyc];
    case 'modl'
        N = geoidheight(lat,lng,'EGM96','warning'); % Geoid Height: gives a warning if longitude is betwee -180 and 180 but it works
        %lng(lng>180) = lng(lng>180) - 360;
        hsp = hsp(:) - N(:);      % h ort = hellips - Ngeo
        hsp(hsp<0) = 0;
        w = atmoshwm(lat(:),lng(:),hsp(:)-N(:),'model','quiet'); % Meridional (xN) and zonal (xE) wind
end

windpx = scatteredInterpolant( lng*pi/180,lat*180/pi,hsp-N(:),w(:,1) );
windpy = scatteredInterpolant( lng*pi/180,lat*180/pi,hsp-N(:),w(:,2) );
save('Data\Wind_Simp.mat','windpx','windpy');