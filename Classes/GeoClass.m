classdef GeoClass < referenceEllipsoid 
    %GEOCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %a
        %b
        %e
        %f
    end
    
    methods
        function obj = GeoClass()
            %GEOCLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@referenceEllipsoid('WGS84');
        end
        
        function [Rmu,Dmu] = RadCurv(obj,mu)
            %RADCURV Function that returns the radius of curvatures of an
            %elliptic modeled earth.
            %   INPUT
            %   -mu: Geodetic Latitude [rad]
            %   OUTPUT
            %   - Rmu: Radius of Curvature of the meridian
            Rmu = obj.SemimajorAxis*( 1-obj.Eccentricity^2 ) / ...
                ( 1-( obj.Eccentricity*sin(mu) )^2 )^(3/2);
            Dmu = obj.SemimajorAxis/...
                sqrt( 1 - ( obj.Eccentricity*sin(mu) )^2 );
        end

        function [Rnu,m] = rhumbLine(obj,lat,lng)
        %RHUMBLINE: function that calculates the path along the rhumb line
        %for the points given
        %   INPUT
        %   -lat: [lat(A),lat(B)]  geodetic latitudes of teh starting and ending
        %       points [rad]
        %   -lng: [lng(A),lng(B)] longitudes of teh starting and ending
        %       points [rad]
        % Definition of Isometric Latitudes
            latc = nan(length(lat),1);
            for i = 1:length(lat)
                latc(i) = obj.IsometricLat(lat(i));
            end
        % Differences
            dlat = latc(2) - latc(1);
            dlng = lng(2) - lng(1);
         % Rhumb path   
            Rnu = atan2(dlng,dlat);

        end

        function lc = IsometricLat(obj,lat)
        %ISOMETRICLAT: function that calculates the isometric latitude for
        %the given ellipsoid.
        %   INPUT
        %   -lat: geodetic latitude [rad]
        %   OUTPUT
        %   -lc: isometric latitude [rad]
        lc = log( tan(pi/4+lat/2)*( ( 1-obj.Eccentricity*sin(lat))/...
            ( 1+obj.Eccentricity*sin(lat) ) )...
            ^(obj.Eccentricity/2)  ); % 60 is used to obtain the result in radiants and not seconds
        end

        function ang = LatLon2Rad(~,ainp,ref,flg)
        %LATLONG2RAD: function that transforms the standard format for
        %latitude and longitude into radiants
        %   INPUT
        %   -ainp: [deg,minute,second]. In can accept more angles if are
        %       given as rown. [ang(1);ang(2);..]
        %   -ref: N-S or E-W
        %   -flg: deg/rad specifies how the angle will be expressed as
        %   output
        if nargin < 4
            flg = 'rad';
        end
        sz = size(ainp); ang = nan(sz(1),1);
        if ischar(ref)
            ref = {ref}; % Converts scalar to cell so thata flg{i} does not throw error
        end
        for i = 1:sz(1)
            % Checks if it is N S E W
            if ref{i} == 'N' || ref{i} == 'E'
                sgn = 1;
            elseif ref{i} == 'S' || ref{i} == 'W'
                sgn = -1;
            else
                warning('Specify North-South or East-West')
                sgn = nan;
            end
            ang(i) = ainp(i,1);
            for j = 2:sz(2)
                ang(i) = ang(i) + ainp(i,j)/(60^(j-1));
            end
            ang(i) = ang(i)*sgn;
        end
        if isequal(flg,'rad')
            ang = ang*pi/180;
        end

        end
        
        function [Vout,M] = NED2DIS(~,Vin,flg,lat,lng)
        %NED2DIS: function that transforms a vector form ECEF to NED system
        %and vice-versa
        %   INPUT
        %   - Vin: vector in teh starting reference frame
        %   - flg: 'N2E' NED to ECEF and 'E2N' ECEF to NED
        %   - lat: [rad] geodetic latitude
        %   - lng: [rad[ longitude
            sl = sin(lat); cl = cos(lat);
            sm = sin(lng); cm = cos(lng);
            
            M = [-sm*cl,-sm*sl,cm; -sl,cl,0; -cm*cl,-cm*sl,-sm];
            if isequal(flg,'E2N')
                M = M';
            end
            
            Vout = M*Vin;
        end
    end
end

