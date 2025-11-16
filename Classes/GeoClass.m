classdef GeoClass < referenceEllipsoid 
    %GEOCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %a
        %b
        %e
        %f
        LMercfs
    end
    
    methods
        function obj = GeoClass()
            %GEOCLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@referenceEllipsoid('WGS84');
            obj.LMercfs = obj.CalcLMer();
        end
        
        function Cfs = CalcLMer(obj)
            %CALCMER: function that calculates the coefficients for the
            %meridian arch length method.
            e = obj.Eccentricity;
            Cfs = nan(6,1);
            Cfs(1) = 1 + 0.75*e^2 +45/64*e^4+175/256*e^6+...
                11025/16384*e^8+43659/65536*e^10;
            Cfs(2) = 0.75*e^2+15/16*e^4+525/512*e^6+...
                2205/2048*e^8+72975/65536*e^10;
            Cfs(3) = 15/64*e^4 + 315/256*e^6+...
                2205/4096*e^8 + 10395/16384*e^10;
            Cfs(4) = 35/512*e^6+315/3048*e^8+...
                31185/131072*e^10;
            Cfs(5) = 315/16384*e^8+3465/65536*e^10;
            Cfs(6) = 693/131072*e^10;
        end
        
        function R = LatLon2Vec(obj,lat,lon,h)
        %LATLON2VEC: function that gives the position vector in ECEF
        %coordinates given geodetic latitude, longitude and geodetic
        %altitude using built in Matlab mapping toolbox
        %   INPUT
        %   -lat: geodetic latiude [rad]
        %   -lon: longitude [rad]
        %   - h: geodetic altitude [m]
        if length(lat) ~= length(lon) && length(lat) ~= length(h)
            error('Latitude,Longitude and Height vector must have the same number of elements');
        end
            pts = length(lat);
            R = nan(3,pts);
            [R(1,:),R(2,:),R(3,:)] = geodetic2ecef(obj,lat,lon,h,'radians');

        end

        function [Rnu,m] = rhumbLine(obj,lat,lng)
        %RHUMBLINE: function that calculates the path along the rhumb line
        %for the points given. It uses MATLAB navigation toolbox
        %   INPUT
        %   -lat: [lat(A),lat(B)]  geodetic latitudes of teh starting and ending
        %       points [rad]
        %   -lng: [lng(A),lng(B)] longitudes of teh starting and ending
        %       points [rad]            
            Rnu = azimuth("rh",lat(1),lng(1),lat(2),lng(2),obj,'radians');
            m = distance("rh",lat(1),lng(1),lat(2),lng(2),obj,'radians');
        end

        %% 
        function R = LatLon2VecOLD(obj,lat,lon,h)
        %LATLON2VEC: function that gives the position vector in ECEF
        %coordinates given geodetic latitude, longitude and geodetic
        %altitude
        %   INPUT
        %   -lat: geodetic latiude [rad]
        %   -lon: longitude [rad]
        %   - h: geodetic altitude [m]
        % Tested with geodetic2ecef matlab class
            [~,Dmu] = obj.RadCurv(lat);
            R = [(Dmu+h)*cos(lat)*cos(lon);...
                (Dmu+h)*cos(lat)*sin(lon);...
                (Dmu*(1-obj.Eccentricity^2)+h)*sin(lat)];

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

        function [Rnu,m] = rhumbLineOLD(obj,lat,lng)
        %RHUMBLINE: function that calculates the path along the rhumb line
        %for the points given
        %   INPUT
        %   -lat: [lat(A),lat(B)]  geodetic latitudes of teh starting and ending
        %       points [rad]
        %   -lng: [lng(A),lng(B)] longitudes of teh starting and ending
        %       points [rad]
        % Tested with Table 6.3 and matlab functions 

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
         % Distance
            Ms = obj.ArchDist(lat);
            m = (Ms(2) - Ms(1))/cos(Rnu);
        end
        
        function M = ArchDist(obj,lat)
            nlat = length(lat); v2 = nan(1,6);
            M = nan(nlat,1); 
            em1 = obj.SemimajorAxis*(1-obj.Eccentricity^2);
             
            for j = 1:nlat
                sg = -1;
                v2(1) = em1*lat(j);
                for i = 1:5
                    v2(i+1) = sg*em1*sin(2*i*lat(j))/(2*i);
                    sg = -sg;
                end
                M(j) = v2*obj.LMercfs(:);
            end
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
       
        %% 
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
        % Tested with ecef2ned matlab function
            % If Latitude and Longitude are given as vectors 
            nl = length(lat); nm = length(lng); nv = length(Vin(1,:));
            if nl > 1 && (nl ~= nm) 
                error('Number of Latitudes and Longitudes given is not equal')
            end
            if nv > 1 && nl>1
                error('You can pass either multiple lat-long pairs with one vector, or one lat-long pair with multiple vectors. Not both.')
            end
            if ~isequal(flg,'N2E') && ~isequal(flg,'E2N')
                error('Flaf must be E2N or N2E')
            end
            if nv == 1
                % One vector multiple lat-lon
                Vout = nan(3,nl);
                for j = 1:nl
                    sm = sin(lat(j)); cm = cos(lat(j));
                    sl = sin(lng(j)); cl = cos(lng(j));
                    M = [-sm*cl,-sm*sl,cm; -sl,cl,0; -cm*cl,-cm*sl,-sm];% ECEF2NED
                    if isequal(flg,'N2E')
                        M = M';
                    end
                    Vout(:,j) = M*Vin(:);
                end
            else
                % Multiple vectors, one lat/lon
                Vout = nan(3,nv);
                sm = sin(lat); cm = cos(lat);
                    sl = sin(lng); cl = cos(lng);
                    M = [-sm*cl,-sm*sl,cm; -sl,cl,0; -cm*cl,-cm*sl,-sm];% ECEF2NED
                    if isequal(flg,'N2E')
                        M = M';
                    end
                for j = 1:nv
                    Vout(:,j) = M*Vin(:,j);
                end



            end


            
            
                
    
                

        end

    
    end
end

