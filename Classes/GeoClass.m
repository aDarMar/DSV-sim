classdef GeoClass < referenceEllipsoid 
    %GEOCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        LMercfs
    end
    
    methods
        function obj = GeoClass()
        % GEOCLASS Constructor for the GeoClass object
        %   This constructor initializes a GeoClass instance that inherits
        %   from the referenceEllipsoid superclass with the WGS84 ellipsoid
        %   model. Upon instantiation, it automatically calculates and stores
        %   the Mercator scale factor property (LMercfs) for map projections.
        %   Syntax:
        %       geoObj = GeoClass()
        %
        %   OUTPUT:
        %       - obj: GeoClass object with initialized WGS84 ellipsoid
        %             properties and calculated Mercator scale factor
        % --------------------------------------------------------------- %
            % Initialize as WGS84 reference ellipsoid (inherits from superclass)
            obj = obj@referenceEllipsoid('WGS84');

            % Calculate and store Mercator scale factor for map projection
            % (OLD)
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
        % LATLON2VEC: Converts geodetic coordinates to ECEF position vectors
        %   This function transforms geodetic (latitude, longitude, altitude)
        %   coordinates into Earth-Centered, Earth-Fixed (ECEF) Cartesian
        %   position vectors using MATLAB's Mapping Toolbox geodetic2ecef function.
        %   INPUT
        %   -lat: geodetic latiude [rad]
        %   -lon: longitude [rad]
        %   - h: geodetic altitude [m]
        %   OUTPUT:
        %    - R: 3xN matrix of ECEF position vectors [meters]
        %            Each column represents [X; Y; Z] coordinates for one point
        % --------------------------------------------------------------- %
        % Validate input dimensions to ensure consistent vector lengths
            if length(lat) ~= length(lon) && length(lat) ~= length(h)
                error('Latitude,Longitude and Height vector must have the same number of elements');
            end
            % Determine number of coordinates to process
            pts = length(lat);
            % Preallocate output matrix for efficiency (3 rows: X, Y, Z coordinates)
            R = nan(3,pts);
            [R(1,:),R(2,:),R(3,:)] = geodetic2ecef(obj,lat,lon,h,'radians');
        end

        function [Rnu,m] = rhumbLine(obj,lat,lng)
        % RHUMBLINE: Calculate rhumb line azimuth and distance between two points
        %   This function computes the constant-bearing (rhumb line) navigation
        %   parameters between two geographic points using MATLAB's Navigation
        %   Toolbox. A rhumb line maintains a constant bearing (azimuth) and
        %   crosses all meridians at the same angle.
        %   INPUTS:
        %       - lat: 1x2 vector of geodetic latitudes [radians]
        %               lat(1): Latitude of starting point A
        %               lat(2): Latitude of ending point B
        %       - lng: 1x2 vector of longitudes [radians]
        %               lng(1): Longitude of starting point A  
        %               lng(2): Longitude of ending point B
        %   OUTPUT:
        %       - Rnu: Azimuth angle from pint A to B
        %       - m: distange laong the rhumb line between points A and B
        % --------------------------------------------------------------- %
            Rnu = azimuth("rh",lat(1),lng(1),lat(2),lng(2),obj,'radians');
            % distance gives a linear distance in the units of the semimajor axis of the ellipsoid
            m = distance("rh",lat(1),lng(1),lat(2),lng(2),obj,'radians')*obj.SemimajorAxis;
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
        % NED2DIS: Transforms vectors between ECEF and NED coordinate systems
        %   This function converts vectors between Earth-Centered, Earth-Fixed (ECEF)
        %   and North-East-Down (NED) local tangent plane coordinate systems.
        %   The transformation accounts for geographic position (latitude/longitude)
        %   to define the local NED reference frame orientation.
        %   INPUTS:
        %       - Vin: 3xN matrix of vectors in the starting reference frame
        %             Columns represent individual vectors [X; Y; Z] or [N;
        %             E; D] and are given in [m]
        %       - fl: Transformation direction flag:
        %             'E2N': Transform from ECEF to NED coordinates
        %             'N2E': Transform from NED to ECEF coordinates
        %       - lat: Geodetic latitude(s) [rad] defining NED frame origin(s)
        %             Scalar: Single latitude for all vectors
        %             Vector: Multiple latitudes for multiple vectors
        %       - lng: Longitude(s) [rad] defining NED frame origin(s)
        %             Scalar: Single longitude for all vectors
        %             Vector: Multiple longitudes for multiple vectors
        %   OUTPUTS:
        %     Vout  - 3xN matrix of vectors in the target reference frame
        %     M     - Transformation matrix used for the conversion
        % Tested with ecef2ned matlab function
        % --------------------------------------------------------------- %
            % If Latitude and Longitude are given as vectors
            nl = length(lat); nm = length(lng); nv = length(Vin(1,:));
            % Ensure latitude and longitude vectors have matching lengths
            if nl > 1 && (nl ~= nm)
                error('Number of Latitudes and Longitudes given is not equal')
            end
            % Prevent ambiguous case: multiple vectors with multiple locations
            if nv > 1 && nl>1
                error('You can pass either multiple lat-long pairs with one vector, or one lat-long pair with multiple vectors. Not both.')
            end
            % Validate transformation flag
            if ~isequal(flg,'N2E') && ~isequal(flg,'E2N')
                error('Flaf must be E2N or N2E')
            end
    
            if nv == 1
                % CASE 1: Single Vector, Multiple Locations
                Vout = nan(3,nl);
                for j = 1:nl
                    % Compute trigonometric values for current position
                    sm = sin(lat(j)); cm = cos(lat(j));
                    sl = sin(lng(j)); cl = cos(lng(j));
                    % Define ECEF to NED transformation matrix
                    % This matrix rotates from ECEF to local NED frame
                    M = [-sm*cl,-sm*sl,cm; -sl,cl,0; -cm*cl,-cm*sl,-sm];% ECEF2NED
                    % For NED to ECEF transformation, use transpose
                    if isequal(flg,'N2E')
                        M = M';
                    end
                    % Apply transformation to input vector
                    Vout(:,j) = M*Vin(:);
                end
            else
                % CASE 2: Multiple Vectors, Single Location
                Vout = nan(3,nv);
                % Compute trigonometric values for the single position
                sm = sin(lat); cm = cos(lat);
                sl = sin(lng); cl = cos(lng);
                % Define ECEF to NED transformation matrix
                M = [-sm*cl,-sm*sl,cm; -sl,cl,0; -cm*cl,-cm*sl,-sm];% ECEF2NED
                if isequal(flg,'N2E')
                    M = M';
                end
                % Process each vector using the same transformation matrix
                for j = 1:nv
                    Vout(:,j) = M*Vin(:,j);
                end
            end
        end

    end
end

