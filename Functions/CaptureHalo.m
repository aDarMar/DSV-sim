function [dst,dR,err] = CaptureHalo(x,y_way,bounds,windpx,windpy,GEO)
%CAPTUREHALO Evaluate the ode integration terminator function
%   INPUT
%       - x: state vector
%       - y_way:  waypoint data vector
%       - bounds: bounds defining regions in IAS-h plane
%       - windpx,windpy: function handles to interpolating functions
%       - GEO: Reference ellipsoid class
%   OUTPUT
%       - dst: value of function
%           (IAS/IASb)^2 + (h/hb)^2 + (dR/dRb)^2 - 1
%       - dR: linear distance from teh current position to the waypoint
%       - err: difference between waypoint and curent state
% ----------------------------------------------------------------------- %
%
% Interpolate wind vector at current aircraft position
    % Uses 2D wind field maps (windpx, windpy) and current state x
    Vw = WindMap(windpx,windpy,x);
    % State output vector
    y = CompleteDynNoLin_Out(x(:),Vw);
     % Convert current geodetic coordinates to ECEF position vector
    R = GEO.LatLon2Vec(x(11),x(12),0);
    % IAS M h hdot errors
    err = y_way(1:4) - y(1:4);
    % Linear Distance between aircraft current position and waypoint
    dR = norm(y_way(9:11)-R(1:3),2);       
     % The point must be inside an ellipsoid in the h-IAS-dR space
    dst = ( 20*err(1)/bounds(1) )^2 + ( 15*err(3)/bounds(2) )^2 ...
        + (dR/200)^2 - 1;                     
end

