function [dst,dR,err] = CaptureHalo(x,y_way,bounds,windpx,windpy,GEO)
%CAPTUREHALO Summary of this function goes here
%   Detailed explanation goes here
    Vw = WindMap(windpx,windpy,x);
    y = CompleteDynNoLin_Out(x(:),Vw);
    R = GEO.LatLon2Vec(x(11),x(12),0);
    err = y_way(1:4) - y(1:4);
    dR = norm(y_way(9:11)-R(1:3),2);             % Linear Distance between aircraft curr pos and
    dst = ( 20*err(1)/bounds(1) )^2 + ( 15*err(3)/bounds(2) )^2 ...
        + (dR/200)^2 - 1;                      % The point must be inside an ellipse in the h-IAS plane
end

