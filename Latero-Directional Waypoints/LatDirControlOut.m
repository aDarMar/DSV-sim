function [u,u_out,dxdt] = LatDirControlOut(t,x,y_way,x_add,bounds,AC)
%LATDIRCONTROLOUT Summary of this function goes here
%   INPUT
%   - x: [Va.ga,h,m,CL,T,Vd,Psi,Phi,p]
%   - y: [VIAS,M,h,hdot,phi,p,psi,Vg,psiG]
%   - y_way: [VIAS,M,h,hdot,Kv,mu,lng]

%   - x_add: [ID,VIAS,dVd/dt,x(7),Kh] evaluated at previous successful iteration
%   - bounds: [vbound,hbound,V/g] in SI units
%   OUTPUT
%   - u_out: [ID,VIAS,hdotc,Kh,Vd]

    u_out(1) = ZoneIdf(err,bounds);     % Identifies the zone in which the aircraft is                 Flag vector [ slow,fast,Low,High,lowenergy]
    u = 0;                              % Force Vector [Roll]

    Psic = wayRoute();
    u = AC.Klat(:,:,k)*( [0;Psic]*-y(6:7) );

end

function wayRoute()
    % Rhumb Line Tracking
    dR = rhumbLatDist();

end