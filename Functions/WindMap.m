function Vw = WindMap(windpx,windpy,x)
%WINDMAP Function that gives the current wind speed based on a latitude
% longitude and height table
%   INPUT
%   - x: extended state vector [Va.ga,h,m,CL,T,Vd,Psi,Phi,p,mu,lng]
%   OUTPUT
%   - Vw: Wind Speed in local NED coordinates
% ----------------------------------------------------------------------- %
%tic
    %load( [wind_name,'.mat'],'windpx','windpy' );
    %toc
    nxs = length(x(:,1));
    Vw = zeros( nxs,2 );
    if isscalar( x(1,:) ) % During the integration x is a column vector
        Vw(1) = windpx( x(11),x(12),x(3) );
            Vw(2) = windpy( x(11),x(12),x(3) );
    else
        %tic
        for i = 1:length(x(:,1))
            Vw(i,1) = windpx( x(i,11),x(i,12),x(i,3) );
            Vw(i,2) = windpy( x(i,11),x(i,12),x(i,3) )*10;
        end
    end
    %toc
end
