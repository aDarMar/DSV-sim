function [hed,hdf,dhdf] = deltaHead(tdir,hedfi,Ri)
%DELTAHEAD Summary of this function goes here
%   Detailed explanation goes here
% Vector of Angles swept by turn
    hed = Ri'*[1;0;0];
    cr =[0,1,0]*Ri;
    % Understand thanctual angle referred to North direction
    hed = atan2(cr,hed);
    if tdir > 0
        % Right Turn
        if hedfi < hed
            hdf = 2*pi-hedfi;
            dhdf = 2*pi-hedfi+hed;
            %hds = [hds,linspace( pi,,np )];
        else
            hdf = hedfi;
            dhdf = hedfi-hed;
        end
    else
        % Left Turn
        if hedfi < hed
            hdf = hed;
            dhdf = hedfi-hed;
        else
            hdf = hedfi-2*pi;
            dhdf = -2*pi+hedfi-hed;
        end
    end
end

