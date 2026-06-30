function [hed,hdf,dhdf] = deltaHead(tdir,hedfi,Ri,Rc)
%DELTAHEAD Summary of this function goes here
%   Detailed explanation goes here
%   INPUT
%   - tdir: flag that specifies turn direction
%   - hedfi: final heading of the turn [rad]
%   - Ri: Vector pointing from center of the circle to starting point
%       in ECEF
%   
% Vector of Angles swept by turn
    hed = atan2(Ri(2),Ri(1));
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

