function [du,u] = IptFor(t)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

global u0

du = zeros(2,1);
%u = zeros(2,1);
if t == 0
    du(1) = 0.00;
end
u = u0(:) + du;
end

