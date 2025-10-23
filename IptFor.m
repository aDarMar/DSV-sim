function [du,u] = IptFor(t)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

global u0

du = zeros(2,1); %[dCL,dT]
%u = zeros(2,1);
if  t < 11
    du(1) = 0.001;
    du(2) = u0(2)*0.0;
end
u = u0 + du; % [CL;T]
%u(2) = u0(2);%*1.01; % T
end

