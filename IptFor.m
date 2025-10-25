function [du,u] = IptFor(t,chs)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

global u0

if nargin == 1
    chs = 0;
end

switch  chs
    case 0
        % Sistema senza controllore
        du = zeros(2,1); %[dCL,dT]
        %u = zeros(2,1);
        if  t >0
            du(1) = 0.1;
            du(2) = u0(2)*0.0;
        end
        u = u0 + du; % [CL;T]
    case 1
        % Sistema con controllo PI
        du = zeros(4,1); %[dCL_p,dCL_i
        if t > 0
            du(1) = 0.1;
        end
        u = [u0+du(1:2);0;0];
    case 2
        du = zeros(4,1); % numero di output y
        if t > 5
            du(4) = 1000/60;
        end
        u = du;
    case 3
        du = zeros(4,1);
        du(4) = 0;
        u = du;
end
%u(2) = u0(2);%*1.01; % T
end

