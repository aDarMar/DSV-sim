function [outputArg1,outputArg2] = DynEqs6DoF(inputArg1,inputArg2)
%DYNEQS6DOF Summary of this function goes here
%   Detailed explanation goes here
% Right-hand-sides of the ODE system;
% - rows 1-6 are the equations of traslational and rotational motion of a rigid
body;
%% - rows 7-12 are the navigation and gimbal equations, to calculate CG coordinates
and Euler angles in the Earth frame (ECEF)


omegatilde = @(x) [ 0 -x(6), x(5); x(6), 0 , -x(4); -x(5), x(4) , 0 ];
zero = zeros(3);

dxdt = [ omegatilde(x), zero, zero, zero; ...
        zero, obj.I\(omegatilde(state)*obj.I), zero, zero; ...
        T_BE(state), zero, zero, zero; ...
        zero, T_gimb(state), zero, zero] * x(:) + ...
[g/W*X(t,state); g/W*Y(t,state); g/W*Z(t,state);...
(I)\[L_roll(t,state);M_pitch(t,state);N_yaw(t,state)]; zeros(6,1)];
end

