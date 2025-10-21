function dydt = mms_T(t,y,k,m)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

dydt = [ y(2); -k/m*y(1)];
end

