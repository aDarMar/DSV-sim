function dydt = LongDynLin(t,y,A,B,C)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

[u,~] = IptFor(t);

dydt = A*y(:) + B*u;


end

