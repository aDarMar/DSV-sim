function dydt = LongDynLin(t,y,A,B,u)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

u = u(:);
dydt = A*y(:) + B*u;

end

