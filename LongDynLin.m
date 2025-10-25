function dydt = LongDynLin(t,y,A,B,C,chs,K1,K2)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin == 5
    chs = 0;
end
[u,~] = IptFor(t,chs);
if chs == 3
    if t > 5
        u =  K1*([0;0;0;1000/60] - C*y(:) ) - K2*C*y(:);
    else
        u = zeros(4,1);
    end
end
dydt = A*y(:) + B*u;


end

