function dxdt = TestEq(t,x,A,B,C,K1,K2)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
 yref = 0;
 u = zeros(2,1);
    if t > 5
        yref = 1000/60;
        e = yref(:) - C*x(:);
        u = K1*60*e -K2*C*x(:);
    end
    dxdt = A*x(:) + B*u(:);

end