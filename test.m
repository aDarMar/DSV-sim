close all; clear; clc;

A = zeros(4,4); B = zeros(4,2); C = zeros(1,4);
A(1,1) = -7.2e-3; A(1,2) = -32.2; A(1,4) = -2.74;
A(2,1) = 2e-4; A(2,4) = 1.202e-1;
A(3,2) = 578.47;

B(1,1) = -2.74;
B(2,1) = 1.202e-1;
B(4,2) = 1;

C(1,2) = 578.47;

K1 = [2.08e-4;9e-5]; K2 = [0;0];

Tfin = 30; x0 = zeros(4,1);


[tlf,xlf] = ode45(@(t,x)TestEq(t,x,A,B,C,K1,K2),[0,Tfin],x0);

yout = C*xlf';

plot(tlf,yout*60,'k',tlf,tlf*0+1000,'--r')