close all; clear; clc;

addpath("Functions\")

%% Test Building Linear System from AC
AC.m = 15649; AC.Sw = 56.57; AC.CD0 = 0.03; AC.CLmd = 0.2; AC.bw = 24.6;
AC.e = 0.85; AC.ARw = AC.bw^2/AC.Sw;
x0 = [120,0,500];      % Va [m/s] ga [rad] h [m]
%dx = [2.5;0;0]*0;

% Linearized Longitudinal Model
[A,B,C,u0] = LongLinSys(x0,AC);

%% Test Output Functions
scs = [0:0.1:1];dx = x0(:).*scs;%[10;pi/180;10];
y0 = C*x0(:); y_lin = C*dx;
y_nl = LongDynNoLin_Out(x0(:)+dx);
err = abs( 1 - (y0+y_lin)./y_nl ); %err = err(~isnan(err));
if norm(err(~isnan(err)),'inf') > 1e-1
    warning('Linear and Non Linear Output mismatch')
end
figure()
loglog(scs,err)
%% 1
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

%%
clear K1 yout tlf xlf K2
C = zeros(2,4); C(1,1) = 9.45e-4; C(2,2) = 3.47e4;
K = zeros(2,2); K(1,2) = 2.7e-5;
Ai = A - B*K*C; lam = eig(Ai);
K = zeros(2,2); K(1,1) = -1.5;
Ai = Ai - B*K*C; lam = eig(Ai);
