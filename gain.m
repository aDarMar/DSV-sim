close all; clear; clc

global u0

% Linear vs NonLinear Model Comparison

y0 = [1,0];
Tfin = 120;
K = 1;
m = 0.5;

[t,y] = ode45(@(t,y) mms_T(t,y,K,m),[0,Tfin],y0); % Crea una funzione anonima che chiama al funzione mms_Ts

plot(t,y(:,1),'k',t,cos(t*sqrt(K/m))*y0(1),'r')

%% 

AC.m = 15649; AC.Sw = 56.57; AC.CD0 = 0.03; AC.CLmd = 0.2; AC.bw = 24.6;
AC.e = 0.85; AC.ARw = AC.bw^2/AC.Sw;
x0 = [130,0,50];      % Va [m/s] ga [rad] h [m]
% Linearized Longitudinal Model
[A,B,C,u0] = LongLinSys(x0,AC);
wn = sqrt( - A(2,1)*A(1,2) ); zita = -0.5*A(1,1)/wn;
Tfin = 40/(zita*wn); 
[tl,xl] = ode45(@(t,x)LongDynLin(t,x,A,B,C),[0,Tfin],[0,0,0]);
% NonLinear Longitudianl Model
options = odeset('RelTol',1e-8); 
[tnl,xnl] = ode45(@(t,x)LonDynNoLin(t,x,AC),[0,Tfin],x0,options);
TIT = {'dV vs t','dg vs t','dh vs t'};
for i = 1:3
    subplot(3,1,i); plot(tl,x0(i)+xl(:,i),'-k',tnl,xnl(:,i),'--r'); title(TIT{i})
end
%subplot(3,1,2); plot(t,xl(:,2)); title()
%subplot(3,1,3); plot(t,xl(:,3)); title()