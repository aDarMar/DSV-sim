close all; clear; clc

global u0

%% Linear vs NonLinear Model Comparison

y0 = [1,0];
Tfin = 120;
K = 1;
m = 0.5;

[t,y] = ode45(@(t,y) mms_T(t,y,K,m),[0,Tfin],y0); % Crea una funzione anonima che chiama al funzione mms_Ts

plot(t,y(:,1),'k',t,cos(t*sqrt(K/m))*y0(1),'r')

%% 

AC.m = 15649; AC.Sw = 56.57; AC.CD0 = 0.03; AC.CLmd = 0.2; AC.bw = 24.6;
AC.e = 0.85; AC.ARw = AC.bw^2/AC.Sw;
x0 = [120,0,500];      % Va [m/s] ga [rad] h [m]
dx = [2.5;0;0]*0;

% Linearized Longitudinal Model
[A,B,C,u0] = LongLinSys(x0,AC);
wn = sqrt( - A(2,1)*A(1,2) ); zita = -0.5*A(1,1)/wn;
Tfin = 4/(zita*wn); 
[tl,xl] = ode45(@(t,x)LongDynLin(t,x,A,B,C),[0,Tfin],dx);

% NonLinear Longitudianl Model
x0 = x0(:) + dx(:);
options = odeset('AbsTol',1e-5,'RelTol',1e-7); 
[tnl,xnl] = ode45(@(t,x)LonDynNoLin(t,x,AC),[0,Tfin],x0,options);
TIT = {'dV vs t','dg vs t','dh vs t'};
figure()
for i = 1:3
    subplot(3,1,i); plot(tl,x0(i)+xl(:,i)-dx(i),'-k',tnl,xnl(:,i),'--r'); title(TIT{i})
end
hold on;
[T, a, P, rho] = atmosisa(xnl(:,3));
figure()


for i=1:length(tnl)
    [~,out(1:2,i)] = IptFor(tnl);
    Cd(i) = AC.CD0 + 1/(AC.e*pi*AC.bw^2/AC.Sw)*(out(1,i)-AC.CLmd)^2 ;
    q(i) = 0.5*rho(i)*xnl(i,1)^2;
    D(i) = q(i)*( AC.CD0+1/(AC.e*pi*AC.bw^2/AC.Sw)*(out(1,i)-AC.CLmd)^2 )*AC.Sw;
end
subplot(2,1,1); plot( tnl,q(:)./(0.5*rho(1).*x0(1).^2.) )
subplot(2,1,2); plot(...%tnl,0.5*rho.*xnl(:,1).^2.*out(1,:),...%
    tnl,out(2,:),...
    tnl,D,'--k',tnl,q,tnl,Cd );

%subplot(2,1,2); plot(tl,AC.m*9.81*xl(:,3) +  AC.m*0.5*xl(:,1) );
% x0 = x0 + xl(end,:)'; 
% [tnl,xnl] = ode45(@(t,x)LonDynNoLin(t,x,AC),[Tfin,2*Tfin],x0,options);
% for i = 1:3
%     subplot(3,1,i); plot(tnl,xnl(:,i),'-*r'); title(TIT{i})
% end
%subplot(3,1,2); plot(t,xl(:,2)); title()
%subplot(3,1,3); plot(t,xl(:,3)); title()

%% Controllers
% Debug B767 30000ft
A = zeros(3,3); B =zeros(3,2); C = zeros(4,3);
A(1,1) = -0.0889; A(1,2) = -32.2; A(2,1) = 3.66e-4; A (3,2) = 787;
B(1,1) = -21.7; B(1,2) = 1.63e-4; B(2,1) = 1.73e-1;
C(1,1) = 0.593; C(2,1) = 1e-3; C(4,2) = 4.72e4;

Kps = zeros(2,4); Kis = zeros(2,4);
K = [Kps;Kis];
Ai = [A,B;zeros(2,3),zeros(2,2)]; Bi = [B,zeros(3,2);zeros(2,2),eye(2,2)];
Ci = [C,zeros(4,2)];

Acl = Ai - Bi*K*Ci;

nKs = 11; Ks = linspace(-1,1,nKs);

i = 1;
K(1,1) = Ks(i);
Acl = Ai - Bi*K*Ci;
eig(Acl)

