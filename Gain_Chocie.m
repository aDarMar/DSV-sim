close all; clear; clc

global u0

DEBUG = true;
%% Aircraft Definition
if DEBUG
    % Esempio DC9
    A = zeros(3,3); B = zeros(3,2); C = zeros(4,3); D = zeros(4,2);
    A(1,1) = -7.2e-3; A(1,2) = -32.2; A(2,1) = 2e-4; A(3,2) = 578.47;
    B(1,1) = -2.74; B(2,1) = 1.202e-1; B(1,2) = 2e-4;
    C(2,1) = 9.45e-4; C(4,2) = 60*578.47; % h dot è già in ft/min così
    % NOTA: C(2,1) è sbagliato nel documento (solo l'esponente)
    Ai = [A,B;zeros(2,3),zeros(2,2)]; Bi = [B,zeros(3,2);zeros(2),eye(2)];
    Ci = [C,zeros(4,2)]; Di = [D,zeros(4,2)];
    
    u0 = zeros(2,1);
else
    AC.m = 15649; AC.Sw = 56.57; AC.CD0 = 0.03; AC.CLmd = 0.2; AC.bw = 24.6;
    AC.e = 0.85; AC.ARw = AC.bw^2/AC.Sw;
    x0 = [120,0,500];      % Va [m/s] ga [rad] h [m]
    dx = [2.5;0;0]*0;

    % Linearized Longitudinal Model
    [A,B,C,u0] = LongLinSys(x0,AC);
end
wn = sqrt( - A(2,1)*A(1,2) ); zita = -0.5*A(1,1)/wn;
Tfin = 4/(zita*wn); 
%% Flight Conditions
dx = [0,0,0,0,0];
%% Controllers Design
% Sistema Senza feedback
[tl,xl] = ode45(@(t,x)LongDynLin(t,x,Ai,Bi,Ci,1),[0,Tfin],dx);
xdot = nan(5,length(tl));
for i = 1:length(tl)
    xdot(:,i) = LongDynLin(tl(i),xl(i,:),Ai,Bi,Ci,1);
end
plot(tl,xdot(3,:))
% - CL control of h
Kp = zeros(2,4); Ki = zeros(2,4); Kb = zeros(2,4);
% Gain Set
Kp(1,4) = 2.08e-4; Ki(1,4) = 9e-5;
K1 = [Kp+Kb;Ki]; K2 = [Kb;zeros(2,4)];

Acl = Ai - Bi*K1*Ci; Bcl = Bi*K2;

% Sistema con feedback
% - CL control of h
Tfin = 30;
EvalSys(Tfin,Ai,Bi,Ci,K1,K2,dx);
rLocusDef(Ai,Bi,Ci,4,1); % Kp14
K = zeros(4,4); K(1,4) = 2.6e-5; Ain = Ai - Bi*K*Ci; 
rLocusDef(Ain,Bi,Ci,4,3); % Ki14
K = zeros(4,4); K(3,4) = 9e-5; Ain = Ain - Bi*K*Ci;
% - CL control of M
rLocusDef(Ai,Bi,Ci,4,1); % Kb14 root locus (bisogan dividere per 60 per ottenere il valore del docu)
K = zeros(4,4); K(1,4) = 2.7e-5; Ain = Ai - Bi*K*Ci; 
rLocusDef(Ain,-Bi,Ci,2,1); % Kp12
K = zeros(4,4); K(1,2) = -1.5; Ain = Ain - Bi*K*Ci; eig(Ain)
rLocusDef(Ain,-Bi,Ci,2,3); % Ki12 cioe controllo int su M con CL