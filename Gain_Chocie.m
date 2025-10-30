close all; clear; clc

global u0
addpath('.\Classes')
addpath('.\Functions')
addpath('.\Data')

DEBUG = false;
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
    
else
    nmfl = "Q100.aero"; AC = ACclass(nmfl);
    %AC.m = 15649; AC.Sw = 56.57; AC.CD0 = 0.03; AC.CLmd = 0.2; AC.bw = 24.6;
    %AC.e = 0.85; AC.ARw = AC.bw^2/AC.Sw;
    x0 = [120,0,500];      % Va [m/s] ga [rad] h [m]
    dx = [2.5;0;0]*0;

    % Linearized Longitudinal Model
    % [A,B,C,u0] = LongLinSys(x0,AC);
end

%% Possible Flight Conditions

[T, a, P, rho] = atmosisa([0,4500,2500,7650]);
rom = nan(1,2); rom(1) = AC.MZF/rho(1); rom(2) = AC.MTO/rho(2); rom(3) = AC.MTO/rho(3); rom(4) = AC.MTO/rho(4);                            % Min and Max m/rho [m^3]
CL = linspace(0.05,1.5,100); V = sqrt( rom*2*9.81./(AC.Sw*CL(:) ) );

fig = figure();
ax = axes('Parent',fig);
plot( ax,CL,V ); hold(ax,'on');

hmax = 7625; [T, a, P, rho] = atmosisa(hmax);
Vmax = 150; Vstall = 60; Mlim = 0.5; qmax = 0.5*rho*Vmax^2;

%% Reference Condition
[xref,x0] = Flight_Env([Vmax,Vstall,Mlim,qmax,hmax],AC,ax);

%% Response

fig = figure();
ax2 = axes('Parent',fig); hold(ax2,'on');
for i = 1:length(x0(:,1))
   
    [A,B,C,D] = AC.LongLinSys(x0(i,1:3),x0(i,4) );
    lam = eig(A);
    plot(ax2,real(lam),imag(lam),'ob');
    
end
[A,B,C,D] = AC.LongLinSys(xref(1:3),xref(4) );
lam = eig(A);
plot(ax2,real(lam),imag(lam),'o','MarkerSize', 8, ...          % dimensione del marker
        'MarkerEdgeColor', [0 0 0], ...% colore bordo (nero)
        'MarkerFaceColor', [1 0 0], ...% colore interno (rosso)
        'LineWidth', 1); 
% System with Integral Control
Ai = [A,B;zeros(2,5)]; Bi = [B,zeros(3,2);zeros(2,2),eye(2)];
Ci = [C,zeros(4,2)];
% % vls = lhsdesign(1500,3); 
% % ms = vls(:,1)*(AC.MTO - AC.MZF) + AC.MZF; 
% % hs = vls(:,2)*(hmax); % h from S/L to 25000ft
% % Vs = vls(:,3)*(Vmax - Vstall) + Vstall; % VTAS from VStall to Vmax@h max
% % [~, ~, ~, rhos] = atmosisa(hs); morho = ms./rhos;
% % 
% % CLs = morho*2*9.81./(AC.Sw*Vs.^2);
% % 
% % plot( ax,CLs,Vs,'or')
% % idx = 1:length(Vs);
% % j = 1; itemp = nan(1,length(Vs));
% % for i = 1:length(Vs)
% %     [T, a, P, rho] = atmosisa(hs(i));
% %     flg = Vs(i)/a < Mlim;               % Must be below Max Mach
% %     flg = 0.5*rho*Vs(i)^2 < qmax && flg;    % Must be below max q
% %     flg = CLs(i) < AC.CLmax( Vs(i)/a,AC.ReCalc(hs(i),Vs(i)/a) ) && flg; % CL < CLmax
% %     T = AC.Thrust_Law( 1,hs(i),'ipt' );
% %     flg = AC.polar( Vs(i)/a,AC.ReCalc(hs(i),Vs(i)/a),CLs(i) ) ...
% %         < T/(0.5*rho*Vs(i)^2*AC.Sw) && flg; % Treq < Tmax
% %     if flg
% %        itemp(j) = i;
% %        j = j + 1;
% %     end
% % end
% % itemp = itemp(~isnan(itemp));
%plot( ax,x0(:,5),x0(:,1),'*b')


%x0 = Vs(itemp); x0 = x0(:); x0 = [x0,itemp(:)*0,hs(itemp),ms(itemp)];



%% Gain Choice of Universal Flight Condition

% Finding Reference Condition
%Ms = 0:0.5; h




% Response 

% fig = figure();
% ax2 = axes('Parent',fig); hold(ax2,'on');
% for i = 1:length(x0(:,1))
%    
%     [A,B,C,D] = AC.LongLinSys(x0(i,1:3),x0(i,4) );
%     lam = eig(A);
%     plot(ax2,real(lam),imag(lam),'ob');
%     
% end

%% Gain Definitions for Reference Condition

wn = 2; zita = 0.9;
[Kp,Ki,Kb] = Gains(wn,zita,Ai,Bi,Ci,D);
% K = zeros(4,4); K(1,4) = 2.6e-5; Ain = Ai - Bi*K*Ci;
% rLocusDef(Ain,Bi,Ci,4,3); % Ki14

% CL control of hdot

wns = wn^2; zwn = 2*zita*wn;
GN = [ B(2,1)*C(4,2),0,1; ...
    C(4,2)*( A(2,1)*B(1,1) - A(1,1)*B(2,1) ), 0 , zwn; ...
    0, C(4,2)*( A(2,1)*B(1,1) - A(1,1)*B(2,1) ), wns ];
RHS = [ A(1,1)+zwn;wns + A(1,2)*A(2,1); 0];

kcl = GN\RHS; Kp(1,4) = kcl(1); Ki(1,4) = kcl(2);

Ai = [A,B;zeros(2,5)]; Bi = [B,zeros(3,2);zeros(2,2),eye(2)];
Ci = [C,zeros(4,2)]; K = [Kp+Kb;Ki];

rLocusDef(Ai,Bi,Ci,4,1); % Kp14
Kit = zeros(4,4); Kit(1,4) = kcl(1); Ain = Ai - Bi*Kit*Ci;
rLocusDef(Ain,Bi,Ci,4,3); 
Kit = zeros(4,4); Kit(3,4) = kcl(2); Ain = Ain - Bi*Kit*Ci;% Ki14
lam = eig(Ain);
%plot(real(lam),imag(lam),'ob');
    
fig(2) = figure();
j = 2; ax2(j) = axes('Parent',fig(2)); hold(ax2(j),'on');

for i = 1:length(itemp)
   
    [A,B,C,D] = AC.LongLinSys(x0(i,1:3),x0(i,4) );
    Ai = [A,B;zeros(2,5)]; Bi = [B,zeros(3,2);zeros(2,2),eye(2)];
    Ci = [C,zeros(4,2)];
    Acl = Ai - Bi*K*Ci;
    
    lam = eig(Acl);
    plot(ax2(j),real(lam),imag(lam),'ob');
    
end

