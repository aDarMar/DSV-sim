close all; clear; clc

global u0
addpath('.\Classes')
addpath('.\Functions')
addpath('.\Data')

main();

function main()
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
    for it = 1:length(x0(:,1))
    
        [A,B,C,D] = AC.LongLinSys(x0(it,1:3),x0(it,4) );
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
    
    %% Gain Definitions for Reference Condition
    
    lam_c = nan(5,3); % Roots of controlled system in ref cond
    fig_rl = figure('Name','Eigenvalues out of reference condition');
    wn = [2;0.05;0.4]; zita = [0.9;0.7;0.7]; p = [0,nan;-0.05,nan;-0.1,-0.2]; 
    CTR = {'CLcHd','CLcV','CLTcVh'}; 
    TIT = {'CL control of hdot','CL control of V','CL and T control of V,hdot'};
    Kp = zeros(2,4); Kb = Kp; Ki = Kp;
    nctrnd = length(wn);
    for iTs = 1:nctrnd
        axs(iTs) = subplot(1,3,iTs,'Parent',fig_rl); hold(axs(iTs),'on');
        if CTR{iTs} == "CLTcVh"
            [Kpt,Kit,Kbt,lam] = ...
                Gains( wn(iTs),zita(iTs),Ai,Bi,Ci,D,CTR{iTs},p(iTs,1:2) );
        else
            [Kpt,Kit,Kbt,lam] = ...
                Gains( wn(iTs),zita(iTs),Ai,Bi,Ci,D,CTR{iTs},p(iTs) );
        end
        lam_c (:,iTs) = lam; [temp1,temp2] = OmZitaCalc(lam_c(:,iTs));
        plot_conds( x0,xref,Kbt,Kit,Kpt,axs(iTs),TIT{iTs} );
        Kp = Kp + Kpt; Ki = Ki + Kit; Kb = Kb + Kbt;
    end

    function [lams,fig,ax,lin] = plot_conds(xtest,xref,Kb,Ki,Kp,ax,tit)

        lams = AC.CheckCont(xtest,Kp,Ki,Kb);
        lamr = AC.CheckCont(xref,Kp,Ki,Kb);
        if nargin < 6
            fig = figure();
            ax = axes('Parent',fig); hold(ax,'on');
            tit = 'Roots at different flight conditions';
        end
        plot(ax,real(lams),imag(lams),'o','MarkerSize', 5, ...          % dimensione del marker
            'MarkerEdgeColor', [0 0 0], ...% colore bordo (nero)
            'MarkerFaceColor', [0.5 0 0], ...% colore interno (rosso)
            'LineWidth', 0.5);
        plot(ax,real(lamr),imag(lamr),'diamond','MarkerSize', 8, ...          % dimensione del marker
            'MarkerEdgeColor', [0 0 0], ...% colore bordo (nero)
            'MarkerFaceColor', [0 0.5 0], ...% colore interno (rosso)
            'LineWidth', 0.5);
        grid(ax,'minor');
        title(ax,tit);
    end

end

function [wn,zita] = OmZitaCalc(lam,A)
lam = lam(abs(lam)~=0); nlam = length(lam);
wn = nan(nlam,1); zita = wn;
for ic = 1:nlam
    if imag(lam(ic)) ~=0 % Check if underdamped
        zita(ic) = imag(lam(ic))/real(lam(ic));
        zita(ic) = 1/sqrt( 1 + zita(ic)^2 );
        wn(ic) = -real(lam(ic))/zita(ic);
    else
        % TODO overdamped and critically damped
        disp('a')
    end
end
end