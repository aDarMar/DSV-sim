close all; clear; clc;

addpath('Data')
addpath('Classes')
addpath('Functions')

nmfl = "Q100_comp.aero"; Aero = ACclass(nmfl);
main(Aero)

function main(Aero)
    %% Reference Condition
    %   Find an equilibrium condition
    % Longitudinal
    V0 = 130;           % Desired Speed
    gam0 = -0.0/180*pi;           % Desired flight path angle
    almin = 10*pi/180;  % Maximum alpha
    
    x0 = zeros(13,1);
    x0(13) = 15000; x0(5) = 0;  % mass and q0
    x0(9) = -3000;              % h
    
    lb = [V0*cos(almin),V0*sin(-almin),0,-15];  % Lower Bounds
    ub = [V0,V0*sin(almin),1,15];               % Upper Bounds
    
    xvar0 = [V0*cos(-1),V0*sin(-1),0.5,2.5];  % u,w,deltaT,deltaE
    
    eqV = fmincon(@zeroFun,xvar0,[],[],[],[],lb,ub,@nlConst);
    
    function [c,ceq] = nlConst(xvar)
    %NLCONST: nonlinear constraint for the zero finding algorithm. We
    % assign w,u only to calculate alpha, while the total speed is fixed
    %   INPUT
    %   - xvar: [u,w,deltaT,deltaE]
    % ------------------------------------------------------------------ %
        ceq = [];   % u,w must satisfy teh V0 constraint
        c = abs( norm(xvar(1:2),2) - V0 ) - 30 ;
    end
    
    function f = zeroFun(xvar)
    %ZEROFUN: RHS of the complete 6DoF equation, used to find a
    % longitudinal trim condition. The state vector is:
    % [u,v,w,p,q,r,x,y,z,psi,teta,phi]
    %   INPUT
    %   - xvar: [u,w,deltaT,deltaE]
    %   OUTPUT
    %   - f = fu^2 + fw^2 + fq^2 + fz^2 : objective function.
    % ------------------------------------------------------------------ %
        x = x0;
        x(1) = xvar(1); x(3) = xvar(2);
        x(11) = gam0 + atan(xvar(2)/xvar(1));       % Teta = gamma + alphaB
        u0(1:2) = xvar(3:4); 
        dx = Aero.DynEqs6DoF(x,u0);                 % [u,v,w,p,q,r,x,y,z,psi,teta,phi] but v,p,r,y,psi,phi = 0 already
        f = norm(dx([1,3,5,9]),2)^2;                % du + dw + dq + dz = 0

    end
   % Building the State Vector in teh Trim Condition
   alpha0 = atan(eqV(2)/eqV(1));           
    x0([1,3]) = eqV(1:2); u0(1:2) = eqV(3:4);       % Assign u,w
    x0(11) = gam0 + alpha0;                         % Teta = gamma + alphaB
    % Checks if the minimum for f is actually a trim condition
    CHC = true;
    if CHC
        dx = Aero.DynEqs6DoF(x0,u0);
        f = norm(dx([1,3,5,9]),2)^2;
        if abs(f) > 1e-6
            error('Minimum is not zero for f, trim not found')
        end
    end

    %% Linearization
    [Alng,Blng,MTlong,Altd,Bltd,x0s] = Aero.LinSysComp(x0,u0);
    ik = 5; MTlong = eye(6);
    dx0s = zeros(13,1);
    [~,Mod] = freeResp( Alng,dx0s,'lon',MTlong,[] );
    dx0s([1,3,5,7,9,11]) = 100*real( Mod(ik).mode ); %x0s(1)*0.05;                                   % Perturbation in Stability Axes
    t0 = 0; tfin = 180;
    dx0 = dx0s;                                             % Perturbations in Body Axes
    dx0(1:3) = Aero.Wind2Body(dx0s(1:3),'W2B',alpha0,0);  
    dx0(4:6) = Aero.Wind2Body(dx0s(4:6),'W2B',alpha0,0);
    % Non-Linear Simulation
    Dyn6DoF =@(t,x) Aero.DynEqs6DoF(x,u0);
    [tvec,xnl] = ode113(Dyn6DoF,[t0,tfin],x0+dx0);
    % Linear Simulation
    [xl,Mod] = freeResp(Alng,dx0s,'lon',MTlong,[],tvec);
    xl = x0s' + real(xl);                % xl should be real but some very small imaginary parts are possible (1e-14)
    % Convert Results back in Body axes
    temp = xl';
    temp(1:3,:) = Aero.Wind2Body(temp(1:3,:),'W2B',alpha0,0);
    temp(4:6,:) = Aero.Wind2Body(temp(4:6,:),'W2B',alpha0,0);
    temp(11,:) = temp(11,:) + alpha0; temp = temp';
    RefV = [1/x0s(1),1/x0s(1),x0s(5)*Aero.cref*0.5/x0s(1),ones(1,3)];
    err = abs( xnl - temp );
    for iE = 1:6
        iR = [1,3,5,7,9,11];
        err(:,iR(iE)) = err(:,iR(iE))*RefV(iE)*100; %err(err<10) = nan;
    end
    % Nonlinear Forces
    [F,FA,FT,FW] = Aero.Forces(xnl',u0,tvec);
    GRF = false;
    if GRF
        plotv = [tvec,xnl,temp,F',FA',FT',FW'];
        figS = [ ones(13,2), (2:14)', (115:127)' ]; % [figID,xidx,yidx1,yidx2]

        Aux = [ 2*ones(6,1),ones(6,1),(28:33)',(134:139)',(140:145)',(146:151)' ];
        figS = [ figS,nan( length(figS(:,1)), length(Aux(1,:)) - length(figS(1,:)) ) ; Aux  ];
        %figS(4:7,4) = 114:117;% u,w,q,teta
        IMTIT = {'State Variables','Forces in Body'};
        TIT = {'u','v','w','p','q','r','x_{CG}','y_{CG}','z_{CG}','\psi',...
            '\theta','\phi','m','F_x','F_y','F_z','M_z','M_y','M_z'};
        LG = repmat({'-'},size( figS(:,3:end)) );
        for iG = 1:6
            LG{14+iG,1} = 'F_{tot}'; LG{14+iG,2} = 'F_{A}'; LG{14+iG,3} = 'F_{T}';
            LG{14+iG,4} = 'F_{W}';
        end
        PlotPerImag( plotv,3,figS,IMTIT,LG,TIT )
    end
    %% Root Locus
    % dCLda,dCMda,dCMdad,dCLdad,dCLdq,dCMdq
    SWP(1).lbnd = 0.3; SWP(1).ubnd = 1; SWP(1).n = 20;
    SWP(2).lbnd = 0.3; SWP(2).ubnd = 1; SWP(2).n = 20;
    i = 2;
    VarSpan = nan(6,SWP(i).n); Sols = nan(6,SWP(i).n);
    VarSpan(i,:) = linspace( SWP(i).lbnd,SWP(i).ubnd,SWP(i).n );
    Vst = [];
    for j = 1:SWP(i).n
        [Atmp,~,~,~,~,~] = Aero.LinSysComp( x0,u0,VarSpan(:,j) );
        [V,D] = eig(Atmp); 
        [Sols(:,j) ,Vst] = sortMod(Vst,V,D);
        
        
        %Sols(:,j) = diag(D);
    end
    plot( real(Sols),imag(Sols),'*r' )

end
%% Functions
function [Xs,Md] = freeResp(A,x0,flg,MLNG,MLTD,tvec)
    switch flg
        case 'lon'
            i = [1,3,5,7,9,11];
            j = 1:6;
        case 'lonred'
            i = [1,3,5,11];
            j = [1:3,6];
            MLNG = MLNG(j,j);
        case 'latdir'
            i = [2,4,6,8,10,12];
        case 'latdired'
    
    end
    A = A(j,j);
    x0 = x0(i);
    Md = ModalPrp(A,x0,MLNG);
    if nargin > 5
        nt = length(tvec); nmod = length(Md);
        Xs = zeros(13,nt);
        for it = 1:nt
            in = 1;
            Xs(i,it) = Xs(i,it) + Md(in).mode*Md(in).incnd*exp( Md(in).eig*tvec(it) );
            for in = 2:nmod
                % if abs(Md(in).eig - Md(in-1).eig) < 1e-3
                %     Xs(i,it) = Xs(i,it) + Md(in).mode*...
                %         Md(in).incnd*exp( Md(in).eig*tvec(it) )*tvec(it);
                % else
                Xs(i,it) = Xs(i,it) + Md(in).mode*Md(in).incnd*exp( Md(in).eig*tvec(it) );
                % end
            end
        end
        Xs = Xs';
    else
        Xs = [];
    end
end

function [Md,ModPF] = ModalPrp(A,x0,MLNG)
%MODALPRP
%% Longitudinal
    [V,D] = eig( MLNG*A/inv(MLNG) );
    %V = MLNG*V;
    alp0 = V\x0;   % Coefficients
    neig = length(V(1,:));
    ModPF = nan(3);
    for i = 1:neig
        [Md(i).wn,Md(i).zita] = OmZitaCalc(D(i,i));
        Md(i).mode = V(:,i);
        Md(i).incnd = alp0(i);
        Md(i).eig = D(i,i);
        
    end
    % Modal Work
    for i = 1:neig
        for j = i:neig
            ModPF(i,j) = transpose(Md(i).mode)*Md(j).mode;
        end
    end

end

function [EiOr,Vst] = sortMod(Vst,V,D)
    nEg = length(D(:,1)); EiOr = nan(nEg,1);
    if ~isempty(Vst)
        idx = EiOr;
        for iE = 1:nEg
            Vt = V( abs( V(:,iE) )>0,iE );
            V(:,iE) = V(:,iE)*sign( Vt(1) );
            if imag(D(iE,iE)) ~= 0
                 V(:,iE) = V(:,iE)*sign(imag(D(iE,iE)));% It should avoid the sign change
            end
            nm = ctranspose(V(:,iE))*V(:,iE);
            V(:,iE) = V(:,iE)/nm;
            prJ = abs( transpose( V(:,iE) )*Vst(:,1) );
            prM = prJ;
            idx(iE) = 1;
            for k = 2:nEg
                prJ = abs( transpose( V(:,iE) )*Vst(:,k) );
                if prJ > prM
                    prM = prJ;
                    idx(iE) = k;
                end
            end
            EiOr( idx(iE) ) = D(iE,iE);
        end
    else
        for iE = 1:nEg
            Vt = V( abs( V(:,iE) )>0,iE );
            nm = ctranspose(V(:,iE))*V(:,iE);
            V(:,iE) = V(:,iE)*sign( Vt(1) );
            if imag(D(iE,iE)) ~= 0
                 V(:,iE) = V(:,iE)*sign(imag(D(iE,iE)));% It should avoid the sign change
            end
            V(:,iE) = V(:,iE)/nm;

        end
        EiOr = diag(D);
    end
    Vst = V;
end

