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
    [Alng,Blng,~,Altd,Bltd,x0s] = Aero.LinSysComp(x0,u0);
    ik = 1; MTrsf = eye(6); 
    CHS = 'latdir';
    switch CHS
        case 'lon'
            Amat = Alng;
            ji = [1,3,5,7,9,11];
        case 'latdir'
            
            %ji = [2,4,6,8,10,12];
            ji = [2,4,6,10];
            Amat = Altd;%([1:3,5],[1:3,5]);
    end
    
    % MTlong(1,1) = 1/x0s(1); MTlong(2,2) = MTlong(1,1);
    % MTlong(3,3) = MTlong(1,1)*Aero.cref/2;
    dx0s = zeros(13,1);
    [~,Mod] = freeResp( Amat,dx0s,CHS,MTrsf );
    dx0s(ji) = 5*real( Mod(ik).mode )*0.1; %x0s(1)*0.05;                                   % Perturbation in Stability Axes
    t0 = 0; tfin = 180*0 + 4/(Mod(ik).zita*Mod(ik).wn);
    dx0 = dx0s;                                             % Perturbations in Body Axes
    dx0(1:3) = Aero.Wind2Body(dx0s(1:3),'W2B',alpha0,0);  
    dx0(4:6) = Aero.Wind2Body(dx0s(4:6),'W2B',alpha0,0);
    % Non-Linear Simulation
    options = odeset('RelTol',1e-6,'AbsTol',1e-5);
    Dyn6DoF =@(t,x) Aero.DynEqs6DoF(x,u0);
    tfin = 360;
    [tvec,xnl] = ode113(Dyn6DoF,[t0,tfin],x0+dx0,options);
    % Linear Simulation
    [xl,Mod] = freeResp(Amat,dx0s,CHS,MTrsf,tvec);
    xl = x0s' + real(xl);                % xl should be real but some very small imaginary parts are possible (1e-14)
    
    % % TEST
    % fn = @(t,x) Alng*x(:);
    % [tst1,tst2] = ode45( fn,[0,tfin],dx0s([1,3,5,7,9,11]) );
    % figure; plot( tst1,tst2(:,1) )
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
    for tv = 1:length(tvec)
        dxnl(tv,:) = Aero.DynEqs6DoF(xnl(tv,:)',u0)';
    end

    GRF = true;
    if GRF
        V0nl = sqrt( xnl(:,1).^2 + xnl(:,3).^2 + xnl(:,2).^2 );
        V0l = sqrt( temp(:,1).^2 + temp(:,3).^2 + temp(:,2).^2);
        plotv = [tvec,xnl,temp,F',FA',FT',FW',...
            atan( xnl(:,3)./xnl(:,1) )*180/pi,atan( temp(:,3)./temp(:,1) )*180/pi,...
            V0nl,V0l,...
            tvec*0+x0s(1),xnl(:,11)*180/pi,temp(:,11)*180/pi, ...
            asin( xnl(:,2)./V0nl )*180/pi,...
            asin( temp(:,2)./V0l )*180/pi ];

        % [ t, xnl(1:12), xl(1:12), F(1:3), M(1:3), FA,    FT,     FW,    alphaNL, alphaL, Vnl, VL, V0, tetaNL deg, tetaL deg, beta nl, beta L ]
        %   1  2:14       15:27     28:30   31:33   34:39  40:45   46:51   52       53      54   55 56  57          58
        %
        figS = [ ones(13,2), (2:14)', (15:27)' ]; % [figID,xidx,yidx1,yidx2]

        Aux = [ 2*ones(6,1),ones(6,1),(28:33)',(34:39)',(40:45)',(46:51)' ];
        figS = [ figS,nan( length(figS(:,1)), length(Aux(1,:)) - length(figS(1,:)) ) ; Aux  ];
        Aux = [ 3*ones(2,1),ones(2,1),[52,53,57:60;54:56,nan(1,3)] ];
        figS = [ figS,nan( length(figS(:,1)), length(Aux(1,:)) - length(figS(1,:)) ) ; Aux  ];
        %figS(4:7,4) = 114:117;% u,w,q,teta

        IMTIT = {'Stability Derivatives'};
        LG = repmat({'-'},18,4);
        LINS = repmat({'-','--','-.',':','-','--'},21,1);
        XLAB = repmat({'t [s]'},21,1); 
        YLAB = {'u [m/s]','v [m/s]','w [m/s]','p [rad/s]','q [rad/s]',...
            'r [rad/s]','x$_{CG}$ [m]','y$_{CG}$ [m]','z$_{CG}$ [m]',...
            '$\phi$ [rad]','$\theta$ [rad]','$\psi$ [rad]','m','F$_{B x}$ [N]',...
            'F$_{B y}$ [N]','F$_{B z}$ [N]','$\mathcal{L}$ [Nm]','$\mathcal{M}$ [Nm]',...
            '$\mathcal{N}$ [Nm]','$\alpha, \theta, \beta$ [deg]','V [m/s]' };

        %% Time Response
        plto = DataPlot(plotv);
        %IMSAV = {'State Variables_trim','Forces in Body_trim','Angles trim'};
        IMSAV = {'State Variables Phugoid','Forces in Body Phugoid','Angles Phugoid'};
        IMTIT = {'State Variables','Forces in Body','Angles'};
        TIT = {'u','v','w','p','q','r','x_{CG}','y_{CG}','z_{CG}','\psi',...
            '\theta','\phi','m','F_x','F_y','F_z','M_z','M_y','M_z'};
        LG = repmat({'-'},size( figS(:,3:end)) );
        for iG = 0:5
            LG{14+iG,1} = '$F_{tot}$'; LG{14+iG,2} = '$F_{A}$'; 
            LG{14+iG,3} = '$F_{T}$'; LG{14+iG,4} = '$F_{W}$';
        end
        LG{20,1} = '$\alpha$ Non lineare'; LG{20,2} = '$\alpha$ Lineare'; 
        LG{20,3} = '$\theta$ Non lineare'; LG{20,4} = '$\theta$ Lineare'; 
        LG{20,5} = '$\beta$ Non lineare'; LG{20,6} = '$\beta$ Lineare';
        LG{21,1} = 'V Non lineare'; LG{21,2} = 'V Lineare';  LG{21,3} = 'V$_0$';
        LINS(21,:) = repmat({'-',':'},1,3);
        for ig = 1:length(XLAB)
            yidx = figS(ig, ~isnan( figS(ig,:) ) );
            plto = plto.definePlot(figS(ig,2),yidx(3:end),...
                figS(ig,1),'legend',LG(ig,:),'grid','minor','xlabel',...
                XLAB{ig},...
                'ylabel',YLAB{ig},'linestyle',LINS(ig,:));
        end
        %plto.ylms([14,16,18],:) = repmat( [-10,10],3,1 );    

        figs = plto.PlotPerImag( 3,IMSAV,IMTIT,'cartesian',false );
        %% Phasor Plot
        % Normalised Eigenvectors
        % % MTlong(1,1) = 1/x0s(1); MTlong(2,2) = MTlong(1,1);
        % % MTlong(3,3) = MTlong(1,1)*Aero.cref/2;
        % % [~,Mod] = freeResp( Alng,dx0s,'lon',MTlong,[] );
        % % 
        % % IMSAV = {'Phasor Plot Reduced Phugoid'}; IMTIT = {'Phasor Plot'};
        % % figS = [ones(2,1),[1:5;6:10] ];
        % % figS = [ones(2,1),[1:4;5:8] ];
        % % LG = repmat({'$\frac{u}{U_0}$',...
        % %     '$\frac{w}{U_0}$','$q$','$\theta$'},2,1);
        % % plotv = [Mod(3).mode([1:3,6]);...
        % %     Mod(4).mode([1:3,6])/Mod(4).mode(6)]; plotv = plotv(:)';
        % % pltrad = DataPlot( plotv );
        % % for ig = 1:length(figS(:,1))
        % %     yidx = figS(ig, ~isnan( figS(ig,:) ) );
        % % pltrad = pltrad.definePlot( figS(ig,2),yidx(3:end),...
        % %         figS(ig,1),'legend',LG(ig,:),'grid','minor' );
        % % end
        % % figs = pltrad.PlotPerImag( [1,2],IMSAV,IMTIT,'polar',false );
        % 
        % plto.plotPolar([Mod(3).mode([1:3,5:6])/Mod(3).mode(5),...
        %     Mod(6).mode([1:3,5:6])/Mod(5).mode(6)],LEG);
    end

    %% Root Locus
    % dCLda,dCMda,dCMdad,dCLdad,dCLdq,dCMdq
    SWP(1).lbnd = 2*pi*0.2; SWP(1).ubnd = 2*pi*1.1; SWP(1).n = 200;     % CLa
    SWP(2).lbnd = -1; SWP(2).ubnd = 1; SWP(2).n = 438;                % CMa/SSM
    SWP(3).lbnd = -20; SWP(3).ubnd = 5; SWP(3).n = 500;              % CMda
    SWP(4).lbnd = -1; SWP(4).ubnd = 4; SWP(4).n = 500;              % CLda
    SWP(5).lbnd = -1; SWP(5).ubnd = 10; SWP(5).n = 500;              % CLdq
    SWP(6).lbnd = -57; SWP(6).ubnd = 10; SWP(6).n = 500;              % CMdq

    IMSAV = {'Root Locus'}; IMTIT = {'Root Locus'};
    Gain = {'$C_{L \alpha}$','$C_{\mathcal{M} \alpha}$',...
        'C$_{L_{\dot{\alpha}}}$','C$_{\mathcal{M}_{\dot{\alpha}}}$',...
        'C$_{L_{q}}$','C$_{\mathcal{M}_{q}}$'};
    for i = 1:length(SWP)
        VarSpan = nan(6,SWP(i).n); Sols = nan(4,SWP(i).n);
        VarSpan(i,:) = linspace( SWP(i).lbnd,SWP(i).ubnd,SWP(i).n );
        j = 1;
        [Atmp,~,~,~,~,~] = Aero.LinSysComp( x0,u0,VarSpan(:,j) );
        Atmp = Atmp([1:3,6],[1:3,6]);
        [Vst,Dst] = eig(Atmp);
        Sols(:,j) = diag(Dst);
        for j = 1:SWP(i).n
            if any(j==[174,383],2)
                disp('a')
            end
            [Atmp,~,~,~,~,~] = Aero.LinSysComp( x0,u0,VarSpan(:,j) );
            Atmp = Atmp([1:3,6],[1:3,6]);
            [V,D] = eig(Atmp);

            [Dst,Vst] = track_eigenvalues(Dst, D, Vst, V);

            %% [Sols(:,j) ,Vst] = sortMod(Vst,V,D);
            %temp = OmZitaCalc()

            Sols(:,j) = diag(Dst);
        end
        
        sz = size( Sols );
        plotv = nan( sz(2),sz(1) );
        for iG = 1:length(plotv(1,:))
            plotv(:,2*iG-1) = real(Sols(iG,:)');
            plotv(:,2*iG) = imag(Sols(iG,:)');
        end
        plrl = DataPlot( plotv );
        plrl = plrl.rootLocus( VarSpan(i,:),Gain{i},false );
    end
    % plrl.definePlot( [1:2:sz(1)*2]',[2:2:sz(1)*2]',1 );
    % plrl.PlotPerImag( [1,1],IMSAV,IMTIT,'cartesian',false );

end
%% Functions
function [Xs,Md] = freeResp(A,x0,flg,Mtrs,tvec)
%FREERESP: function that calculates the analitycal free response of a
%linear system.
%   INPUT
%   - A: state matrix
%   - x0: initial condition
%   - flg: flag specifying if the dynamic is longitudinal or
%       latero-directional
%   - Mtrs: matrix used to transform the eigenvectors
%   -tvec: time vector where the solution is computed
% ----------------------------------------------------------------------- %
    switch flg
        case 'lon'
            i = [1,3,5,7,9,11];
            j = 1:6;
        case 'lonred'
            i = [1,3,5,11];
            j = [1:3,6];
            Mtrs = Mtrs(j,j);
        % case 'latdir'
        %     i = [2,4,6,8,10,12];
        %     j = 1:6;
        % case 'latdired'
        case 'latdir'
            i = [2,4,6,10];
            j = [1:3,5];
            Mtrs = Mtrs(j,j);
    end
    A = A(j,j);                     % Reduce matrix if flg is 'reduced'
    x0 = x0(i);                     % Take just the state varibales needed
    Md = ModalPrp(A,x0,Mtrs);
    if nargin > 4
        % If tvec is passed the function gives the response
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
        % Otherwise returns just the modal properties
        Xs = [];
    end
end

function [Md,ModPF] = ModalPrp(A,x0,MLNG)
%MODALPRP
%% Longitudinal
    [V,D] = eig( MLNG*A/MLNG ); % M*A*M(M^-1)
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
%SORMOD: function that sorts the eigenvalues based on the shape of
%eigenvectors
%   INPUT
%   - Vst: Eigenvectors at previous time step
%   - V: eigenvectors at curret time-step to be sorted
%   - D: eigenvalues at current time
% ---------------------------------------------------------------------- %
    nEg = length( D(:,1) );     % Number of Eigenvalues
    EiOr = nan(nEg,1);          % Vector to store ordered eigenvalues
    if ~isempty(Vst)
        idx = EiOr;
        for iE = 1:nEg
            diff = V(:,iE) - Vst(:,1);
            mindif = max( norm(diff,2) );
            idx(iE) = 1;
            for jE = 2:nEg
                diff = V(:,iE) - Vst(:,jE);
                if max( norm(diff,2) ) < mindif
                    mindif = max( abs(diff) );
                    idx(iE) = jE;
    
                end
            end
        end
        % Order Eigenvalues
        for iE = 1:nEg
            Vst( iE,: ) = V( idx(iE),: );
            EiOr( iE ) = D( idx(iE),idx(iE) );
        end
    else
        EiOr = diag( D );
        Vst = V;
    end



    % %         Vt = V( abs( V(:,iE) )>0,iE );
    % %         V(:,iE) = V(:,iE)*sign( Vt(1) );
    % %         if imag(D(iE,iE)) ~= 0
    % %              V(:,iE) = V(:,iE)*sign(imag(D(iE,iE)));% It should avoid the sign change
    % %         end
    % %         nm = ctranspose(V(:,iE))*V(:,iE);
    % %         V(:,iE) = V(:,iE)/nm;
    % %         prJ = abs( transpose( V(:,iE) )*Vst(:,1) );
    % %         prM = prJ;
    % %         idx(iE) = 1;
    % %         for k = 2:nEg
    % %             prJ = abs( transpose( V(:,iE) )*Vst(:,k) );
    % %             if prJ > prM
    % %                 prM = prJ;
    % %                 idx(iE) = k;
    % %             end
    % %         end
    % %         EiOr( idx(iE) ) = D(iE,iE);
    % %     end
    % % else
    % %     for iE = 1:nEg
    % %         Vt = V( abs( V(:,iE) )>0,iE );
    % %         nm = ctranspose(V(:,iE))*V(:,iE);
    % %         V(:,iE) = V(:,iE)*sign( Vt(1) );
    % %         if imag(D(iE,iE)) ~= 0
    % %              V(:,iE) = V(:,iE)*sign(imag(D(iE,iE)));% It should avoid the sign change
    % %         end
    % %         V(:,iE) = V(:,iE)/nm;
    % % 
    % %     end
    % %     EiOr = diag(D);
    % % end
    % % Vst = V;
end

function [matched_eigs,matched_eivec] = track_eigenvalues(eigs_prev, eigs_curr, V_prev, V_curr)
    n = length( eigs_prev );
    matched_eigs = zeros( size(eigs_curr) );
    matched_eivec = V_prev*0;
    for i = 1:n
        % Combine distance and eigenvector similarity
        distances = abs( eigs_prev(i,i) - diag(eigs_curr) );        % eig(i@t) - eig(j@t-dx) for all j
        similarities = diag( abs(diag(V_prev(:, i)' * V_curr)) );   % 
        
        % Weighted score (adjust weights as needed)
        m = max(distances);
        if m == 0
            m = 1;
        end
        score = 0.7 * (1 - distances/m) + 0.3 * similarities;
        [~, best_match] = max(score);
        
        matched_eigs(i,i) = eigs_curr(best_match,best_match);
        matched_eivec(:,i) = V_curr(:,best_match);
        eigs_curr(best_match,best_match) = NaN;  % Remove matched eigenvalue
        idx(i) = best_match;
    end
end