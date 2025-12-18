function [xref,x0] = Flight_Env(FLCond,AC,plotobj,rhom)
%FLIGHT_ENV This function generates a set of possible flight conditions (h, Va, m) 
%   for a given aircraft (AC) within specified limits (FLCond). It then 
%   filters these conditions to find equilibrium flight states (x0) and 
%   determines an optimal reference condition (xref) for controller design
% INPUT:
%   - FLCond: Vector of flight limits [Vmax, Vstall, Mlim, qmax, hmax]
%             * Vmax: Maximum True Airspeed [m/s]
%             * Vstall: Stall Speed [m/s]
%             * Mlim: Maximum Mach Number limit [-]
%             * qmax: Maximum Dynamic Pressure limit [Pa]
%             * hmax: Maximum Altitude [m]
%   - AC: Object containing aircraft characteristics 
%           (masses, geometry, aerodynamic functions, thrust law).
%   - plotobj: DataPlot object used for plotting the results.
%   - rhom: Vector of $\frac{m}{\rho}$ values for plotting contours.
% OUTPUT:
%   - xref: Optimal reference equilibrium condition for controller design
%           [Va, ga=0, h, m, m/rho, CL]
%   - x0: Matrix of all equilibrium flight conditions found.
%         Columns: [Va, 0, h, m, m/rho, CL, Emax]
% ----------------------------------------------------------------------- %
    % Unpack Flight Limits
    Vmax = FLCond(1); Vstall = FLCond(2);
    Mlim = FLCond(3); qmax = FLCond(4); hmax = FLCond(5);
    % Flag for testing (if true, uses a single pool for LHS design)
    TST = false;
    if TST
        % --- Single Pool of Flight Conditions (Testing) ---
        vls = lhsdesign(1500,3); % Latin Hypercube Sampling
        ms = vls(:,1)*(AC.MTO - AC.MZF) + AC.MZF; % Mass from MZF to MTOM
        hs = vls(:,2)*(hmax); % h from S/L to 25000ft
        Vs = vls(:,3)*(Vmax - Vstall) + Vstall; % VTAS from VStall to Vmax@h max
        x = [Vs(:),ms(:)*0,hs(:),ms(:),ms(:)*nan];
        
    else
        % --- Two Intervals of Flight Conditions
        %The possible flight conditions are divided in two intervals:
        % From S/L to 4500m (highest airport existing) MZF <-> MTO
        % 1. Low Altitude Segment: S/L to 4500m
        vls = lhsdesign(200,3);
        ms = vls(:,1)*(AC.MTO - AC.MZF) + AC.MZF;                   % MZF to MTOM
        hs = vls(:,2)*(4500);                                       % h from S/L to 25000ft
        Vs = vls(:,3)*(Vmax*0.8 - Vstall) + Vstall;                 % VTAS from VStall to Vmax@h max
        x = [Vs(:),ms(:)*0,hs(:),ms(:),ms(:)*nan,ms(:)*nan,ms(:)*nan];
        % 2. High Altitude Segment
        % From 4500 m to hmax MTO * 0.995 <-> MLND/0.99
        vls = lhsdesign(200,3);
        ms = vls(:,1)*(AC.MTO*0.99 - AC.MLND/0.99) + AC.MLND/0.99;  % MLND to 0.99MTOM
        hs = vls(:,2)*(hmax-4500) + 4500;                           % h from S/L to 25000ft
        Vs = vls(:,3)*(Vmax - Vstall*1.5) + Vstall*1.5;             % VTAS from VStall to Vmax@h max
        
        x = [ x;[Vs(:),ms(:)*0,hs(:),ms(:),ms(:)*nan,ms(:)*nan,ms(:)*nan] ];
        % Columns of x: [Va, ga, h, m, m/rho, CL, Emax ]
        %                1   2   3  4    5     6     7
    end
    
    [~, ~, ~, rhos] = atmosisa( x(:,3) ); x(:,5) = x(:,4)./rhos;    % m/rho
    x(:,6) = x(:,5)*2*9.81./(AC.Sw*x(:,1).^2);                      % CL @ L=W
    % Initialization for equilibrium check
    rif.Emax = -1; rif.V = Vmax*2; rif.iEmax = -1;
    j = 1; itemp = nan( 1,length( x(:,1) ) );
    Emax = 21;                                          % Assumed maximum L/D ratio for normalization
    % --- Filter Flight Conditions to Find Equilibrium States (x0) ---
    for i = 1:length( x(:,1) )
        [~, a, ~, rho] = atmosisa(x(i,3));
        M = x(i,1)/a; Re = AC.ReCalc(x(i,3),M);                     % Re and Mach numbers
        CLmax = AC.CLmax( M,Re )/1.1;                               % CLmax/1.1
        CD = AC.polar( M,Re,x(i,6) );                               % CD at current CL          
        flg = M < Mlim;                                             % Must be below Max Mach          
        flg = 0.5*rho*x(i,1)^2 < qmax && flg;                       % Must be below max q
        flg = x(i,6) <  CLmax && flg;                               % CL < CLmax
        T = AC.Thrust_Law( 1,x(i,3),'ipt' );
        flg =  ...
            CD < T/(0.5*rho*x(i,1)^2*AC.Sw) && flg;                 % Treq < Tmax
        x(i,7) = x(i,6)/CD; 
        if flg                                                      % Iif flg = true the chosen V h m combination if an equilibrium one
        itemp(j) = i;
           j = j + 1;
        end
    end
    % Extract equilibrium states
    itemp = itemp(~isnan(itemp));
    x0 = x(itemp,:);
%/* \label{cod:Flight_env_1} (continua...) */
%/*(... continua da \ref{cod:Flight_env_1}) */
    %% Reference Condition
    % Find the equilibrium condition that minimizes damping and maximizes omega_n. 
    % This is typically achieved by maximizing the L/D ratio (min damping) and 
    % minimizing speed (max \omega_n for phugoid).

    % The state vector for fmincon is [Va, CL, h]
    xref = fmincon(@objfun,[Vstall*1.5,1,1000],[],[],[],[],[Vstall*1.1,0.2,0],[Vmax,1.7,hmax],@nlcstr);
    
    objfun(xref);
    % Recalculate full state variables for xref
    xref = [xref(1),0,xref(3),mcalc(xref),nan,xref(2)];
    %   [Vs, 0, hs, ms, m/rho, CL ]
    %    1   2  3   4   5      6
    %% Plots
    sz = size( plotobj.plotv );
    plotv = nan( max( length(x(:,1)),sz(1) ) ,...
        sz(2)+6 );
    plotv( 1:sz(1),1:sz(2) ) = plotobj.plotv;                                   % Copy plot data into new vector
    plotv( 1:length(x(:,1)),sz(2)+1:2+sz(2) ) = x(:,[6,1]);                                     % Vs and CL
    plotv( 1:length(x0(:,1)),sz(2)+3:4+sz(2) ) = x0(:,[6,1]);                               % Vs and CL at equilibrium
    plotv( 1,sz(2)+5:sz(2)+6 ) = xref([6,1]);                                                  % Vs and CL at Reference Condition
    % [ CL,Vs@rho/m,Vs,CLs,Vs@Eq,CLs@Eq ]
    plotobj = DataPlot( plotv );                                                 % Update DataPlot object
    figS = [ 1,...
        ones(1,sz(2)-1),sz(2)+1,sz(2)+3,sz(2)+5,...                             % x indices
        2:sz(2),sz(2)+2,sz(2)+4,sz(2)+6];                                       % y indices
    XLAB = 'C$_L$'; YLAB = 'V [m/s]';
    LG = repmat({'-'},1,sz(2)+3);
    LIN = repmat({'-'},1,sz(2)+3);
    LIN{sz(2)} = 'osquare'; LIN{sz(2)+1} = 'oo'; LIN{sz(2)+2} = 'odiamond';
    for iG = 1:sz(2)-1
        LG{iG} = ['$\frac{m}{ \rho}= $ ',num2str(rhom(iG))];
    end
        LG{sz(2)} = 'Condizioni di Volo Generate';
        LG{sz(2)+1} = 'Condizioni di Volo Equilibrate';
        LG{sz(2)+2} = 'Condizione di riferimento';
    plotobj = plotobj.definePlot( figS( [2:sz(2)+3 ]),figS( [sz(2)+4:end ]),...
        figS(1),'legend',LG,'linestyle',LIN,'xlabel',XLAB,'ylabel',YLAB,...
        'grid','minor');
    plotobj.PlotPerImag([1,1],{'Condizioni di Volo'},{'Condizioni di Volo'},'cartesian');

    %% Functions        
    function f = objfun(x)
    %OBJFUN: Minimizes a weighted combination of 1/E (damping) and Va/Vmax
    %   (speed) to find a point with high L/D and low speed.
    %
    % INPUT:
    %   - x: Optimization state vector [Va, CL, h]
    % OUTPUT:
    %   - f: Weighted cost function value (to be minimized)
    % --------------------------------------------------------------- %
        w = 0.5; % Weighting factor
        [~, as, ~, ~] = atmosisa(x(3));
        MN = x(1)/as; ReN = AC.ReCalc(x(3),MN);
        E = x(2)/AC.polar( MN,ReN,x(2) );
        % Minimizing f maximizes the weighted objective.
        f = -( w*E/Emax + (1-w)*Vmax/x(1) );

    end
    
    function [c,ceq] = nlcstr(x)
    %NLCSTR: Nonlinear constraints for fmincon.
    %
    %   Ensures the reference condition x satisfies limits on mass, CL,
    %   dynamic pressure, and required thrust.
    %
    % INPUT:
    %   - x: Optimization state vector [Va, CL, h]
    % OUTPUT:
    %   - c: Vector of nonlinear inequality constraints (c <= 0)
    %   - ceq: Vector of nonlinear equality constraints (ceq = 0)
    % --------------------------------------------------------------- %
        [m,q,MN] = mcalc(x); %m = q*x(2)*AC.Sw/9.81;
        Th = AC.Thrust_Law( 1,x(3),'ipt' ); ReN = AC.ReCalc(x(3),MN);
        CDs = AC.polar(MN,ReN,x(2));
        % c(1): Mass upper limit (m - MTO <= 0)
        c(1) = m - AC.MTO;
        % c(2): Mass lower limit (-m + MZF <= 0)
        c(2) = - m + AC.MZF;
        % c(3): CL upper limit
        c(3) = x(2) - AC.CLmax(MN,ReN);
        % c(4): Dynamic Pressure upper limit
        c(4) = q - qmax;
        % c(5): Thrust required limit
        c(5) = CDs - Th/(q*AC.Sw);
        ceq = [];
    end
    
        function [m,q,MN] = mcalc(x)
        %MCALC: Calculates mass, dynamic pressure, and Mach number from [Va, CL, h].
        %
        % INPUT:
        %   - x: State vector [Va, CL, h]
        % OUTPUT:
        %   - m: Aircraft Mass [kg] (calculated from $m = \frac{q C_L S_w}{g}$)
        %   - q: Dynamic Pressure [Pa]
        %   - MN: Mach Number [-]
        % --------------------------------------------------------------- %
            [~, as, ~, ro] = atmosisa(x(3));
            MN = x(1)/as;
            q = 0.5*ro*x(1)^2;
            m = q*x(2)*AC.Sw/9.81;
        end


end


