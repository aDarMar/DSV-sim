function [xref,x0] = Flight_Env(FLCond,AC,plotobj,rhom)
%FLIGHT_ENV Plots the possible flight conditions for a given AC and returns
%the reference point to calculate gains for controllers.
%   Detailed explanation goes here

Vmax = FLCond(1); Vstall = FLCond(2);
Mlim = FLCond(3); qmax = FLCond(4); hmax = FLCond(5);
TST = false;
if TST
    % One pool of flight conditions
    vls = lhsdesign(1500,3);
    ms = vls(:,1)*(AC.MTO - AC.MZF) + AC.MZF;
    hs = vls(:,2)*(hmax); % h from S/L to 25000ft
    Vs = vls(:,3)*(Vmax - Vstall) + Vstall; % VTAS from VStall to Vmax@h max
    x = [Vs(:),ms(:)*0,hs(:),ms(:),ms(:)*nan];
    
else
    % We divide the possible flight conditions is two intervals:
    % From S/L to 4500m (highest airport existing) MZF <-> MTO
    vls = lhsdesign(200,3);
    ms = vls(:,1)*(AC.MTO - AC.MZF) + AC.MZF;                   % MZF to MTOM
    hs = vls(:,2)*(4500);                                       % h from S/L to 25000ft
    Vs = vls(:,3)*(Vmax*0.8 - Vstall) + Vstall;                 % VTAS from VStall to Vmax@h max
    
    x = [Vs(:),ms(:)*0,hs(:),ms(:),ms(:)*nan,ms(:)*nan,ms(:)*nan];
    % From 4500 m to hmax MTO * 0.995 <-> MLND/0.99
    vls = lhsdesign(200,3);
    ms = vls(:,1)*(AC.MTO*0.99 - AC.MLND/0.99) + AC.MLND/0.99;  % MLND to 0.99MTOM
    hs = vls(:,2)*(hmax-4500) + 4500;                           % h from S/L to 25000ft
    Vs = vls(:,3)*(Vmax - Vstall*1.5) + Vstall*1.5;             % VTAS from VStall to Vmax@h max
    
    x = [ x;[Vs(:),ms(:)*0,hs(:),ms(:),ms(:)*nan,ms(:)*nan,ms(:)*nan] ];
    %   [Vs, nan, hs, ms, m/rho, CL, Emax ]
    %    1    2    3   4   5     6
end

[~, ~, ~, rhos] = atmosisa( x(:,3) ); x(:,5) = x(:,4)./rhos;                % m/rho
x(:,6) = x(:,5)*2*9.81./(AC.Sw*x(:,1).^2);                                  % CL @ L=W

rif.Emax = -1; rif.V = Vmax*2; rif.iEmax = -1;
j = 1; itemp = nan( 1,length( x(:,1) ) );
Emax = 21;                                                                  % Assumed Emax TODO RENDERLO DIP.DALL'AEREO

for i = 1:length( x(:,1) )
    [T, a, P, rho] = atmosisa(x(i,3));
    M = x(i,1)/a; Re = AC.ReCalc(x(i,3),M);                                 % Re and Mach numbers
    CLmax = AC.CLmax( M,Re )/1.1; CD = AC.polar( M,Re,x(i,6) );
    flg = M < Mlim;                                                         % Must be below Max Mach          
    flg = 0.5*rho*x(i,1)^2 < qmax && flg;                                   % Must be below max q
    flg = x(i,6) <  CLmax && flg;                                           % CL < CLmax
    T = AC.Thrust_Law( 1,x(i,3),'ipt' );
    flg =  ...
        CD < T/(0.5*rho*x(i,1)^2*AC.Sw) && flg;                             % Treq < Tmax
    x(i,7) = x(i,6)/CD; 
    if flg                                                                  % Iif flg = true the chosen V h m combination if an equilibrium one
    itemp(j) = i;
       j = j + 1;
    end
end
itemp = itemp(~isnan(itemp));
x0 = x(itemp,:);

%% Reference Condition
% We want the equilibrium condition at max L/D (minimum damping) and
% minimum speed (maximum wn)

xref = fmincon(@objfun,[Vstall*1.5,1,1000],[],[],[],[],[Vstall*1.1,0.2,0],[Vmax,1.7,hmax],@nlcstr);

objfun(xref);
xref = [xref(1),0,xref(3),mcalc(xref),nan,xref(2)];
% [Vs, nan, hs, ms, m/rho, CL, Emax ]

%   [Vs, 0, hs, ms, m/rho, CL ]
%    1    2    3   4   5     6
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
        % Objective function
       w = 0.5;
        [~, as, ~, ~] = atmosisa(x(3));
        MN = x(1)/as; ReN = AC.ReCalc(x(3),MN);
        E = x(2)/AC.polar( MN,ReN,x(2) );
        
        f = -( w*E/Emax + (1-w)*Vmax/x(1) );
        
    end

    function [c,ceq] = nlcstr(x)
    %NLCSTR: Nonlinear constraints
      
        [m,q,MN] = mcalc(x); %m = q*x(2)*AC.Sw/9.81;
        Th = AC.Thrust_Law( 1,x(3),'ipt' ); ReN = AC.ReCalc(x(3),MN);
        CDs = AC.polar(MN,ReN,x(2));
        c(1) = m - AC.MTO;
        c(2) = - m + AC.MZF;
        c(3) = x(2) - AC.CLmax(MN,ReN);
        c(4) = q - qmax;
        c(5) = CDs - Th/(q*AC.Sw);
        ceq = [];
    end

    function [m,q,MN] = mcalc(x)
        [~, as, ~, ro] = atmosisa(x(3));
        MN = x(1)/as;
        q = 0.5*ro*x(1)^2;
        m = q*x(2)*AC.Sw/9.81;
    end


end


