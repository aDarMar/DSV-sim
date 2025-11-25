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
    V0 = 150;           % Desired Speed
    gam0 = 0;           % Desired flight path angle
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
    x0([1,3]) = eqV(1:2); u0(1:2) = eqV(3:4);       % Assign u,w
    x0(11) = gam0 + atan(eqV(2)/eqV(1));            % Teta = gamma + alphaB
    % Checks if the minimum for f is actually a trim condition
    CHC = true;
    if CHC
        dx = Aero.DynEqs6DoF(x0,u0);
        f = norm(dx([1,3,5,9]),2)^2;
        if abs(f) > 1e-6
            error('Minimum is not zero for f, trim not found')
        end




    end


    %[~,~,~,~,x1,angl] = Aero.LinSysComp([V0,x0(5),-x0(9),x0(13)]);
    %x1(1) = V0*cos(angl(1)); x1(3) = V0*sin(angl(1));
    %dx2 = Aero.DynEqs6DoF(x1,[angl(3),angl(2)]);

    












%% 
    % % [Alon,Blon,Alatdir,Blatdir,x0,angl] = Aero.LinSysComp([V0,x0(5),-x0(9),x0(13)]);
    % % dx0 = zeros(13,1);
    % % dx0(1) = x0(1)*0.1*0; 
    % t0 = 0; tfin = 500; nt = 200; 
    % % tvec = linspace(t0,tfin,nt);
    % % 
    % % u0 = [1;angl(2);0;0];
    % dx0 = 0;
    % 
    % Dyn6DoF =@(t,x) Aero.DynEqs6DoF(x,u0);
    % 
    % % [tvec2,xnl] = ode113(Dyn6DoF,[t0,tfin],x0+dx0);
    % % xl = freeResp(Alon,dx0,'lonred',tvec2);
    % % xl = x0' + real(xl);
    % xl = xnl*0 + x0';
    % %% Plots
    % plotv = [tvec2,xnl,xl];
    % figS = [ ones(13,2), (2:14)', (115:127)' ]; % [figID,xidx,yidx1,yidx2]
    % %figS(4:7,4) = 114:117;% u,w,q,teta
    % IMTIT = {'State Variables'};
    % TIT = {'u','v','w','p','q','r','x_{CG}','y_{CG}','z_{CG}','\psi',...
    %     '\theta','\phi','m'};
    % LG = repmat({'-'},13,2);
    % PlotPerImag( plotv,5,figS,IMTIT,LG,TIT )

    %% Linearization
    Aero.LinSysComp(x0,u0);


    %% Functions
end

function [Xs,Md] = freeResp(A,x0,flg,tvec)
    
switch flg

    case 'lon'
        i = [1,3,5,7,9,11];
    case 'lonred'
        i = [1,3,5,11];

    case 'latdir'
        i = [2,4,6,8,10,12];
    case 'latdired'

end

    x0 = x0(i);
   Md = ModalPrp(A,x0);
    nt = length(tvec); nmod = length(Md);
    Xs = zeros(13,nt);
    for it = 1:nt
        for in = 1:nmod
            Xs(i,it) = Xs(i,it) + Md(in).mode*Md(in).incnd*exp( Md(in).eig*tvec(it) );
        end
    end
    Xs = Xs';
end

function [Md,ModPF] = ModalPrp(A,x0)
%MODALPRP
    [V,D] = eig(A);
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
            ModPF(i,j) = abs( transpose(Md(i).mode)*Md(j).mode );
        end
    end

end


