close all; clear; clc;
%BUILDAEROMODL: script that builds the .mat files used by the ACclass to
%define the aerodynamic properties of the aircraft.
addpath('Data')
addpath('Classes')

load('longitudinal_database.mat')

nmfl = "Q100_comp.aero"; AC = ACclass(nmfl);

nA = 20; ndE = 9; nM = 4; nh = 9;
k = 1; h = 1; kk = 1;
aero_synt = nan(nM*nh,17); GRF = false; DBG = true;
%% CA vs Alpha Fit
for iM = 1:nM
    
    for iH = 1:nh
        % Stability Derivatives
        Re = AC.ReCalc( SLNG(k,3),SLNG(k,2) );
        CLq = SLNG(k,9); CMq = SLNG(k,10);
        Cyb = SLAT(k,4); Cnb = SLAT(k,5);
        CL = nan(nA,1); CD = CL; CM = CL; 
        CLdav = CL; alpha = CL; CMdav = CL;
        for iA = 1:nA
            %[ a,M,h,CL,CD,CM,CLad,CMad,CLq,CMq ]
            alpha(iA) = SLNG(k,1);
            CL(iA) = SLNG(k,4);
            CD(iA) = SLNG(k,5);
            CM(iA) = SLNG(k,6);
            CLdav(iA) = SLNG(k,7);
            CMdav(iA) = SLNG(k,8);
            % [M,h,alpha,Cyb,Cnb,Clb,Clp,Cnp,Clr,Cnr]
            
            LAT = SLAT(k,6:10);
%             Clb(iA) = ;
%             Clp(iA) = SLAT(k,7);
%             Cnp(iA) = SLAT(k,8);
%             Clr(iA) = SLAT(k,9);
%             Cnr(iA) = SLAT(k,10);
            
            
            k = k + 1;
        end
        
        % Control Derivatives
        for iE = 1:ndE
            %[ deltaE,M,h,,dCLde,dCMde ]]
            deltaE(iE) = CTR(kk,1);
            CLev(iE) = CTR(kk,5);
            CMev(iE) = CTR(kk,4);
            kk = kk + 1;
        end
        alphal = 8;
        %% Longitudinal 
        % Stability Derivatives
        [CLa,CL0,CLmax,alpham] = CAvsAoa(alpha,CL,alphal);
        [CMa,CM0,~,~] = CAvsAoa(alpha,CM,8,alpham);
        [CD0,K,CLcdm] = polar(CL,CD,alpha,alpham);
        
        [errCLda,CLda,~,~] = CAvsAoa(alpha,CLdav,alphal);
        [errCMda,CMda,~,~] = CAvsAoa(alpha,CMdav,alphal);
        
        temp = nan( 1,2*length(LAT) );
        for iL = 1:length(LAT)
            % [Clb,Clp,Cnp,Clr,Cnr]
            [temp(2*k-1),temp(2*k),~,~] = CAvsAoa(alpha,LAT(:,k),alphal);
        end
        
        % Control Derivatives
        [CMe,CM0e,~,~] = CAvsAoa(deltaE,CMev,deltaE(end)+1,deltaE(end));
        [CLe,CL0e,~,~] = CAvsAoa(deltaE,CLev,deltaE(end)+1,deltaE(end));
        
        % [ M,Re,CD0,K,CL@CDmin,CLmax,alphamax,CLa,CL0,
        %   CMa,CM0,x,CLda,x,CMda,CLq,CMq,Cyb,Cnb,Clb,Clp,Cnp,Clr,Cnr ]

        aero_synt(h,:) = [SLNG(k-1,2)...
            ,Re,CD0,K,CLcdm,CLmax,alpham,CLa,CL0,CMa,CM0,...
            errCLda,CLda,errCMda,CMda,CLq,CMq,Cyb,Cnb,temp];
        % [ M,Re,CMe,CM0e,CLe,CL0e,min( deltaE ),max( deltaE ) ]
        aero_synt_C(h,:) = [SLNG(k-1,2),Re,CMe,CM0e,CLe,CL0e,deltaE(1),deltaE(end)];

        if GRF
            plotData(2,SLNG(k-1,2),Re,deltaE,CLev,CMev,aero_synt_C(h,:));
            plotData(1,SLNG(k-1,2),Re,alpha,CL,CD,CM,CLdav,CMdav,aero_synt(h,:));
        end
        h = h + 1;
    end
    
    %synth(h,:) = [DATA.INP.mach(iM)...
    %    ,Re,CD0,K,CLcdm,CLa,CL0,max(CL),AoA(CL==max(CL))];
    % [M,Re,CD0,K,CL@min(Cd),CLa,CL0,CLmax,AoA@CLmax]
    
end

%% CA vs Mach Fit
aero_synt_M = nan(nA,4);
k = 1;
for iA = 1:nA
    
    CL = nan(nM,1); CD = CL; CM = CL;
    CLdav = CL; M = CL; CMdav = CL;
       for iM = 1:nM
           q = iA + nA*nh*(iM-1);
           M(iM) = SLNG(q,2);
           CL(iM) = SLNG(q,4);
           CM(iM) = SLNG(q,5);
       end
       CMp = polyfit(M,CM,2); 
       CLp = polyfit(M,CL,4);
       aero_synt_M(k,:) = [alpha(iA),CMp];
       
       plotvsM( alpha(iA),M,CL,CM );
       
       k = k + 1;
end

%% Save Results
SVF = false;
if SVF
save('Data\aero_database_long.mat','aero_synt','aero_synt_M','aero_synt_C');
end
% CA vs Re Fit

%% Auxiliary Functions 

function [CAa,CA0,CAmax,alpham] = CAvsAoa(alpha,CA,alphalin,alpham)

    CAmax = max(CA);    % Max value of CA. FOr variables different from CL it doesnt have a meaning
    if nargin < 4
        % IF alpha_max is not passed it is evaluated. NOTE: the results is
        % meaningful only iof CA = CL
        alpham = alpha( CA==CAmax );
    end
    
    i = 1:length(CA);
    im = i(alpha==alpham);
    
    CAmax = CA(im);
    alpha = alpha(i<im+1); % Excludes post stall
    CA = CA(i<im+1);
   
    % Linear Trait Between -6 and 6 Degrees ASSUMPTION
    
    alpha = alpha(alpha<alphalin); alpha = alpha(alpha>-alphalin);
    CA = CA(alpha<alphalin); CA = CA(alpha>-alphalin);
    
    p2 = polyfit(alpha,CA,1);
    
    CAa = p2(1); CA0 = p2(2);
end

function [CD0,K,CLcdm,plr] = polar(CL,CD,alpha,alpham)
    plr = @polbuild;
    CL = CL(alpha<alpham); CD = CD(alpha<alpham);
    p1 = polyfit(CL,CD,2);

    K = p1(1); CLcdm = -0.5*p1(2)/p1(1); CD0 = p1(3) - K*CLcdm^2;

        function out = polbuild(CL)
            out = p1(1)*CL.^2 + p1(2)*CL + p1(3);
        end
end

function ax = plotData(dF,M,Re,alpha,CL,CD,CM,CLd,CMd,synth)

    %   - synth: M,Re,CD0,K,CL@CDmin,CLmax,alphamax,CLa,CL0,CMa,CM0, ,CLda, ,CMda
    fig = figure();
    
    switch dF
        case 1
            % Stability
            plotv = [alpha(:),CL(:),CM(:),CLd(:),CMd(:)];
            TIT = {'CL - $\alpha$','CM - $\alpha$','CL$_{\dot{\alpha}}$ - $\alpha$',...
                'CM$_{\dot{\alpha}}$ - $\alpha$','Polar' };
            for i = 1:4
                ax(i) = subplot(3,2,i);
                plot(ax(i),plotv(:,1),plotv(:,i+1),'or' ); hold(ax(i),'on');
                plot(ax(i),plotv(:,1),synth(8+2*(i-1))*alpha + synth(9+2*(i-1)) ...
                    ,'--r' );
                title(ax(i),TIT{i},'Interpreter','Latex')
                legend(ax(i),{'Data','Linear Fit'})
            end
            i = i +1; ax(i) = subplot(3,2,5:6);
            plot(ax(i),CL,CD); hold(ax(i),'on');
            plot(ax(i),CL,synth(3)+synth(4)*(CL-synth(5)).^2,'--r')
        case 2
            % COntrol
            plotv = [alpha(:),CD(:),CL(:)]; % CD is CM
            TIT = {'\Delta CL - \delta_e','\Delta CM - \delta_e' };
            for i = 1:2
                ax(i) = subplot(2,1,i);
                plot(ax(i),plotv(:,1),plotv(:,i+1),'or' ); hold(ax(i),'on');
                plot(ax(i),plotv(:,1),CM(3+2*(i-1))*alpha + CM(4+2*(i-1)) ...
                    ,'--r' ); % CM is synth
                title(ax(i),TIT{i})
                legend(ax(i),{'Data','Linear Fit'})
            end
    end
    sgtitle(['Re = ',num2str(Re),' M = ',num2str(M)])
end

function ax = plotvsM(Alpha,M,CL,CM,synth)
    fig = figure();
    plotv = [M(:),CL(:),CM(:)];
    
    
    TIT = { 'CL - M','CM - M',' ' };
    
    u = [4,1];
    for i = 1:length(plotv(1,:))-1
        ax(i) = subplot(3,1,i);
        C0 = plotv(1,i+1)*sqrt(1-M(1)^2);
        pp = polyfit(plotv(:,1),plotv(:,i+1),u(i));
        C = polyval(pp,M);
        plot(ax(i),plotv(:,1),plotv(:,i+1),'or' ); hold(ax(i),'on');
        plot(ax(i),plotv(:,1),C0./sqrt(1-M.^2),'*g');
        plot(ax(i),plotv(:,1),C,'--r');
        title(ax(i),TIT{i});
%                 plot(ax(i),plotv(:,1),synth(8+2*(i-1))*alpha + synth(9+2*(i-1)) ...
%                     ,'--r' );
    end
    sgtitle(['\Alpha = ',num2str(Alpha)]);%,' M = ',num2str(M)])
end