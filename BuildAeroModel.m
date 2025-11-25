close all; clear; clc;
%BUILDAEROMODL: script that builds the .mat files used by the ACclass to
%define the aerodynamic properties of the aircraft.
%   OUTPUT
%   - aero_synt:   [ M,Re,CD0,K,CL@CDmin,CLmax,alphamax,CLa,CL0,CMa,CM0,
%                       CLda_a,CLda0,CMda_a,CMda0,CMq,CLq,Cyb,Cnb,Clb,Clp,
%                       Cnp,Clr,Cnr ]
%   - aero_synt_C: [ M,Re,CMe,CM0e,CLe,CL0e,min( deltaE ),max( deltaE ) ]
%   - aero_synt_M: [ alpha,CLm,CL0m,CMm,CM0m ]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('Data')
addpath('Classes')
addpath('Functions')

load('longitudinal_database.mat')

nmfl = "Q100_comp.aero"; AC = ACclass(nmfl);

nA = 20; ndE = 9; nM = 4; nh = 9;
k = 1; h = 1; kk = 1;
aero_synt = nan(nM*nh,19); GRF = false; DBG = true;
aero_synt_C = nan(nM*nh,8);
%% CA vs Alpha Fit
for iM = 1:nM
    
    for iH = 1:nh
        % Stability Derivatives
        iS = nA*(iH-1) + nA*nh*(iM-1) + 1;      % Starting Index
        iE = nA*iH + nA*nh*(iM-1);              % Ending index
        % Longitudinal Control
        iSe = ndE*(iH-1) + ndE*nh*(iM-1) + 1;    % Starting Index
        iEe = ndE*iH + ndE*nh*(iM-1);            % Ending index
        % Latero-Directional Control

        alphal = 8;                             % End of linearity alpha 
        Re = AC.ReCalc( SLNG(iS,3),SLNG(iS,2) );  % Reynolds Number

        %% Longitudinal 
        % Stability Derivatives
        [CLa,CL0,CLmax,alpham] = CAvsAoa(SLNG(iS:iE,1),...
            SLNG(iS:iE,4),alphal);                          % alpha,CL,alphal
        [CD0,K,CLcdm] = polar(SLNG(iS:iE,4),SLNG(iS:iE,5),...
            SLNG(iS:iE,1),alpham);
        
        aero_synt(h,1:9) = [SLNG(iS,2),Re,CD0,K,CLcdm,...   
            CLmax,alpham,CLa,CL0];
        aero_synt(h,10:15) = AeroLinRegs( SLNG(iS:iE,1),... % [CM,CLad,CMad]
            SLNG(iS:iE,6:8),alphal,alpham );
        aero_synt(h,16:17) = SLNG(iS,9:10);                 % [CMq,CLq]
        % Control Derivatives
        aero_synt_C(h,1:2) = [SLNG(iS,2),Re];              % [M,Re]
        aero_synt_C(h,3:6) = AeroLinRegs( CTR(iSe:iEe,1),...
            CTR(iSe:iEe,4:5),15 );                             % [CMe,CM0e,CLe,CL0e]
        aero_synt_C(h,7:8) = [min(CTR(iSe:iEe,1))...
            ,max(CTR(iSe:iEe,1)) ];       % [min( deltaE ),max( deltaE ) ]
        
        %% LATERO-DIRECTIONAL
        % Stability Derivatives
        aero_synt(h,18:19) = SLAT(iS,4:5);                  % [Cyb,Cnb
        aero_synt(h,20:29) = AeroLinRegs( SLNG(iS:iE,1),...
            SLAT(iS:iE,6:10),alphal,alpham);                      % [Clb,Clp,Cnp,Clr,Cnr]
        % Control Derivatives

        if GRF
            % Stability Derivatives vs Alpha Plot
            plotv = [SLNG(iS:iE,[1,4:8]),SLAT(iS:iE,6:10)]; % alpha,CL,CD,CM,CLda,CMda
            auxp = CTR(iSe:iEe,[1,4:5]);
            

            CL = aero_synt(h,8)*SLNG(iS:iE,1) + aero_synt(h,9);
            CD = aero_synt(h,3) + aero_synt(h,4)*...
                (SLNG(iS:iE,4)-aero_synt(h,5)).^2;
            % Longitudinal Stability
            for iP = 1:3
                graf(:,iP) = aero_synt(h,9+2*iP-1)*SLNG(iS:iE,1) ...
                    + aero_synt(h,9+2*iP);
            end
            % Latero Directional Stability
            for iP = 1:5
                graf(:,iP+3) = aero_synt(h,19+2*iP-1)*SLNG(iS:iE,1) ...
                    + aero_synt(h,19+2*iP);
            end
            % Longitudinal Control
            for iP = 1:2
                graf2(:,iP) = aero_synt_C(h,2+2*iP-1)*CTR(iSe:iEe,1) ...
                    + aero_synt_C(h,2+2*iP);
            end
            % Check DImensions
            if ndE > nA
                plotv = [plotv,auxp,CL,CD,graf,graf2;nan(  ndE-nA,length(plotv(1,:)) ) ]; % FINIRE
            else
                plotv = [ plotv,CL,CD,graf,[auxp,graf2;...
                    nan( nA-ndE,length( [auxp(1,:),graf2(1,:)] ) ) ] ];
            end



            %plotv = [plotv,CL,CD,graf];
            figS = [ ones(12,1),[1,2,ones(1,8),ones(1,2)*22]',...
                [(302:311),323:324]',[(112:121),125:126]' ];
            % 
            IMTIT = {'Stability Derivatives'};
            LG = repmat({'-'},12,2);
            TIT = {'CL - $\alpha$','Polar','CM - $\alpha$','CL$_{\dot{\alpha}}$ - $\alpha$',...
                    'CM$_{\dot{\alpha}}$ - $\alpha$','Cl_\beta - \alpha','Cl_p - \alpha',...
                    'Cn_p - \alpha','Cl_r - \alpha','Cn_r - \alpha','CM_{\delta_E} - \delta_E',...
                    'CL_{\delta_E} - \delta_E'};
            PlotPerImag( plotv,5,figS,IMTIT,LG,TIT )

        end
        h = h + 1;
    end

    
end

%% CA vs Mach Fit
aero_synt_M = nan(nA,4);
h = 1; iM = 1:nM;
for iA = 1:nA
    q = iA + nA*nh*(iM-1);  % Mach Indices at same alpha
    aero_synt_M(h,2:5) = AeroLinRegs( SLNG(q,2),... % [CL,CM]
        SLNG(q,[4,6]) );
    aero_synt_M(h,1) = SLNG(iA,1);

    if GRF
        plotvsM( alpha(iA),M,CL,CM );
    end
    h = h + 1;
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
        % meaningful only if CA = CL
        alpham = alpha( CA==CAmax );
    end
    % Removing NaNs
    CA = CA( ~isnan(CA) );
    alpha = alpha( ~isnan(CA) );

    i = 1:length(CA);
    im = i(alpha==alpham);
    if isempty(im) && isempty(CA)
        % If CA is empty the values are set as NaN
        CA = nan;
        alpha = nan;
    elseif isempty(im)
        % If the value is not found BUT CA is not empty than the last alpha is taken
        im = i(end);
        CAmax = CA(im);
        alpha = alpha(~(i>im)); % Excludes post stall
        CA = CA(~(i>im));
    end

    CA = CA(all([~(alpha>alphalin),~(alpha<-alphalin)],2)); 
    alpha = alpha(all([~(alpha>alphalin),~(alpha<-alphalin)],2));
    ord = 1;

    if ~(length(CA)>ord)
        % Works only if ord = 1
        p2 = polyfit(alpha,CA,min( ord,length(CA)-1 ) );
        CAa = nan; CA0 = p2
    else
        p2 = polyfit(alpha,CA,ord);
        CAa = p2(1); CA0 = p2(2);
    end
    
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

% function ax = plotData(dF,aero_synt)
% 
%     %   - synth: M,Re,CD0,K,CL@CDmin,CLmax,alphamax,CLa,CL0,CMa,CM0, ,CLda, ,CMda
%     fig = figure();
% 
%     switch dF
%         case 1
%             % Stability
%             plotv = [alpha(:),CL(:),CM(:),CLd(:),CMd(:)];
%             TIT = {'CL - $\alpha$','Polar','CM - $\alpha$','CL$_{\dot{\alpha}}$ - $\alpha$',...
%                 'CM$_{\dot{\alpha}}$ - $\alpha$',};
%             for i = 1:4
%                 ax(i) = subplot(3,2,i);
%                 plot(ax(i),plotv(:,1),plotv(:,i+1),'or' ); hold(ax(i),'on');
%                 plot(ax(i),plotv(:,1),synth(8+2*(i-1))*alpha + synth(9+2*(i-1)) ...
%                     ,'--r' );
%                 title(ax(i),TIT{i},'Interpreter','Latex')
%                 legend(ax(i),{'Data','Linear Fit'})
%             end
%             i = i +1; ax(i) = subplot(3,2,5:6);
%             plot(ax(i),CL,CD); hold(ax(i),'on');
%             plot(ax(i),CL,synth(3)+synth(4)*(CL-synth(5)).^2,'--r')
%         case 2
%             % COntrol
%             plotv = [alpha(:),CD(:),CL(:)]; % CD is CM
%             TIT = {'\Delta CL - \delta_e','\Delta CM - \delta_e' };
%             for i = 1:2
%                 ax(i) = subplot(2,1,i);
%                 plot(ax(i),plotv(:,1),plotv(:,i+1),'or' ); hold(ax(i),'on');
%                 plot(ax(i),plotv(:,1),CM(3+2*(i-1))*alpha + CM(4+2*(i-1)) ...
%                     ,'--r' ); % CM is synth
%                 title(ax(i),TIT{i})
%                 legend(ax(i),{'Data','Linear Fit'})
%             end
%     end
%     sgtitle(['Re = ',num2str(Re),' M = ',num2str(M)])
% end

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

function reslin = AeroLinRegs(alpha,CA,alphal,alpham)
%RESLIN: function that returns the linear regressions for data given as
%input
%   INPUT
%   - alpha: vector of alphas [deg]
%   - CAs: vector/matrix of aerodynamic coefficients
if nargin <3
    alphal = max(alpha); alpham = max(alpha);
elseif nargin < 4
    alpham = alphal;
end
    nipt = length( CA(1,:) );
    reslin = nan(1,nipt*2);
    for iL = 1:nipt
        % [Clb,Clp,Cnp,Clr,Cnr]
        [reslin(2*iL-1),reslin(2*iL),~,~] = CAvsAoa(alpha,CA(:,iL),...
            alphal,alpham);
    end
end