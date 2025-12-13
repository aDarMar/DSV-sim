function plot_results(AC,flg,t,x,x_add,y_way,x_stop,windpx,windpy,GEO)
%PLOT_RESULTS Function that plots the results of the integration
%   INPUT
%   - x:  state vector [Va,ga,h,m,CL,T,VD,psi,phi,p,mu,l]
%   - x_add: additional data taken from ode's OutputFcn 
%           [IDl,IASd,hdotc,Kh,Vd,CL,T,Roll,PhiD,PsiD,dR]
%   - x_stop: [t,bounds] at which the integration is stopped 
%                       because a waypoint has been reached
%   tl,xl solution vectors for the linear case
    switch flg
        case 'complete'
            % Setup for longitudinal and lateral plot
    
    
            Vw = WindMap(windpx,windpy,x);
            y = CompleteDynNoLin_Out(x',Vw);
            y = y';                                 %
            f_add = CalcForce(t,x,x_add,x_stop,y,AC);           % Calculate Forces
            [errs,tmpw] = errCalc(t,x,x_stop,x_add,y_way,windpx,windpy,GEO);         % Error Calculation
            %% Variables vs Time
            pvec = [t,x,x_add,y,f_add,errs];                    % Plot Vector
            % [ t,Va,ga,h,m,CL,T,VD,psi,phi,p,mu,l,              1:13
            %   IDl,IASd,hdotc,Kh,Vd,CL,T,Roll,PhiD,PsiD,dR     14:24
            %   IAS,M,h,hdot,psi,p,phi,Vg,psiGT]                25:33
            %   D,L,W*sin(g),W*cos(g),CLp(V),CLp(hdot),         34:39
            %   CLb(V),CLb(hdot),Tp(V),Tb(V) ]                  40:43
            %   IASerr,Merr,herr,hdoterr,Psierr,Phierr,         44:49
            %   dR,dst]                                         50:51
    
            % Commanded Variables and Forces
            figS = [1,1,14,nan; 1,1,18,16; 1,1,19,20; ...
                1,1,22,23; 1,1,21,nan];
    
            % State Variables
            temp = [ones( length(x(1,:)),1 )*2,...
                ones( length(x(1,:)),1 ),[2:1+length(x(1,:))]'];
            figS = fillMatrix(figS,temp);
    
            % COmmanded Variables Only
            temp = [ones(4,1)*3,ones(4,1),...
                [14,nan(1,3);28,16,nan(1,2);25,18,nan(1,2); ...
                10,22,9,23] ] ;
            figS = fillMatrix(figS,temp);
            % Forces
            temp = [ones(3,1)*4,ones(3,1),...
                [34:37,20,nan;19,38:41,6;20,42,43,7,nan,nan] ];
            figS = fillMatrix(figS,temp);
            % Integral Vd debug
            temp = [ones(2,1)*5,ones(2,1),[8,nan;15,17] ];
            figS = fillMatrix(figS,temp);
            % Errors
            temp = [ones(3,1)*6,ones(3,1),[44,46;50,24;51,nan]];
            figS = fillMatrix(figS,temp);
            % Kh Errors ID
            temp = [ones(3,1)*7,ones(3,1),[17,16;19,20;14,nan]];
            figS = fillMatrix(figS,temp);



            dms = size(figS);
            LEG = repmat({'-'},dms(1),dms(2));
            LEG{4,1} = '$\Phi_D$'; LEG{4,2} = '$\Psi_D$';
            LEG{19,2} = '$\dot{h}_D$'; LEG{19,1} = '$\dot{h}$';
            LEG{20,2} = '$V_D$'; LEG{20,1} = 'V';
            LEG{21,2} = '$\Phi_D$'; LEG{21,1} = '$\Phi$';
            LEG{21,4} = '$\Psi_D$'; LEG{21,3} = '$\Psi$';
            LEG{22,1} = 'D'; LEG{22,2} = 'L'; LEG{22,3} = 'W $\cos(\gamma_a)$';
            LEG{22,4} = 'W $\sin(\gamma_a)$'; LEG{22,5} = 'T';
            LEG{23,1} = 'C$_L$'; LEG{23,2} = 'C$_{L_p}^V$'; LEG{23,3} = 'C$_{L_p}^h$';
            LEG{23,4} = 'C$_{L_b}^V$'; LEG{23,5} = 'C$_{L_b}^h$'; LEG{23,6} = 'I$_L$';
            LEG{24,1} = 'T'; LEG{24,2} = 'T$_p^V$'; LEG{24,3} = 'T$_b^V$';
            LEG{24,4} = 'I$_T$';
            LEG{28,1} = '$\Delta$ R'; LEG{28,2} = '$\Delta R_{lat}$';

            XLAB = repmat({'t [s]'},dms(1));
            YLAB = {'ID','$V_c$ [kts] | $\dot{h}_d$ [ft/min]','C$_L$ [-] | T [N]',...
                '$\Phi_c$, $\Psi_c$ [rad]','p [rad/s]','V [m/s]','$\gamma_a$ [rad]',...
                'h [m]','M [kg]','I$_{C_L}$ [-]','I$_T$ [N]','I$_{V_D}$ [??]',...
                '$\Psi$ [rad]','$\Phi$ [rad]','p [rad/s]',...
                '$\mu$ [rad]','l [rad]','ID','$\dot{h}$ [ft/min]','V [kts]',...
                '$\Psi,\Phi$ [rad]','F [N]','F [N]','F [N]','-','-',...
                '$e_V$ [kts] | $e_h$ [ft]','$\Delta R$ [m]','-',...
                '$K_{\dot{h}}$ [ft/min] | $\dot{h}_d$','C$_L$ [-] | T [N]','ID'};
            LINST = repmat( {'-'},dms(1),dms(2) );
            LINST{2,2} = 'r-'; LINST{3,2} = 'r-'; LINST{27,2} = 'r-';
            LINST{30,2} = 'r-'; LINST{31,2} = 'r-';
            FIGT = {'ID','Longitudinal Commanded Values','C$_L$ and T',...
                '$\Phi_C$ and $\Psi_C$','Roll','V','$\gamma_a$','h','m',...
                'I$_{C_L}$','I$_T$','I$_{V_D}$','$\Psi$','$\Phi$','p',...
                '$\mu$','l','ID','$\dot{h}_c$ vs $\dot{h}$','$V_D$ vs $V_{IAS}$',...
                '$\Psi_D$ vs $\Psi$','Forces','Breakdown of Lift',...
                'Breakdown of Thrust','$x_7$','IAS and $V_D$','V and h error',...
                'Distance Error','Terminator Function'};
    
            NFIG = {'Commanded Values','State Variables','Commanded Variables',...
                'Forces','Integral VD Debug','Errors','Kh'};

            pltdt = DataPlot( pvec );
            for iG = 1:length( figS(:,1) )
                yidx = figS( iG,~isnan( figS(iG,:) ) ); yidx = yidx(3:end);
                pltdt = pltdt.definePlot( figS(iG,2),yidx,figS(iG,1),...
                    'xlabel',XLAB{iG},'ylabel',YLAB{iG},'linestyle',...
                    LINST(iG,:),'grid','minor','legend',LEG(iG,:) );
                
                % PlotResComp(t,x,x_aux,x_debug,store_way,figS,GEO,...
                %     AC,FIGT,LEG,NFIG);  % x_debug is y_way
            end
            pltdt.PlotPerImag( [3,1],NFIG,NFIG,'cartesian',false)
            %% Error Plane
            % herr vs IASerr
            nway = length(tmpw)-1;
            figS = nan( nway,5 );
            pvec = nan( length(errs(:,1)),4*nway ); % number of time steps x number of waypoints 
            IMSAV = {'Vh_error'};
            % Store IAS-V as different columns for each waypoint
            for i = 2:nway+1
                pvec( 1:tmpw(i)-tmpw(i-1),2*i-3 ) = ...
                    errs( tmpw(i-1)+1:tmpw(i),1 ); % IAS error 2*(i-1) - 1
                pvec( 1:tmpw(i)-tmpw(i-1),2*i-2 ) = ...
                    errs( tmpw(i-1)+1:tmpw(i),3 ); % h error 2*(i-1)

                pvec( 1:11,2*i-3+2*nway ) = ...
                    [-1;1;nan;-1;1;nan;-1;-1;nan;1;1]*x_stop( i,2 );   % IAS Bounds
                pvec( 1:11,2*i-2+2*nway ) = ...
                    flip([-1;1;nan;-1;1;nan;-1;-1;nan;1;1])*x_stop( i,3 ); % h Bound

                figS(i-1,1) = 1; figS(i-1,2:5) = [ 2*i-3 , 2*i-2, ...
                    2*i-3 + 2*nway,2*i-2 + 2*nway];
                
            end

            plter = DataPlot( pvec );
            
            for i = 1:nway
                %yidx = figS( i,~isnan( figS(i,: ) ) );
               plter = plter.definePlot( figS(i,[2,4]),figS(i,[3,5]),figS(i,1),...
                   'grid','minor','linestyle',{'-'},'xlabel','$e_V$',...
                   'ylabel','$e_h$','title',...
                   ['Waypoint ',num2str(i-1),' $\rightarrow$ ',num2str(i)]);
            end
            [~,plter] = plter.PlotPerImag( [2,2],IMSAV,IMSAV,'cartesian',false );
            % Reverting the axes
            for i = 1:length(plter.axs)
                plter.axs{i}.XDir = "reverse";
                plter.axs{i}.YDir = "reverse";
            end
            %% Trajectory
            % Plot on Mercator Map and Plot in ECEF Coordinates
            fig = figure('Name','Trajectory on Mercator Map');
            set( fig, 'Units', 'normalized', ...
                        'Position', [0.1,0.1,0.8,0.8] ) ;

            gx = geoaxes('Parent',fig);                     % Geoaxes object definiton for tiledlayout
            p = geoplot(gx,x(:,11)*180/pi,x(:,12)*180/pi);      % Trajectory
            p.LineWidth = 1.5; p.Color = [0.15,0.15,0.15];
            hold(gx,'on');
            plotMap(y_way,GEO,gx)
            set( gx,'FontSize',16,'FontName','Times New Roman' );
            gx.Basemap = 'darkwater';
            
            idx1 = 1500; idx2 = 1624;
            idx1 = 1; idx2 = 500;
            plttrj = DataPlot( [t(idx1:idx2),x(idx1:idx2,:)] );

            plttrj.trajectoryPlot('ECEF',[12,13,4,9,3,10,1],GEO,AC);
            plttrj.trajectoryPlot('NEDi',[12,13,4,9,3,10,1],GEO,AC);
            %% Plot Waypoints
            %plter.trajectoryPlot('ECEF','mu',x(:,12),'lat',x(:,13),'h',x(:,4),...
            %    'psi',x(:,9),'theta',x(:,3),'phi',x(:,10) );
            %plter.trajectoryPlot('NEDi','mu',x(:,12),'lat',x(:,13),'h',x(:,4),...
            %    'psi',x(:,9),'theta',x(:,3),'phi',x(:,10) );
    end

end

function figS = fillMatrix(figS,temp)
    ls = length(figS(1,:)); lt = length(temp(1,:));
    if ls > lt
        figS = [figS;temp,nan( length(temp(:,1)),ls-lt )];
    elseif ls < lt
        figS = [figS,nan( length(figS(:,1)),lt-ls );temp,];
    else
        figS = [figS;temp];
    end
end


function [fig,ax] = plotfigs(AC,t,x,x_aux,x_debug)
%
%   INPUT
%   - x_aux: [ID,VIAS,hdotc,Kc,Vd,CL,T]

    % Output Variables and Commanded Variables
    i = 1;                                          % figure index
    y = LongDynNoLin_Out(x'); y = y';                        % output vector
    fig(i) = figure('Name','Output and Commanded Variables');
    % fig 1: hdot,V,CL,T
    plotvec = [y(:,1),y(:,4),x_aux(:,6:7)];             % Vector used to plot results [VIAS,hdot,CL,T]
    plotvec_aux = [x_aux(:,5),x_aux(:,3)];              % [Vc,hdotc,CLc,Tc]
    nCase = length(plotvec(1,:));                       % Number of subplots
    TIT = {'Actual vs Commanded IAS','Actual vs Commanded hdot','CL','T'};
    for j = 1:nCase
        ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
        hold(ax(i,j),'on'); title(ax(i,j),TIT{j});
        plot( ax(i,j),t,plotvec(:,j) );
        if j < 3
            plot( ax(i,j),t,plotvec_aux(:,j),'--r' );
        end
        % if j == 3
        %     plot( ax(i,j),t,x(:,5),'--r')
        %     err = x_aux(:,5) - y(:,1);
        %     for kk = 1:length(t)
        %         CL1(kk) = AC.Kp(1,1,u)*err(kk,1); % CL
        %         CL2(kk) = AC.Kp(1,4,u)*err(kk,4); 
        %         CL3(kk) = -AC.Kb(1,1,u)*y(kk,1);
        %         CL4(kk) = -AC.Kb(1,4,u)*y(kk,4); 
        % 
        % 
        %     end
        %     plot( ax(i,j),t,x(:,5),'--m')
        %     plot( ax(i,j),t,CL1,'--b')
        %     plot( ax(i,j),t,CL2,'-.b')
        %     plot( ax(i,j),t,CL3,':b')
        %     plot( ax(i,j),t,CL4,'-b')
        %     legend(ax(i,j),{'C_L','I_c','Kp_c','Kp_k','Kb_c','Kb_h'})
        % end
    end
    % for j = 3:nCase
    %     ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
    %     hold(ax(i,j),'on'); title(ax(i,j),TIT{j});
    %     plot( ax(i,j),t,plotvec(:,j) );
    %     if nargin > 4
    %         plot( ax(i,j),x_debug(:,1),x_debug(:,3+j),'--m' );
    %     end

    % end
    % State Variables
    i = 2;  fig(i) = figure('Name','1');
    fig(i) = figure('Name','State Variables');
    nCase = 6; TIT = {'V','\gamma_a','h','m','I_c','I_T'};
    for j = 1:nCase
        ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
        hold(ax(i,j),'on'); title(ax(i,j),TIT{j});
        plot( t,x(:,j) );

    end
    % Forces
    i = 3; j = 1;
    fig(i) = figure('Name','Forces');
    qs = t(:)*nan; L = qs; D = qs; W = [qs,qs]; Re = qs;
    for in = 1:length(x(:,1))
        [T, a, P, rho] = atmosisa(x(in,3));
        qs(in) = 0.5*rho(:)*x(in,1).^2; Re(in) = AC.ReCalc(x(in,3),y(in,2));
        D(in) = qs(in)*AC.Sw*AC.polar(y(in,2),Re(in),x_aux(in,6));
        L(in) = qs(in)*AC.Sw*x_aux(in,6);
        W(in,1) = 9.81*x(in,4)*sin(x(in,2));
        W(in,2) = 9.81*x(in,4)*cos(x(in,2));
    end
    plotvec = [L,D,W,x_aux(:,7)]; nCase = length(plotvec(1,:));
    ax(i,j) = subplot(2,1,j,'Parent',fig(i) ); hold(ax(i,j),'on');
    %ax(i,j) = axes('Parent',fig(i));  title(ax(i,j),'Forces');
    grid(ax(i,j),'minor');
    colors = [
        0,    0,    1;    % Blu scuro
        1,    0,    0;    % Rosso
        0.5,  1,    0;    % Verde lime
        0,    1,    1;    % Ciano
        1,    0,    1;    % Magenta
        1,    0.647, 0;   % Arancione
        1,    1,    0     % Giallo
        ];
    NM = {'L','D','Wsin(\gamma)','Wcos(\gamma)','T'};
    for k = 1:nCase
        %ax(i,1) = subplot(nCase,1,j,'Parent',fig(i));
        plot( ax(i,j),t,plotvec(:,k), 'LineWidth', 1.2,  'Color', ...
            colors(k,:), 'LineStyle', '-', ...
            'DisplayName', NM{k} );
    end
    
    legend(ax(i,1));
    nCase = 2; j = 2; NM = {'Re*1e7','M'};
    ax(i,j) = subplot(2,1,j,'Parent',fig(i) ); hold(ax(i,j),'on');
    plotvec = [Re*1e-7,y(:,2)];
    for k = 1:nCase
        %ax(i,1) = subplot(nCase,1,j,'Parent',fig(i));
        plot( ax(i,j),t,plotvec(:,k), 'LineWidth', 1.2,  'Color', ...
            colors(k,:), 'LineStyle', '-', ...
            'DisplayName', NM{k} );
    end
    legend(ax(i,j));
    % IDs e Temp
    i = 4; j = 1;
    fig(i) = figure('Name','Vd');
    plotvec = [x_aux(:,1),x(:,7),x(:,7)*nan,x_aux(:,5),x_aux(:,4)];    %  [ID,x7,x*,Vd] [x_debug(:,9:10),x_debug(:,3),x_debug(:,6)];      % Vector used to plot results [VIAS,hdot,CL,T,Vc,hdotc]
    plotvec_aux = [x_aux(:,1)*nan,x(:,7)*nan,x(:,7)*nan ,x_aux(:,2)...
        ,x_aux(:,2)*nan]; % [ no,no,no,VIAS]
    nCase = length(plotvec(1,:)); TIT = {'ID','x_7','x^*','Vd - Vc','K_h'};
    for j = 1:nCase
        ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
        hold(ax(i,j),'on'); title(ax(i,j),TIT{j});
        plot( ax(i,j),t,plotvec(:,j) );
        plot( ax(i,j),t,plotvec_aux(:,j),'--r' );
    end
    
    % for j = 1:nCase
    %     ax(i,j+2) = subplot(nCase,1,j+2,'Parent',fig(i));
    %     hold(ax(i,j+2),'on'); title(ax(i,j+2),TIT{j+2});
    %     plot( ax(i,j+2),plotvec(:,1),plotvec(:,j) );
    %     plot( ax(i,j+2),plotvec_aux(:,1),plotvec(:,j+1),'--b' );
    %     j = j + 1;
    % end

end

function [fig,ax] = IAShplot(AC,t,x,x_aux,x_debug,store_way)
    % h-IAS error plane
    nfigpp = 4; % Number of subplots per page
    nrow = 2;
    ncol = 2;
    i = 1;                                          % figure index
    y = LongDynNoLin_Out(x'); y = y';                        % output vector
    store_way = store_way(2:end,:);                             % Removes the starting point
    store_way(:,8:11) = LongDynNoLin_Out(store_way(:,4:7)')';
    fig(i) = figure('Name','h-V error plane');

    n_way = length( store_way(:,1));
    nfig = ceil(n_way/nfigpp);
    fig(nfig+1) = figure('Name','Responses');
    %axS = axes('Parent',fig(nfig+1)); hold(axS,'on');
    axS(1) = subplot(3,1,1,'Parent',fig(nfig+1)); hold(axS(1),'on');
    axS(2) = subplot(3,1,2,'Parent',fig(nfig+1)); hold(axS(2),'on');
    axS(3) = subplot(3,1,3,'Parent',fig(nfig+1)); hold(axS(3),'on');
    ip = 1; lst = 0;
    for ifi = 1:nfig
        fig(ifi) = figure('Name',['h-V error plane - ',num2str(ifi)]);
        while ip-nfigpp*(ifi-1) < nfigpp+1 && ip < n_way + 1
            idf = ip-nfigpp*(ifi-1);
            ax(ip) = subplot(nrow,ncol,idf,'Parent',fig(ifi)); hold(ax(ip),'on')
            if ip > 1
                idxs =  all([t>store_way(ip-1,1)  , t<store_way(ip,1)],2) ;
            else
                idxs = ~(t>store_way(1,1)) ;
            end
            yac = y( idxs ,: ); Vmax = max( abs(yac(:,1)) ); hmax = max( abs(yac(:,3)) );
            errs = store_way(ip,8:11)' - yac'; errs = errs';
            errs(:,1) = x_aux(idxs,5) - yac(:,1); 
            errs(:,4) = x_aux(idxs,3) - yac(:,4);

            

            Vmax = max( abs(errs(:,1)) ); hmax = max( abs(errs(:,3)) );
            % Grid plotting
            plot( ax(ip),[-Vmax,Vmax],[1,1]*store_way(ip,3)  )
            plot( ax(ip),[-Vmax,Vmax],[-1,-1]*store_way(ip,3)  )
            plot( ax(ip),[-1,-1]*store_way(ip,2),[-hmax,hmax]  )
            plot( ax(ip),[1,1]*store_way(ip,2),[-hmax,hmax]  )
            axis(ax(ip),[-Vmax,Vmax,-hmax,hmax]);
            % Path
            plot( ax(ip),-errs(1,1),-errs(1,3),'or' );% Red dot to indicate starting point
            plot( ax(ip),-errs(:,1),-errs(:,3) );
            
            plot(ax(ip),0,0,'MarkerSize',7,'Marker','diamond',LineWidth=0.7)
            teta = atan2( errs(:,3),errs(:,1) );
            fct =  (errs(:,1)/0.5).^2 + (errs(:,3)/250 ).^2 -1   ;
            
            yyaxis(axS(3),'left'); plot( axS(3),t(idxs),errs(:,1),'r')
            yyaxis(axS(3),'right'); plot( axS(3),t(idxs),errs(:,3),'g' );

            legend(axS(3),{'Speed','h_d'})

            CL1 = t(idxs)*nan; CL2 = t(idxs)*nan; 
            CL3 = t(idxs)*nan; CL4 = t(idxs)*nan;
            T1 = t(idxs)*nan; T3 = t(idxs)*nan;
            for kk = 1:length(yac(:,1))
                u = 1; T1(kk) = AC.Kp(2,1,u)*errs(kk,1)*nan;
                %T2(kk) = AC.Ki(2,1,u)*errs(kk,1)*nan;
                T3(kk) = AC.Kb(2,1,u)*yac(kk,1)*nan;
                switch x_aux(lst+kk,1)
                    case 3
                        u = 2;
                    case 6
                        u = 2;
                    case 7
                        u = 3;
                        T1(kk) = AC.Kp(2,1,u)*errs(kk,1);
                        %T2(kk) = AC.Ki(2,1,u)*errs(kk,1);
                        T3(kk) = AC.Kb(2,1,u)*yac(kk,1);
                    otherwise
                        u = 1;

                end
                CL1(kk) = AC.Kp(1,1,u)*errs(kk,1); % CL
                CL2(kk) = AC.Kp(1,4,u)*errs(kk,4);
                CL3(kk) = -AC.Kb(1,1,u)*yac(kk,1);
                CL4(kk) = -AC.Kb(1,4,u)*yac(kk,4);
                
                
            end
            lst = length(yac(:,1));
            plot( axS(1),t(idxs),x_aux(idxs,6),'r','LineWidth',1)
            plot( axS(1),t(idxs),x(idxs,5),'--m','LineWidth',0.5)
            plot( axS(1),t(idxs),CL1,'--g','LineWidth',0.5)
            plot( axS(1),t(idxs),CL2,'-.b')
            plot( axS(1),t(idxs),CL3,':b','LineWidth',0.5)
            plot( axS(1),t(idxs),CL4,'-.g','LineWidth',0.5)
            legend(axS(1),{'C_L','I_c','Kp_c','Kp_h','Kb_c','Kb_h'})
            
            plot( axS(2),t(idxs),x_aux(idxs,7),'r','LineWidth',1)
            plot( axS(2),t(idxs),x(idxs,6),'--m','LineWidth',0.75)
            plot( axS(2),t(idxs),T1,'--g','LineWidth',0.75)
            plot( axS(2),t(idxs),T3,':b','LineWidth',0.75)
            legend(axS(2),{'T','I_T','Kp_V','Kb_V'})

            ip = ip+1;
        end
    end

end 

function [errs,endix] = errCalc(t,x,x_stop,x_aux,y_way,windpx,windpy,GEO)
%ERRCALC: function that calculates the error
%   INPUT
%   - t: time
%   - x: state vector [Va,ga,h,m,CL,T,VD,psi,phi,p,mu,l]
%   - x_stop: [t,bounds] at which the integration is stopped 
%                       because a waypoint has been reached
%   - x_aux: [IDl,IASd,hdotc,Kh,Vd,CL,T,Roll,PhiD,PsiD,dR]
%   - y_way:
%   OUTPUT
%   - err: [IAS,M,h,hdot,Psi,Phi,dR,dst]
%   - endix: indices when the i-th waypoint has been reached
% ----------------------------------------------------------------------- %
    errs = nan(length(t),8);
    nway = length(x_stop(:,1))-1;
    k = 1; endix = nan(nway,1);
    for i = 1:nway
        while t(k)<x_stop(i+1,1)
            [errs(k,8),errs(k,7),errs(k,1:4)] = ...
                CaptureHalo(x(k,:),y_way(:,i+1),x_stop(i,2:3),windpx,windpy,GEO);
            errs(k,6) = x_aux(k,9) - x(k,9);    % Phi_err
            errs(k,5) = x_aux(k,10) -x(k,8);    % Psi_err
            
            k = k+1;
        end
        endix(i) = k-1;
    end
    endix = [1;endix];
end

function frc = CalcForce(t,x,x_aux,x_stop,y,AC)
%CALCFORCE: Function that calculates the forces during the simulation
%   INPUT
%   - t: time
%   - x: state vector [Va,ga,h,m,CL,T,VD,psi,phi,p,mu,l]
%   - x_stop: [t,bounds] at which the integration is stopped 
%                       because a waypoint has been reached
%   - x_aux: [IDl,IASd,hdotc,Kh,Vd,CL,T,Roll,PhiD,PsiD,dR]
%   - y_way:
%   OUTPUT
%   - err: [D,L,Wsin(gm),Wcos(gm),CLp,CLb,Tb,Tp]
% ----------------------------------------------------------------------- %
    qs = t(:)*nan; L = qs; D = qs; W = [qs,qs]; Re = qs;
    Tp = qs; Tb = qs; CLp = [qs,qs]; CLb = [qs,qs];
    it = 1;
    for in = 1:length(x(:,1))
        [T, a, P, rho] = atmosisa( x(in,3) );
        qs(in) = 0.5*rho(:)*x(in,1).^2; Re(in) = AC.ReCalc(x(in,3),y(in,2));
        D(in) = qs(in)*AC.Sw*AC.polar(y(in,2),Re(in),x_aux(in,6));
        L(in) = qs(in)*AC.Sw*x_aux(in,6);
        W(in,1) = 9.81*x(in,4)*sin(x(in,2));
        W(in,2) = 9.81*x(in,4)*cos(x(in,2));

        if t(in) > x_stop(it+1,1) || t(in) < x_stop(it,1)

            it = it + 1;

        end

        % Longitudinal Zone
        switch x_aux(in,1)
            case 3
                u = 2;
            case 6
                u = 2;
            case 7
                u = 3;
                Tp(in) = AC.Kp(2,1,u)*( x_aux(in,5) - y(in,1) ); % Vd - V
                %T2(kk) = AC.Ki(2,1,u)*errs(kk,1);
                Tb(in) = AC.Kb(2,1,u)*y(in,1);
            otherwise
                u = 1;
        end

        CLp(in,1) = AC.Kp(1,1,u)*( x_aux(in,5) - y(in,1) ); % CL
        CLp(in,2) = AC.Kp(1,4,u)*( x_aux(in,3) - y(in,4) );
        CLb(in,1) = -AC.Kb(1,1,u)*( x_aux(in,5) - y(in,1) );
        CLb(in,1) = -AC.Kb(1,4,u)*( x_aux(in,3) - y(in,4) );
    end
    frc = [D,L,W,CLp,CLb,Tp,Tb];
end