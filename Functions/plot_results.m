function plot_results(AC,flg,t,x,x_aux,x_debug,tl,xl,A,C,xref,store_way,GEO)
%PLOT_RESULTS Function that plots the results of the integration
%   INPUT
%   x - longitudinal state vector [V,ga,h,m]
%   x_aux - additional daata taken from ode's OutputFcn [ID,Vc,hdotc,Kc,Vd,CL,T]
%   tl,xl - solution vectors for the linear case
    switch flg
        case 'ctrl'
        % Commanded valeu case
            plotfigs(AC,t,x,x_aux,x_debug);
        
        case 'cfr'
            % Confronts linear and nonlinear cases
            [fig,ax] = plotfigs(AC,t,x,x_aux,x_debug);
            i = 1;
            y = C*xl'; y = y';
            yref = LongDynNoLin_Out(xref'); yref = yref';                        % output vector
            plotvec = [y(:,1)+yref(1),y(:,4)+yref(4),...
                xl(:,4)+xref(end-1),xl(:,5)+xref(end)];      % Vector used to plot results [VIAS,hdot,CL,T,Vc,hdotc]
            nCase = 4;
            for j = 1:nCase
                ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
                hold(ax(i,j),'on');
                plot( ax(i,j),tl,plotvec(:,j),'.-b' );
                %plot( ax(i,j),t,plotvec(:,nCase+j),'--r' );
            end
            i = 2; nCase = 3;
            for j = 1:nCase
                plot( ax(i,j),tl,xl(:,j)+xref(j),'.-b'  );
            end
            % Plots the roots
            lam = eig(A);
            i = 3; fig(i) = figure('Name','Linearized System Roots');
            j = 3; ax(i,j) = axes('Parent',fig(i));
            plot( ax(i,j),real(lam),imag(lam),'diamond','MarkerSize', 8, ...            % dimensione del marker
            'MarkerEdgeColor', [0 0 0], ...% colore bordo (nero)
            'MarkerFaceColor', [0 0.5 0], ...% colore interno (rosso)
            'LineWidth', 1.5);
        case 'waypoint'
            plotfigs(AC,t,x,x_aux,x_debug);
            IAShplot(AC,t,x,x_aux,x_debug,store_way);
        case 'complete'
        % Setup for longitudinal and lateral plot

            FIGT = {'ID','Longitudinal Commanded Values','CL and T',...
                '\Phi_C and \Psi_C','Roll','V','\gamma_a','h','m',...
                'I_{CL}','I_T','I_{V_D}','\Psi','\Phi','p',...
                'mu','l','ID','\dot{h}_c vs \dot{h}','V_D vs V_{IAS}',...
                '\Psi_D vs \Psi','Forces','Breakdown of Lift',...
                'Brakdown of Thrust','x_7','IAS and V_D','V and h error',...
                'Distance Error','Terminator Function'};
            LEG = repmat({'-'},30,6);
            LEG{4,1} = '\Phi_D'; LEG{4,2} = '\Psi_D';
            LEG{19,2} = '\dot{h}_D'; LEG{19,1} = '\dot{h}'; 
            LEG{20,2} = 'V_D'; LEG{20,1} = 'V';
            LEG{21,2} = '\Phi_D'; LEG{21,1} = '\Phi';
            LEG{21,4} = '\Psi_D'; LEG{21,3} = '\Psi';
            LEG{22,1} = 'D'; LEG{22,2} = 'L'; LEG{22,3} = 'W cos(\gamma_a)';
            LEG{22,4} = 'W sin(\gamma_a)'; LEG{22,5} = 'T';
            LEG{23,1} = 'CL'; LEG{23,2} = 'CL_p^V'; LEG{23,3} = 'CL_p^h';
            LEG{23,4} = 'CL_b^V'; LEG{23,5} = 'CL_b^h'; LEG{23,6} = 'I_L';
            LEG{24,1} = 'T'; LEG{24,2} = 'T_p^V'; LEG{24,3} = 'T_b^V';
            LEG{24,4} = 'I_T';
            LEG{28,1} = '\Delta R'; LEG{28,2} = '\Delta R_{lat}';
            NFIG = {'Commanded Values','State Variables','Commanded Variables',...
                'Forces','Integral VD Debug','Errors'};

            % Commanded Variables and Forces
            figS = [1,1,14,nan; 1,1,18,216; 1,1,19,220; ...
                1,1,22,123; 1,1,21,nan];

            % State Variables
            temp = [ones( length(x(1,:)),1 )*2,...
                ones( length(x(1,:)),1 ),[2:1+length(x(1,:))]'];
            figS = fillMatrix(figS,temp);
                      
            % COmmanded Variables Only
            temp = [ones(4,1)*3,ones(4,1),...
                [14,nan(1,3);28,116,nan(1,2);25,118,nan(1,2); ...
                10,122,9,123] ] ;
            figS = fillMatrix(figS,temp);
            % Forces
            temp = [ones(3,1)*4,ones(3,1),...
                [34:37,20,nan;19,138:141,106;20,142,143,107,nan,nan] ];
            figS = fillMatrix(figS,temp);
            % Integral Vd debug
           temp = [ones(2,1)*5,ones(2,1),[8,nan;15,17] ];
           figS = fillMatrix(figS,temp);
            % Errors
            temp = [ones(3,1)*6,ones(3,1),[44,246;50,124;51,nan]];
            figS = fillMatrix(figS,temp);
            
            PlotResComp(t,x,x_aux,x_debug,store_way,figS,GEO,...
                AC,FIGT,LEG,NFIG);  % x_debug is y_way
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

function frc = CalcForce(t,x,y,AC)
%CalcForce: Function that calculates the forces during the simulation
    qs = t(:)*nan; L = qs; D = qs; W = [qs,qs]; Re = qs;
    Tp = qs; Tb = qs; CLp = [qs,qs]; CLb = [qs,qs];
    for in = 1:length(x(:,1))
        [T, a, P, rho] = atmosisa( x(in,3) );
        qs(in) = 0.5*rho(:)*x(in,1).^2; Re(in) = AC.ReCalc(x(in,3),y(in,2));
        D(in) = qs(in)*AC.Sw*AC.polar(y(in,2),Re(in),x_aux(in,6));
        L(in) = qs(in)*AC.Sw*x_aux(in,6);
        W(in,1) = 9.81*x(in,4)*sin(x(in,2));
        W(in,2) = 9.81*x(in,4)*cos(x(in,2));

        if t(in) < store_way(it+1,1) && t(in) > store_way(it,1)

        else
            it = it + 1;

        end
        u = 1;
        % Longitudinal Zone
        switch x_aux(lst+kk,1)
            case 3
                u = 2;
            case 6
                u = 2;
            case 7
                u = 3;
                Tp(in) = AC.Kp(2,1,u)*( y_way(it,1) - y(in,1) );
                %T2(kk) = AC.Ki(2,1,u)*errs(kk,1);
                Tb(in) = AC.Kb(2,1,u)*y(in,1);
            otherwise
                u = 1;
        end
        CLp(in,1) = AC.Kp(1,1,u)*( y_way(it,1) - y(in,1) ); % CL
        CLp(in,2) = AC.Kp(1,4,u)*( y_way(it,4) - y(in,4) );
        CLb(in,1) = -AC.Kb(1,1,u)*( y_way(it,1) - y(in,1) );
        CLb(in,1) = -AC.Kb(1,4,u)*( y_way(it,4) - y(in,4) );
    end
    frc = [D,L,Wsin,Wcos,CLp,CLb,Tp,Lb];
end