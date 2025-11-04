function plot_results(AC,flg,t,x,x_aux,x_debug,tl,xl,A,C,xref,store_way)
%PLOT_RESULTS Function that plots the results of the integration
%   INPUT
%   x - longitudinal state vector [V,ga,h,m]
%   x_aux - additional daata taken from ode's OutputFcn [t,ID,Vc,hdotc,Kc,CL,T]
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
        
    end

end

function [fig,ax] = plotfigs(AC,t,x,x_aux,x_debug)
    % Output Variables and Commanded Variables
    i = 1;                                          % figure index
    y = LongDynNoLin_Out(x'); y = y';                        % output vector
    fig(i) = figure('Name','Output and Commanded Variables');
    % fig 1: hdot,V,CL,T
    plotvec = [y(:,1),y(:,4),x_aux(:,6:7),x_aux(:,3:4)];      % Vector used to plot results [VIAS,hdot,CL,T,Vc,hdotc]
    nCase = length(plotvec(1,:)) - 2;               % Number of subplots
    TIT = {'Actual vs Commanded IAS','Actual vs Commanded hdot','CL','T'};
    for j = 1:2
        ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
        hold(ax(i,j),'on'); title(ax(i,j),TIT{j});
        plot( ax(i,j),t,plotvec(:,j) );
        plot( ax(i,j),t,plotvec(:,nCase+j),'--r' );
    end
    for j = 3:nCase
        ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
        hold(ax(i,j),'on'); title(ax(i,j),TIT{j});
        plot( ax(i,j),t,plotvec(:,j) );
        if nargin > 4
            plot( ax(i,j),x_debug(:,1),x_debug(:,3+j),'--m' );
        end

    end
    % State Variables
    i = 2;  fig(i) = figure('Name','1');
    fig(i) = figure('Name','State Variables');
    nCase = 5; TIT = {'V','\gamma_a','h','m','I_c'};
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
    ip = 1;
    for ifi = 1:nfig
        fig(ifi) = figure('Name',['h-V error plane - ',num2str(ifi)]);
        while ip-nfigpp*(ifi-1) < nfigpp+1 && ip < n_way + 1
            idf = ip-nfigpp*(ifi-1);
            ax(ip) = subplot(nrow,ncol,idf,'Parent',fig(ifi)); hold(ax(ip),'on')
            if ip > 1
                idxs = all( t>store_way(ip-1,1), t<store_way(ip,1) );
            else
                idxs = t<store_way(1,1) ;
            end
            yac = y( idxs ,: ); Vmax = max( abs(yac(:,1)) ); hmax = max( abs(yac(:,3)) );
            errs = store_way(ip,8:11)' - yac'; errs = errs';
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

            ip = ip+1;
        end
    end
end