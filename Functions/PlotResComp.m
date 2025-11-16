function  PlotResComp(t,x,x_add,y_way,x_stop,figS,GEO,AC,figT,Leg,NFIG)
%UNTITLED Summary of this function goes here
%   INPUT
%   - x: [Va.ga,h,m,CL,T,Vd,Psi,Phi,p,mu,lng]
%   - x_add : [ID,IASd,hdotc,Kh,Vd,CL,T,Roll,PhiD,PsiD,dR]
%   - x_stop: [t,bounds] at which the integration is stopped 
%            because a waypoint has been reached 
%   - figS: matrix containng data required to plot
%           [figID,IDX1,IDX(Y1),IDX(y2),,,]
%   - figT: Cell containing title for all the plots

%% PLOTS DATA
    Vw = WindMap(x);
    y = CompleteDynNoLin_Out(x',Vw);
    y = y';                                 %
    f_add = CalcForce(t,x,x_add,x_stop,y,AC);     % Calculate Forces
    errs = errCalc(t,x,x_stop,x_add,y_way,GEO);            % Error Calculation
    pvec = [t,x,x_add,y,f_add,errs];                     % Plot Vector 
% [ t,Va,ga,h,m,CL,T,VD,psi,phi,p,mu,l,              1:13
%   IDl,IASd,hdotc,Kh,Vd,CL,T,Roll,PhiD,PsiD,dR     14:24
%   IAS,M,h,hdot,psi,p,phi,Vg,psiGT]                25:33
%   D,L,W*sin(g),W*cos(g),CLp(V),CLp(hdot),         34:39
%   CLb(V),CLb(hdot),Tp(V),Tb(V) ]                  40:43
%   IASerr,Merr,herr,hdoterr,Psierr,Phierr,         44:49
%   dR,dst]                                         50:51
    nsbp = 5;                               % Number of subplots per figure
    j  = 1;                                 % Plot Counter
    kk = 1;                                 % ax counter
    k2 = 1;
    cnm = 1;
    nfig = 6;
    ifa = 1;
    for ifi = 1:nfig
        % Figures

        nplt = 0;                           % Plot number definition
        while figS(k2,1) == ifi
            nplt = nplt + 1;
            k2 = k2 + 1;
            if k2 > length( figS(:,1) )
                break
            end
        end

        nfgs = ceil(nplt/nsbp);                                     % Number of figures desired
        for j = 1:nfgs
            % Define Figure
            fig(ifa) = figure('Name',[NFIG{ifi},' - ',num2str(j)]);  % Defines figures required for having no more than sbop subfigures per figure to avoid cluttering
            % Define Axes
            nsplot = min(nplt - nsbp*(j-1),nsbp);                   % Number of subplot per figure
            
            for k = 1:nsplot
                cax = k + nsbp*(j-1);
                
                idxs = figS( kk,~isnan(figS(kk,:)) );               % Save indices of columns to plot
                idxs = idxs(2:end);                                 % Skips the figure ID
                ax(ifi,cax) = ...                                   % [figure,ax(1) .. ax(n) ]
                    subplot(nsplot,1,k,'Parent',fig(ifa));
                hold(ax(ifi,cax),'on');
                legf = true;
                for l = 2:length(idxs)                              % Cycle for lines to plot
                    tmpf = plotting(pvec,idxs(1),idxs(l),ax(ifi,cax),  ...
                        Leg{cnm,l-1},l);   
                    legf = legf && tmpf;
                end
                title( ax(ifi,cax),figT{cnm} );
                if legf
                    legend( ax(ifi,cax) );
                end
                kk = kk+1; cnm = cnm + 1;
            end

            ifa = ifa+1;
        end
    end
%% TRAJECTORY PLOT
% Plot on Mercator Map and Plot in ECEF Coordinates
    fig(ifa) = figure('Name','Trajectory on Mercator Map');
    map = tiledlayout(2,1);                         % Define TiledLayout
    map.Parent = fig(ifa);                          % Sets TiledLayout Parent to current figure
    
    gx(1) = geoaxes('Parent',map);                     % Geoaxes object definiton for tiledlayout
    geoplot(gx(1),x(:,11)*180/pi,x(:,12)*180/pi);      % Trajectory
    hold(gx,'on');
    plotMap(y_way,GEO,gx)
    %gx(2) = geoaxes('Parent',map);
    %geoplot3(gx(2),x(:,11)*180/pi,x(:,12)*180/pi,x(:,3))
    % Plot Waypoints

    axG = nexttile();                               % Cretes New Axis
    %% Trajectory in ECEF with Origin to the Earth Center
    figG = figure();
    axG = axes('Parent',figG);
    % Plot on ECEF map 
    Rgt = GEO.LatLon2Vec(x(:,11),x(:,12),0);        % Ground Track Trajectory
    Rs = GEO.LatLon2Vec(x(:,11),x(:,12),x(:,3));    % Trajectory in ECEF coordinates
    Rs = Rs'; Rgt = Rgt';                           % [X,Y,Z]
  
    plot3(axG,Rs(:,1),Rs(:,2),Rs(:,3)); hold(axG,'on');                % Trajectory Plot
    plot3(axG,Rgt(:,1),Rgt(:,2),Rgt(:,3),'--k')     % Ground Track
    
    
    % Plot Waypoints
    %%Rw = wayid(); FINIREEEE AGGIUNGERE WAYPOINTS
    %%plot3(axG,Rw(:,1),Rw(:,2),Rw(:,3),'or','MarkerSize',5);              % Markers plot
    % Plot Attitude
    ptidx = choosePts(Rs);                          % Indices of points where to plot attitude
    % NED and BODY Systems
    cls = eye(3);
    vers = eye(3);
    scl = 10e2;
    for i = 1:3
        
        
        for ik = ptidx
            Us = GEO.NED2DIS(vers(:,i),'N2E',x(ik,11),x(ik,12));
            quiver3(axG,Rgt(ik,1),Rgt(ik,2),Rgt(ik,3),...
                Us(1)*scl,Us(2)*scl,Us(3)*scl,'LineWidth',1.5,'Color',cls(i,:) );

            Us2 = AC.body2NED(vers(:,i),'B2N',x(ik,8),x(ik,2),x(ik,9));     % Body in NED
            Us2 = GEO.NED2DIS(Us2,'N2E',x(ik,11),x(ik,12));                 % Body 2 ECEF
            plot3(axG,Rs(ik,1),Rs(ik,2),Rs(ik,3),'ok','MarkerSize',6)
            quiver3(axG,Rs(ik,1),Rs(ik,2),Rs(ik,3),...
                Us2(1)*scl,Us2(2)*scl,Us2(3)*scl,'LineWidth',1.5,'Color',cls(i,:) )
        end
    end
    axis( [ min([Rs(:,1);Rgt(:,1)]),max([Rs(:,1);Rgt(:,1)]),...
        min([Rs(:,2);Rgt(:,2)]),max([Rs(:,2);Rgt(:,2)]),...
        min([Rs(:,3);Rgt(:,3)]),max([Rs(:,3);Rgt(:,3)])] );
%% Trajectory in ECEF Relative to Initial Position
% Position Vector Relative to Initial Point
    %
    [Rss(:,1), Rss(:,2), Rss(:,3)] = ecef2ned(Rs(:,1),Rs(:,2),Rs(:,3),x(1,11),x(1,12),0,GEO,"radians");
    Rs = Rs - Rgt(1,:);    
    Rs = GEO.NED2DIS(Rs','E2N',x(1,11),x(1,12)); Rs = Rs';
    Rgt = Rgt - Rgt(1,:);
    Rgt = GEO.NED2DIS(Rgt','E2N',x(1,11),x(1,12)); Rgt = Rgt';

    figG(2) = figure();
    axG(2) = axes('Parent',figG(2));
    

    plot3(axG(2),Rs(:,1),Rs(:,2),Rs(:,3)); hold(axG(2),'on');                % Trajectory Plot
    plot3(axG(2),Rgt(:,1),Rgt(:,2),Rgt(:,3),'--k')     % Ground Track

    for i = 1:3


        for ik = ptidx
            Us = GEO.NED2DIS(vers(:,i),'N2E',x(ik,11),x(ik,12));            % Local NED to ECEF
            Us = GEO.NED2DIS(Us,'E2N',x(1,11),x(1,12));                     % ECEF to Starting NED
            quiver3(axG(2),Rgt(ik,1),Rgt(ik,2),Rgt(ik,3),...
                Us(1)*scl,Us(2)*scl,Us(3)*scl,'LineWidth',1.5,'Color',cls(i,:) );

            Us2 = AC.body2NED(vers(:,i),'B2N',x(ik,8),x(ik,2),x(ik,9));     % Body in Local NED
            %Us2 = GEO.NED2DIS(Us2,'N2E',x(ik,11),x(ik,12));                 % Body 2 ECEF
            plot3(axG(2),Rs(ik,1),Rs(ik,2),Rs(ik,3),'ok','MarkerSize',6)
            quiver3(axG(2),Rs(ik,1),Rs(ik,2),Rs(ik,3),...
                Us2(1)*scl,Us2(2)*scl,Us2(3)*scl,'LineWidth',1.5,'Color',cls(i,:) )
        end
    end
    axG(2).ZDir = 'reverse';
end

function frc = CalcForce(t,x,x_aux,x_stop,y,AC)
%CalcForce: Function that calculates the forces during the simulation
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

function [flg,ax] = plotting(pvec,xidx,yidx,ax,lab,k)
flg = false;
    COLS = [
        1, 0, 0;      % Rosso
        0, 1, 0;      % Verde
        0, 0, 1;      % Blu
        0, 1, 1;      % Ciano
        1, 0, 1;      % Magenta
        1, 1, 0;      % Giallo
        1, 0.5, 0;    % Arancione
        0.5, 0, 0.5   % Viola
        ];
    if yidx > 99 && yidx < 200
    % IDX between 100 and 199 means that we plot a secondary line on top of teh first
        lin = plot(ax,pvec(:,xidx),pvec(:,yidx-100),'LineWidth',1.3,...
            'Color',COLS(k,:),'LineStyle','--' );
    elseif yidx > 199 && log10(yidx) < 300
    % IDX between 200 and 299 means that we plot this line on teh secondary
    % axis
        yyaxis(ax,'right');  % Activates right side
        lin = plot(ax,pvec(:,xidx),pvec(:,yidx-200),'LineWidth',1.3,...
            'Color',COLS(k,:) );
        yyaxis(ax,'left');
    else
    % Normal plot
        lin = plot(ax,pvec(:,xidx),pvec(:,yidx),'LineWidth',1.3,...
            'Color',COLS(k,:) );
    end
    if ~isequal(lab,'-')
        lin.DisplayName = lab;
        flg = true;
    end
end

function idxs = choosePts(Rs)
%CHOOSEPTS: function that chooses the points where the reference graphics
%will be plotted based on the distance between two points
    k = 1; j = 2; tol = 500;    % TODO: aggiungere anche effetto gradienti, cioè capisce ceh deve plottare di piu se ci sono brusche variazioni di assetto
    ddes = 10000;
    idxs(1) = 1;
    for i = 2:length(Rs(:,1))
        d = norm(Rs(i,:)-Rs(k,:),2);        % Distance between teh current point and the preious plotted one
        if d-ddes > 0
            k = i;                          % Saves this index
            idxs(j) = i;
            j = j+1;
        end
    end

end

function errs = errCalc(t,x,x_stop,x_aux,y_way,GEO)
%ERRCALC: function that calculates the error
%
%   OUTPUT
%   - err: [IAS,M,h,hdot,Psi,Phi,dR,dst]
    errs = nan(length(t),8);
    nway = length(x_stop(:,1))-1;
    k = 1;
    for i = 1:nway
        while t(k)<x_stop(i+1,1)
            [errs(k,8),errs(k,7),errs(k,1:4)] = ...
                CaptureHalo(x(k,:),y_way(:,i+1),x_stop(i,2:3),GEO);
            errs(k,6) = x_aux(k,9) - x(k,9);    % Phi_err
            errs(k,5) = x_aux(k,10) -x(k,8);    % Psi_err
            
            k = k+1;
        end
    end

end

function way_out = plotWay(y_way,axgeo) % FINIREEEE
    for i = 1:nway
        switch y_way(5,i)
            case 1
            % T2F
                way_out(i,1) = y_way(7,i); % FInal Latitude
                way_out(i,2) = y_way(8,i); % Final Longitude
                way_out(i,3) = y_way(9,i); % Rx
                way_out(i,3) = y_way(9,i); % Rx
                geoplot(axgeo,y_way(7,i),y_way(8,i),'r','Marker','diamond')
            case 2
                geoplot(axgeo,y_way(7,i),y_way(8,i),'r','Marker','diamond')

        end



        % if y_way(5,i) == 1 || y_way(2,i) == 2
        %     % T2F or C2F Waypoint
        % 
        % elseif
        % 
        % else
        % 
        % end
    end
end