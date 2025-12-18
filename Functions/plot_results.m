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
            temp = [ones(2,1)*5,ones(2,1),[8,nan;15,18] ];
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
            LEG{26,1} = '$V_{IAS}(t_0)$'; LEG{26,2} = '$V_{c}(t_0)$';
            LEG{28,1} = '$\Delta$ R'; LEG{28,2} = '$\Delta R_{lat}$';

            XLAB = repmat({'t [s]'},dms(1));
            YLAB = {'ID','$V_c$ [kt] | $\dot{h}_d$ [ft/min]','C$_L$ [-] | T [N]',...
                '$\Phi_c$, $\Psi_c$ [rad]','p [rad/s]','V [m/s]','$\gamma_a$ [rad]',...
                'h [m]','M [kg]','I$_{C_L}$ [-]','I$_T$ [N]','I$_{V_D}$ [??]',...
                '$\Psi$ [rad]','$\Phi$ [rad]','p [rad/s]',...
                '$\mu$ [rad]','l [rad]','ID','$\dot{h}$ [ft/min]','V [kts]',...
                '$\Psi,\Phi$ [rad]','F [N]','F [N]','F [N]','x$_7$ [kt]','[kt]',...
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