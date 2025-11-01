function plot_results(flg,t,x,x_aux)
%PLOT_RESULTS Function that plots the results of the integration
%   INPUT
%   x - longitudinal state vector [V,ga,h,m]
%   y - longitudinal output vector [VIAS,M,h,hdot]
%   
    switch flg
        case 'ctrl'
        % Commanded valeu case
        i = 1;                                          % figure index
        y = LongDynNoLin_Out(x'); y = y';                        % output vector
        fig(i) = figure('Name','1');
        % fig 1: hdot,V,CL,T
        plotvec = [y(:,1),y(:,4),x_aux(:,5:6),x_aux(:,2:3)];      % Vector used to plot results [VIAS,hdot,CL,T,Vc,hdotc]
        nCase = length(plotvec(1,:)) - 2;               % Number of subplots
        TIT = {'Actual vs Commanded IAS','Actual vs Commanded hdot','CL','T'};
        for j = 1:2
            ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
            hold(ax(i,j),'on'); title(ax(i,j),TIT{j});
            plot( t,plotvec(:,j) );
            plot( t,plotvec(:,nCase+j),'--r' );
        end
        for j = 3:nCase
            ax(i,j) = subplot(nCase,1,j,'Parent',fig(i));
            hold(ax(i,j),'on'); title(ax(i,j),TIT{j});
            plot( t,plotvec(:,j) );

        end



        case 'waypoint'
    
    end

end

