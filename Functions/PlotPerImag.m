function [fig,ax] = PlotPerImag(pvec,nsbp,figS,NFIG,Leg,TIT)
%PLOTPERIMAG Summary of this function goes here
%   Detailed explanation goes here
%   INPUT
%   -pvec: vector of data to plot. Data is plotted alogn columns
%   - nsbp: number of subplot desird for image
%   - figS: array containing indices to plot: [IDXfig,IDXx,IDXy]
%           IDXfig: index of the figure (i.e. plot on figure 1)
%           IDXx: index of x values
%           IDYs: Indices of y values
%   [3,1,2,3,4] means plot on figure 3 on the x axis data copntained in teh
%           column 1 vs data contained in comumns 2 to 4 of vector vPlot
%   - NFIG: Cell of strings containing figures names
%   - LEG: Cell containing legend entries
%   - TIT: Titles of Subplots
%   - STTIT: Titles of Group of Subplots (i.e. Sgtitle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    j  = 1;                                 % Plot Counter
    kk = 1;                                 % ax counter
    k2 = 1;
    cnm = 1;
    nfig = max( figS(:,1) );                % number of figures required
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
                title( ax(ifi,cax),TIT{cnm} );
                if legf
                    legend( ax(ifi,cax) );
                end
                kk = kk+1; cnm = cnm + 1;
            end
    
            ifa = ifa+1;
        end
    end                                   % Number of figures desired
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
        elseif yidx > 199 && yidx < 300
        % IDX between 200 and 299 means that we plot this line on teh secondary
        % axis
            yyaxis(ax,'right');  % Activates right side
            lin = plot(ax,pvec(:,xidx),pvec(:,yidx-200),'LineWidth',1.3,...
                'Color',COLS(k,:) );
            yyaxis(ax,'left');
       elseif yidx > 299 && yidx < 400
        % IDX between 300 and 399 means that we plot only circles without
        % lines
        lin = plot(ax,pvec(:,xidx),pvec(:,yidx-300),'LineWidth',1.3,...
            'Color',COLS(k,:),'LineStyle','none','Marker','o' );
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
