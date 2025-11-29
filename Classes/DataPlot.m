classdef DataPlot < handle
    %DATAPLOT Summary of this class goes here
    %   Detailed explanation goes here

    properties
        nfig
        nplot
        nplotmax
        grd
        lastFig
        linestl
        figID
        xidx
        yidx
        legen
        xlms
        ylms
        xlabl
        ylabl
        ploTitle
        plotFigTit
        plotv
        idx
        leg
        tit
    end
    
    methods
        function obj = DataPlot(plotv)
            %DATAPLOT Construct an instance of this class
            %   Detailed explanation goes here
            obj.plotv = plotv;
            obj.nplotmax = length(plotv(1,:)); % maximum number of plots per image



            obj.lastFig = 0;
            obj.nplot = 0;
            %obj.nfig = 0;
            obj.grd = [];
            obj.figID = [];
            obj.xidx = [];
            obj.yidx = [];
            obj.legen = [];
            obj.xlabl = [];
            obj.ylabl = [];
            obj.ploTitle = [];
            obj.plotFigTit = [];
            obj.linestl = [];
            obj.xlms = [];
            obj.ylms = [];
        end
        

        function obj = definePlot(obj,xidx,yidx,varargin)
            % xidx,yidxs,label,legend
            p = inputParser;
            validateIndex = @(i) isnumeric( i(:) ) && all(i(:)>0,1);
            % Indices of columns to plot
            addRequired(p,'xidx',validateIndex);
            addRequired(p,'yidx',validateIndex);
            % Check if indices are within bounds
            if xidx > obj.nplotmax || any(yidx(:) > obj.nplotmax,1)
                error(['There are only ',num2str(obj.nplotmax)...
                    ,' variables in the array'])
            end
            nyidx = length( yidx(:) );          % Number of y-variables to plot
            % Figure Plot
            if obj.lastFig > 1                  % Is last figure has been defined it adds the current plot in teh last figure
                defaultFigIdx = obj.lastFig;    % FINIREEE
            else
                defaultFigIdx = 1;              % Otherwise it adds the plot in the first figure
            end
            
            function f = checkScal(a)
                ni = length(a); flg = true;
                for i = 1:ni
                    flg = flg && ischar(a{i});
                end
            end
            
            
            function default = modifyBounds(obj,idx)
                default = [ min( min(obj.plotv(:,idx)) ),...
                    max( max(obj.plotv(:,idx)) ) ];
                % for i = 1:2
                    ks = [ 0.1];
                    
                    dint = default(2) - default(1);
                    default(1) = default(1) - dint*ks;
                    default(2) = default(2) + dint*ks;
                % 
                %     if default(i) > 0
                %         default(i) = default(i)*ks(i);
                %     else
                %         default(i) = default(i)*(2-ks(i));
                %     end
                % end
            end
            defaultXlim = modifyBounds(obj,xidx);
            defaultYlim = modifyBounds(obj,yidx);
            % defaultYlim = [ min( min(obj.plotv(:,yidx)) ),...
            %     max( max(obj.plotv(:,yidx))  )];

            addOptional(p,'figID',defaultFigIdx,validateIndex)
            % Non Positional Arguments
            addParameter(p,'legend','-',@checkScal)
            addParameter(p,'grid','off',@ischar)
            addParameter(p,'title','-',@ischar)
            addParameter(p,'xlabel','x',@ischar)
            addParameter(p,'ylabel','y',@ischar)
            addParameter(p,'linestyle','-',@checkScal)
            addParameter(p,'Xlim',defaultXlim,@isnumeric)
            addParameter(p,'Ylim',defaultYlim,@isnumeric)
            p.KeepUnmatched = true;
            parse(p,xidx,yidx,varargin{:})
            
            obj.figID = [obj.figID ; p.Results.figID];
            obj.xidx = [obj.xidx ; p.Results.xidx];
            obj.yidx = [obj.yidx; p.Results.yidx(:)',nan(1,obj.nplotmax-nyidx) ];
            % Legend
            newLeg = [ p.Results.legend,...
                repmat({'-'},1,obj.nplotmax - length(p.Results.legend) ) ];
            newLin = [ p.Results.linestyle,...
                repmat({'-'},1,obj.nplotmax - length(p.Results.linestyle) ) ];
            obj.legen = [obj.legen;newLeg];
            obj.linestl = [obj.linestl;newLin];
            % Plot Title
            obj.xlabl = [obj.xlabl;{p.Results.xlabel}];
            obj.ylabl = [obj.ylabl;{p.Results.ylabel}];
            obj.ploTitle = [obj.ploTitle;{p.Results.title}];
            obj.grd = [obj.grd;{p.Results.grid}];
            % Limits
            obj.xlms = [obj.xlms;p.Results.Xlim];
            obj.ylms = [obj.ylms;p.Results.Ylim];
            obj.nplot = obj.nplot + 1;
            obj.lastFig = p.Results.figID;
        end

        % 
        % 
        function nfig = figureCounter(obj)
            %     %METHOD1 Summary of this method goes here
            %     %   Detailed explanation goes here
            nfig = 1;
            for i=2:obj.nplot
                if obj.figID(i) ~= obj.figID(i)
                    nfig = nfig + 1;
                end
            end
        end
        
        function [fig,ax] = PlotPerImag(obj,nsbp,figTitle,plotFigTit)
            FLEG = 16; FLAB = 16; FTIT = 16;        % Legend, label, Title Font dimensions
            kk = 1;                                 % plots counter
            k2 = 1;
            cnm = 1;
            nfg = obj.figureCounter();             % number of figures required
            if length(figTitle) < nfg || length(plotFigTit) < nfg            % Checks if all figures have a title
                error('Figure names defined are less than teh actual figures')
            end
            ifa = 1;                                % Figures Counter
            for ifi = 1:nfg                        % Cycle for each declared
                % Figures
                nplt = 0;                           % Number of plots per figure
                while obj.figID(k2) == ifi && k2 < obj.nplot
                    nplt = nplt + 1;
                    k2 = k2 + 1;
                end
                %k2 = k2+1;
                if obj.figID(k2) == ifi
                    nplt = nplt + 1;
                end
                
                nfgs = ceil(nplt/nsbp);            % Number of figures desired
                for j = 1:nfgs                      % If the numebr of plots is greater than the maximum numbers of plots per image, then additional figures must be defined
                    if nfgs > 1
                        figName = [figTitle{ifi},' - ',num2str(j)];
                    else
                        figName = figTitle{ifi};
                    end
                    fig(ifa) = figure('Name',figName);  % Defines figures
                    set( fig(ifa), 'Units', 'normalized', ...
                        'Position', [0.1,0.1,0.8,0.8] ) ;
                    tst = figure('Name','Test Figura');  % Defines figures
                    set( fig(ifa), 'Units', 'normalized', ...
                        'Position', [0.1,0.1,0.8,0.8] ) ;
                    % Define Axes
                    nsplot = min(nplt - nsbp*(j-1),nsbp);                   % Number of subplot per figure
                    % - Create grid of axes.
                    [blx, bly] = meshgrid( 0.01, 0.01:0.99/nsplot:0.99 ) ;
                    hAxes = arrayfun( @(x,y) axes( 'Units', 'normalized',...
                        'OuterPosition', ...
                        [x, y, 0.99, 0.99*0.9/nsplot],'Parent',tst... %fig(ifa)...
                        ), blx, bly, 'UniformOutput', false ) ;
                    for k = 1:nsplot
                        cax = k + nsbp*(j-1);                               % cax: number of plots per figure
                        idxs = [ obj.xidx(kk),...
                            obj.yidx(kk,~isnan(obj.yidx(kk,:) ) ) ];        % Save indices of columns to plot
                        % Axes Properties
                        ax(ifi,cax) = ...                                   
                            subplot(nsplot,1,k,'Parent',fig(ifa));          % [figure,ax(1) .. ax(n) ]
                        
                        
                        hold( hAxes{k},'on');
                        xlabel( hAxes{k},obj.xlabl{kk},...
                            "FontSize",FLAB,"Interpreter","latex" );
                        ylabel( hAxes{k},obj.ylabl{kk},...
                            "FontSize",FLAB,"Interpreter","latex" );
                        hAxes{k}.XAxis.FontSize = FLAB;
                        hAxes{k}.YAxis.FontSize = FLAB;


                        hold(ax(ifi,cax),'on');
                        xlabel( ax(ifi,cax),obj.xlabl{kk},...
                            "FontSize",FLAB,"Interpreter","latex" );
                        ylabel( ax(ifi,cax),obj.ylabl{kk},...
                            "FontSize",FLAB,"Interpreter","latex" );
                        
                        ax(ifi,cax).XAxis.FontSize = FLAB;
                        ax(ifi,cax).YAxis.FontSize = FLAB;
                        % Make the axes tick marks and bounding box be really thick.
                        ax(ifi,cax).LineWidth = 0.9;
                        ax(ifi,cax).GridLineWidth = 1;
                        % Let's have the tick marks go outside the graph instead of poking inwards
                        ax(ifi,cax).TickDir = 'out';

                        grid( ax(ifi,cax),obj.grd(kk) );
                        ax(ifi,cax).XLim = obj.xlms(kk,:);
                        ax(ifi,cax).YLim = obj.ylms(kk,:);
                        legf = true;
                        for l = 2:length(idxs)                              % Cycle for lines to plot
                            tmpf = obj.plotting(obj.plotv,idxs(1),idxs(l),ax(ifi,cax),  ...
                                obj.legen{cnm,l-1},l,obj.linestl{cnm,l-1});
                            obj.plotting(obj.plotv,idxs(1),idxs(l),hAxes{k},  ...
                                obj.legen{cnm,l-1},l,obj.linestl{cnm,l-1});
                            legf = legf && tmpf;
                        end
                        if ~strcmp(obj.ploTitle(kk),'-')
                            title( ax(ifi,cax),strcat('\textbf{',obj.ploTitle(kk),'}'),...
                                'Interpreter','Latex','FontSize',FTIT, ...
                                'FontWeight', 'bold');
                        end
                        if nsplot > 1 && ~strcmp( plotFigTit(ifi),'-')
                            sgtitle( plotFigTit(ifi),...
                                'Interpreter','Latex','FontSize',FTIT )
                        end

                        if legf
                            legend( ax(ifi,cax),'Interpreter','latex','FontSize',FLEG );
                        end
                        kk = kk+1; cnm = cnm + 1;
                    end

                
                
                
                end







            end





        end

        function [flg,ax] = plotting(obj,pvec,xidx,yidx,ax,lab,k,lst)
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
            if lst ~= 'o'
                lin = plot(ax,pvec(:,xidx),pvec(:,yidx),'LineWidth',1.3,...
                    'Color',COLS(k,:),'LineStyle',lst );
            else
                lin = plot(ax,pvec(:,xidx),pvec(:,yidx),'LineWidth',1.3,...
                    'Color',COLS(k,:),'LineStyle','none','Marker',lst );
            end

            % if yidx > 99 && yidx < 200
            %     % IDX between 100 and 199 means that we plot a secondary line on top of teh first
            %     lin = plot(ax,pvec(:,xidx),pvec(:,yidx-100),'LineWidth',1.3,...
            %         'Color',COLS(k,:),'LineStyle','--' );
            % elseif yidx > 199 && yidx < 300
            %     % IDX between 200 and 299 means that we plot this line on teh secondary
            %     % axis
            %     yyaxis(ax,'right');  % Activates right side
            %     lin = plot(ax,pvec(:,xidx),pvec(:,yidx-200),'LineWidth',1.3,...
            %         'Color',COLS(k,:) );
            %     yyaxis(ax,'left');
            % elseif yidx > 299 && yidx < 400
            %     % IDX between 300 and 399 means that we plot only circles without
            %     % lines
            %     lin = plot(ax,pvec(:,xidx),pvec(:,yidx-300),'LineWidth',1.3,...
            %         'Color',COLS(k,:),'LineStyle','none','Marker','o' );
            % else
            %     % Normal plot
            %     lin = plot(ax,pvec(:,xidx),pvec(:,yidx),'LineWidth',1.3,...
            %         'Color',COLS(k,:) );
            % end
            if ~isequal(lab,'-')
                lin.DisplayName = lab;
                flg = true;
            end
        end

    end
end

