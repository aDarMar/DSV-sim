classdef DataPlot %< handle
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
        fontdim
        COLS
        axs
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
            obj.xlms = [];      % [xlim(lax),xlim(rax)] 4 x 1
            obj.ylms = [];
            obj.fontdim.FLAB = 16;
            obj.fontdim.FLEG = 16;
            obj.fontdim.FTIT = 16;
            obj.COLS = [
                0, 1, 0;      % Verde
                0, 0, 1;      % Blu
                0, 1, 1;      % Ciano
                1, 0, 1;      % Magenta
                1, 0, 0;      % Rosso
                1, 1, 0;      % Giallo
                1, 0.5, 0;    % Arancione
                0.5, 0, 0.5   % Viola
                ];

        end
        

        function obj = definePlot(obj,xidx,yidx,varargin)
            % xidx,yidxs,label,legend
            p = inputParser;
            validateIndex = @(i) isnumeric( i(:) ) && all(i(:)>0,1);
            % Indices of columns to plot
            addRequired(p,'xidx',validateIndex);
            addRequired(p,'yidx',validateIndex);
            % Check if indices are within bounds
            if any(xidx(:) > obj.nplotmax,1) || any(yidx(:) > obj.nplotmax,1)
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
            
            function flg = checkScal(a)
                ni = length(a); flg = true;
                for i = 1:ni
                    flg = flg && ischar(a{i});
                end
            end
            
            
            function default = modifyBounds(obj,idx)
            % Must be defined after linest has been defined
                if isreal( obj.plotv(:,idx) )
                    default = nan(1,4);
                    for i = 1:length(idx)
                        st = obj.linestl{end,i}; 
                        if strcmp( st(1),'r' )  % Bounds of Right Axes
                            default(3:4) = [ min( default(3),min(obj.plotv(:,idx(i) )) ),...
                                max( default(4),max(obj.plotv(:,idx(i) )) ) ];
                            if any( isnan( default(3:4) ),2 ) % Checks if the vector to plot is all nan
                                default(3:4) = [0,1];
                            end
                        else                    % Bounds of Left Axes
                            default(1:2) = [ min( default(1),min(obj.plotv(:,idx(i) )) ),...
                                max( default(2),max(obj.plotv(:,idx(i) )) ) ];
                            if any( isnan( default(1:2) ),2 ) % Checks if the vector to plot is all nan
                                default(1:2) = [0,1];
                                
                            end
                        end

                    end
                else
                    default = [ min( min( abs(obj.plotv(:,idx)) ) ),...
                        max( max( abs(obj.plotv(:,idx)) ) ) ];
                end
                
                % for i = 1:2
                    ks = [ 0.1];
                    
                    dint = [ default(2) - default(1),default(4) - default(3) ];
                    if dint(1) < 1e-3
                        dint(1) = 1;
                    end
                    if dint(2) < 1e-3
                        dint(2) = 1;
                    end
                    default([2,4]) = default([2,4]) + dint*ks;
                    default([1,3]) = default([1,3]) - dint*ks;

            end
            defaultXlim = [-1,-1,-1,-1];%modifyBounds(obj,xidx);
            defaultYlim = [-1,-1,-1,-1];%modifyBounds(obj,yidx);
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
            % [xlower(lax) lupper(lax) xlower(rax) xupper(rax) ]
            if ~all( p.Results.Xlim(:)-defaultXlim(:),1 )
                temp = modifyBounds(obj,xidx);
            end
            obj.xlms = [obj.xlms;temp];
            if ~all( p.Results.Ylim(:)-defaultYlim(:),1 )
                temp = modifyBounds(obj,yidx);
            end 
            obj.ylms = [obj.ylms;temp];

            obj.nplot = obj.nplot + 1;
            obj.lastFig = p.Results.figID;
        end

        function nfig = figureCounter(obj)
            %     %METHOD1 Summary of this method goes here
            %     %   Detailed explanation goes here
            nfig = 1;
            for i=2:obj.nplot
                if obj.figID(i) ~= obj.figID(i-1)
                    nfig = nfig + 1;
                end
            end
        end
        
        function [fig,obj] = PlotPerImag(obj,nsbp,figTitle,plotFigTit,kind,SAVF)
            %   INPUT
            %   - nsbp: number of subplots per row and colum. If only one
            %       dimension is passed, it is assumed it's for rows
            %   - kind: kind of plot required (cartesian,polar,map)
            if nargin < 6
                SAVF = false;
            end
            if isscalar(nsbp)
                nsbp = [nsbp,1];
            end
            FLEG = 16; FLAB = 16; FTIT = 16;        % Legend, label, Title Font dimensions
            kk = 1;                                 % plots counter
            k2 = 1;
            cnm = 1;
            nfg = obj.figureCounter();              % number of figures required
            if length(figTitle) < nfg || length(plotFigTit) < nfg            % Checks if all figures have a title
                error('Figure names defined are less than teh actual figures')
            end
            ifa = 1;                                % Actual Figures Counter
            for ifi = 1:nfg                         % Cycle for each declared
                ifg = 1;                            % Index used for naming savefiles
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
                
                nfgs = ceil( nplt/(nsbp(1)*nsbp(2)) );             % Number of figures desired
                for j = 1:nfgs                      % If the numebr of plots is greater than the maximum numbers of plots per image, then additional figures must be defined
                    if nfgs > 1
                        figName = [figTitle{ifi},' - ',num2str(j)];
                    else
                        figName = figTitle{ifi};
                    end
                    fig(ifa) = figure('Name',figName);  % Defines figures
                    set( fig(ifa), 'Units', 'normalized', ...
                        'Position', [0.1,0.1,0.8,0.8] ) ;
                    % Define Axes
                    nsplot = min( nplt - nsbp(1)*nsbp(2)*(j-1),...
                        nsbp(1)*nsbp(2) );                   % Number of remaining subplot per figure
                    % Divide the Subplots in Rows and COlumns
                    nRow = ceil( nsplot/nsbp(2) );          % Number of rows necessary
                    if nRow == 1 && nsplot < nsbp(2)
                        hAxes = obj.defineAxes(nRow,nsplot,fig(ifa),kind);
                    else
                        hAxes = obj.defineAxes(nRow,nsbp(2),fig(ifa),kind);
                    end
                    % if mod( nsplot,nsbp(2) ) > 0 && floor( nsplot/nsbp(2) ) == 0
                    %     hAxes = obj.defineAxes(nsplot,1,fig(ifa),kind);
                    % else
                    % 
                    % end

                    if nsplot > 1 && ~strcmp( plotFigTit(ifi),'-')
                        % Title of the figure
                        t = annotation( fig(ifa),'textbox',[.48,.9,0.1,.1],...
                            'String',plotFigTit(ifi),'Units','normalized',...
                            'interpreter','latex','FontSize',obj.fontdim.FTIT );
                        t.LineStyle = 'none'; t.HorizontalAlignment = "center";
                    end


                    for k = 1:nsplot
                        cax = k + nsbp*(j-1);                               % cax: number of plots per figure
                        idxs = [ obj.xidx(kk),...
                            obj.yidx(kk,~isnan(obj.yidx(kk,:) ) ) ];        % Save indices of columns to plot
                        
                        % Cartesian
                        % Axes Properties
                        hold( hAxes{k},'on');

                        
                        hAxes{k}.FontName = "Times New Roman";
                       
                        % Let's have the tick marks go outside the graph instead of poking inwards
                        hAxes{k}.TickDir = 'out';
                        % Check if x and y indices are equal in number
                        xids = obj.xidx( kk,~isnan(obj.xidx(kk,:)) );
                        yids = obj.yidx( kk,~isnan(obj.yidx(kk,:)) );
                        if ~isscalar(xids) && ( length(xids) ~= length( yids ) )
                            % Checks if the number of x indices is equal to
                            % the numeber of y indices only if obj.xidx is
                            % not a scalar
                            error('Number of x indices is different from y indices')
                        end

                        grid( hAxes{k},obj.grd(kk) );

                        % Polar
                
                        legf = true;
                        switch kind
                            case 'cartesian'
                                xlabel( hAxes{k},obj.xlabl{kk},...
                                    "FontSize",obj.fontdim.FLAB,"Interpreter","latex" );
                                ylabel( hAxes{k},obj.ylabl{kk},...
                                    "FontSize",obj.fontdim.FLAB,"Interpreter","latex" );
                                hAxes{k}.XAxis.FontSize = obj.fontdim.FLAB;
                                hAxes{k}.YAxis.FontSize = obj.fontdim.FLAB;
                                % Left Axes Limits
                                hAxes{k}.XLim = obj.xlms(kk,1:2);
                                hAxes{k}.YLim = obj.ylms(kk,1:2);
                                % Make the axes tick marks and bounding box be really thick.
                                hAxes{k}.LineWidth = 0.9;
                                hAxes{k}.GridLineWidth = 1;
                                % Tiks
                                hAxes{k}.XMinorTick = "on";
                                hAxes{k}.YMinorTick = "on";
                                if isscalar(xids)
                                    for l = 1:length(yids)                              % Cycle for lines to plot
                                        tmpf = obj.plotting(kind,obj.plotv,xids(1),yids(l),hAxes{k},  ...
                                            obj.legen{cnm,l},l,obj.linestl{cnm,l}); % xidx is always the first
                                        legf = legf && tmpf;
                                    end
                                else
                                    for l = 1:length(yids)                              % Cycle for lines to plot
                                        tmpf = obj.plotting(kind,obj.plotv,xids(l),yids(l),hAxes{k},  ...
                                            obj.legen{cnm,l},l,obj.linestl{cnm,l}); 
                                        legf = legf && tmpf;
                                    end
                                end
                                if ~isscalar(hAxes{k}.YAxis)
                                    % If a second y axis is defined, apply
                                    % all the properties of the first to
                                    % the second
                                    hAxes{k}.YAxis(2).Limits = obj.ylms(kk,3:4);
                                    hAxes{k}.YAxis(2).MinorTick = 'on';
                                    hAxes{k}.YAxis(2).TickLabelInterpreter = 'latex';
                                    hAxes{k}.YAxis(2).FontSize = obj.fontdim.FLAB;
                                    % Separates the label to assign to the
                                    % different axes
                                    [token,remain] = strtok( ...
                                        obj.ylabl{kk},'|' );
                                    hAxes{k}.YAxis(1).Label.String = token;
                                    hAxes{k}.YAxis(2).Label.String = remain(2:end);
                                    hAxes{k}.YAxis(2).Label.Interpreter = 'latex';
                                    % Align the second grid to the first
                                    ntimj = length( hAxes{k}.YAxis(1).TickValues );     % Number of major tiks of the left axis
                                    ntik = length( hAxes{k}.YAxis(1).MinorTickValues ) + ...
                                        ntimj; % Total Number of Tiks of left axis
                                    tikmin = linspace( hAxes{k}.YAxis(2).Limits(1),...
                                        hAxes{k}.YAxis(2).Limits(2),ntik ); % Tiks values of the right axis
                                    % Find at what indices there are major
                                    % tiks
                                    idT = nan(1,ntimj); idT(1) = 1; idT(end) = ntik;
                                    for kT = 1:ntimj
                                        for i = 2:ntik-ntimj
                                            if hAxes{k}.YAxis(1).TickValues(kT) > hAxes{k}.YAxis(1).MinorTickValues(i-1) && ...
                                                    hAxes{k}.YAxis(1).TickValues(kT) < hAxes{k}.YAxis(1).MinorTickValues(i)
                                                % Checks if the major tick is
                                                % between the two minor labels
                                                idT(kT) = i+kT-1;
                                                break
                                            end
                                        end
                                    end
                                    hAxes{k}.YAxis(2).TickValues = tikmin(idT); % Position of Major Tiks
                                    tikmin(idT) = [];
                                    hAxes{k}.YAxis(2).MinorTickValues = tikmin; % Position of Minor Tiks
                                        
                                end
                            case 'polar'
                                hAxes{k}.RAxis.FontSize = obj.fontdim.FLAB;
                                hAxes{k}.ThetaAxis.FontSize = obj.fontdim.FLAB;
                                hAxes{k}.RLim = [0,max( [obj.ylms( kk,: ),...
                                    obj.xlms( kk,: )] ) ];
                                % hAxes{k}.ThetaLim = ;
                                % Tiks
                                hAxes{k}.RMinorTick = "on";
                                hAxes{k}.ThetaMinorTick = "on";
                                hAxes{k}.LineWidth = 0.9;

                                for l = 1:length(idxs)                              % Cycle for lines to plot
                                    tmpf = obj.plotting(kind,obj.plotv,[],idxs(l),hAxes{k},  ...
                                        obj.legen{cnm,l},l,obj.linestl{cnm,l},k);
                                    legf = legf && tmpf;
                                end
                        end
                        if ~strcmp(obj.ploTitle(kk),'-')
                            title( hAxes{k},strcat('\textbf{',obj.ploTitle(kk),'}'),...
                                'Interpreter','Latex','FontSize',obj.fontdim.FTIT, ...
                                'FontWeight', 'bold');
                        end

                        if legf
                            legend( hAxes{k},'Interpreter','latex','FontSize',obj.fontdim.FLEG );
                        end
                        kk = kk+1; cnm = cnm + 1;
                    end
                    if SAVF
                    % Saves figures in an output folder
                        saveas( fig(ifa),strcat('Output\',...
                            strrep( figTitle{ifi},' ','' ),...
                            '_',num2str( ifg ),'.svg') )
                    end
                    ifa = ifa + 1; ifg = ifg + 1;
                end

            end
            obj.axs = hAxes;
        end

        function [flg,ax] = plotting(obj,kind,pvec,xidx,yidx,ax,lab,k,lst,kk)
        %   INPUT
        %       - lst: flag specifying line style. 
        %           '<linestyle>'to plot a simple line (ie '--' '-.');
        %           'o<marker>' to plot data with markers only (ie 'oo','odiamond'),
        %           'l<linestyle>' to plot lines creating a second y (ie
        %               'l--')
        %       ax
            flg = false;
            switch kind
                case 'cartesian'
                    % Cartesian plot
                    switch lst(1)
                        case 'r'
                            % Second Axis 
                            yyaxis(ax,'right');  % Activates right side
                            %axl.YLim = obj.ylms(kk,3:4);
                            lin = plot(ax,pvec(:,xidx),pvec(:,yidx),'LineWidth',1.3,...
                                'Color',obj.COLS(k,:),'LineStyle','none','LineStyle',lst(2:end) );
                            n = length(pvec(:,yidx)); n = ceil(n/2);
                            yarr = pvec(n,yidx); xarr = pvec(n,xidx);
                            text(ax,xarr,yarr,'$\rightarrow$','interpreter','latex',...
                                'FontSize',obj.fontdim.FLAB);
                            yyaxis(ax,'left');
                        case 'o'
                            % Marker only and no line
                            if isscalar(lst)
                                % Option used for retrocompatibility: if
                                % the marker flag is just 'o' it is
                                % understood as 'oo'
                                lin = plot(ax,pvec(:,xidx),pvec(:,yidx),'LineWidth',1.3,...
                                    'Color',obj.COLS(k,:),'LineStyle','none','Marker',lst );
                            else
                                lin = plot(ax,pvec(:,xidx),pvec(:,yidx),'LineWidth',1.3,...
                                    'Color',obj.COLS(k,:),'LineStyle','none','Marker',lst(2:end) );
                            end
                        otherwise
                            lin = plot(ax,pvec(:,xidx),pvec(:,yidx),'LineWidth',1.3,...
                            'Color',obj.COLS(k,:),'LineStyle',lst );
                    end
                case 'polar'
                    % Polar plot: it makes sense only if the elements are
                    % complex
                    md = abs( pvec(:,yidx) );
                    phs = angle( pvec(:,yidx) );       % phase of each element
                    lin = polarplot(ax,[phs,phs]*180/pi,[0,md],...
                        'LineWidth',1.3,...
                        'Color',obj.COLS(k,:),'LineStyle',lst );

            end


            if ~isequal(lab,'-')
                lin.DisplayName = lab;
                flg = true;
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
        end
        
        function axs = defineAxes(obj,nRows,nCols,fig,plknd)
            
            if nargin < 5
                plknd = 'normal';
            end
            spRow = 0.99/nRows; spCol = 0.99/nCols; % Plots width and height
            [blx, bly] = meshgrid( 0.01:spCol:0.99, 0.01:spRow:0.99 ) ;
            switch plknd
                case 'polar'
                    axs = arrayfun( @(x,y) polaraxes( 'Units', 'normalized',...
                        'OuterPosition', ...
                        [x, y, 0.99*spCol, 0.99*spRow],'Parent',fig...
                        ), blx, bly, 'UniformOutput', false ) ;
                otherwise
                    axs = arrayfun( @(x,y) axes( 'Units', 'normalized',...
                        'OuterPosition', ...
                        [x, y, 0.99*spCol, 0.99*spRow],'Parent',fig...
                        ), blx, bly, 'UniformOutput', false ) ;
            end
        end

        function [obj,axs] = rootLocus(obj,Gain,Gain_name,SAVF)
        %ROOTLOCUS: function that plots the root locus. It is assumed that
        % plotv is n the form: [Re(eig1),Imag(eig1,Re(eig2),Imag(eig2)...


        IMSAV = {'Root Locus'}; IMTIT = {'Root Locus'};

        Reidx = [1:2:obj.nplotmax]' ;
        imagIdx = [2:2:obj.nplotmax]';
        
        obj = obj.definePlot( Reidx(1),imagIdx(1),1,'xlabel','Real','ylabel','Imag',...
            'title',strcat('Root Locus ',Gain_name) );                 % Initialize graphics
        [fig,axR] = obj.PlotPerImag( [1,1],IMSAV,IMTIT,'cartesian',false );
        annIdx = nan(10,obj.nplotmax*0.5);      % Indices where the data must be displayed
        annIdx(1,:) = ones(1,obj.nplotmax*0.5);
        annIdx(end,:) = ones( 1,obj.nplotmax*0.5)*length(obj.plotv(:,1) );
        for i = 1:obj.nplotmax*0.5
            obj.definePlot( Reidx(i),imagIdx(i),1 )
           % Root Locus Branches
            obj.plotting( 'cartesian',obj.plotv,2*i-1,2*i,axR{1},'-',i,'-');
            obj.plotting( 'cartesian',obj.plotv(1,:),2*i-1,2*i,axR{1},'-',i,'o');
            % Find Break-in and Crossover Points
            flg1 = false; flg2 = false;
            for j = 2:length( obj.plotv(:,1) ) % Break in
                if ~(obj.plotv(j,2*i)*obj.plotv(j-1,2*i))>0 && ~flg1
                    annIdx(2,i) = j;
                    flg1 = true;
                elseif ~(obj.plotv(j,2*i-1)*obj.plotv(j-1,2*i-1))>0 && ~flg2 % Crossover
                    annIdx(3,i) = j;
                    flg2 = true;
                end
            end

        end
        annIdx = annIdx( ~isnan(annIdx) );
        annIdx = unique(annIdx);

        % Adjust Limits
        axR{1}.XLim = [ min(min(obj.plotv(:,Reidx))),max(max(obj.plotv(:,Reidx))) ];
        if min(min(obj.plotv(:,imagIdx))) == max(max(obj.plotv(:,imagIdx)))
            axR{1}.YLim = [ -0.1,0.1 ];
        else
            axR{1}.YLim = [ min(min(obj.plotv(:,imagIdx))),max(max(obj.plotv(:,imagIdx))) ];
        end
        % Notes
        ks = 1e-2;
        a = 30*pi/180; dl = axR{1}.YLim(2) - axR{1}.YLim(1) ;
        a = atan( dl/(axR{1}.XLim(2) - axR{1}.XLim(1) ) );
        dl = dl^2 + ( axR{1}.XLim(2) - axR{1}.XLim(1) )^2; dl = sqrt(dl)*ks;
        for j = 1:length( annIdx(:,1) )
            for i =1:obj.nplotmax*0.5
                
                if ~isnan( annIdx(j) )
                    xax = obj.plotv(annIdx(j),2*i-1)-cos(a)*dl;
                    yax = obj.plotv(annIdx(j),2*i)-sin(a)*dl;
                    plot(axR{1},[obj.plotv(annIdx(j),2*i-1),xax],...
                        [obj.plotv(annIdx(j),2*i),yax],'-k','LineWidth',0.6 );
                    text(axR{1},xax,yax ,strcat( Gain_name,' = ',...
                        num2str( Gain(annIdx(j) ) ) ),'Interpreter','Latex',...
                        'FontSize',obj.fontdim.FLAB);
                end
                
                % fig.Position
                % % Normalize Position
                % xarrow = [obj.plotv(j,2*i-1)-cos(a)*dl,obj.plotv(j,2*i-1)] - axR{1}.XLim(1);
                % xarrow = xarrow/( axR{1}.XLim(2) - axR{1}.XLim(1) );
                % xarrow = xarrow + axR{1}.Position(1);
                % yarrow = [0.5,0.5]
                % 
                % annotation('textarrow',xarrow,...
                %     [obj.plotv(j,2*i)-sin(a)*dl,obj.plotv(j,2*i)]*ks/dl,...
                %     'String',strcat( Gain_name,' = ',num2str(Gain) ),'Units','points' );
            end
        end
        
        % Data 
        %tanb = linspace( 0,100,10 ); % 
        %
        tanb = linspace(0,axR{1}.YLim(2),5);
        %zitav = cos(atan(tanb));
        zitav = 0:0.5:1;

        wn = linspace( 0.1,abs(axR{1}.XLim(1)),10 );%max( max( abs( obj.plotv(Reidx,:) ) ) ),30 );
        x = [axR{1}.XLim(1),0];
        for i = 1:length(zitav)
            tanb(i) = sqrt(1 - zitav(i)^2)/zitav(i);
            y = tanb(i) * x; % y = m*x line for constant damping
            plot(axR{1},x,y,'LineWidth',0.5,'Color',[1,1,1]*219/256);
            plot(axR{1},x,-y,'LineWidth',0.5,'Color',[1,1,1]*219/256);
            text( (x(end)+x(1))*0.5,(y(end)+y(1))*0.5,strcat('$\zeta = $',...
                num2str(round(zitav(i),3) ) ),'Interpreter','latex','FontSize',obj.fontdim.FLAB );
        end
        zitav = linspace(0.01,1,500);
        for i = 1:length(wn)
            x = -wn(i)*zitav; y = wn(i)*sqrt(1-zitav);
            plot(axR{1},x,y,'LineWidth',0.5,'Color',[1,1,1]*219/256);
            plot(axR{1},x,-y,'LineWidth',0.5,'Color',[1,1,1]*219/256);
            % wn labels are alignet to a straight line connecting the
            % origin to the bottom left corner
            alab = axR{1}.YLim(1)/axR{1}.XLim(1);           % slope of the straight line
            text( x(end), alab*x(end) ,...
                strcat('$\omega_n = $',...
                num2str(round(wn(i),2) ) ),'Interpreter','latex',...
                'FontSize',obj.fontdim.FLAB );
        end

        if SAVF
            % Saves figures in an output folder
            saveas( fig,strcat('Output\RL_',...
                strrep( strrep( strrep( strrep(Gain_name,'$',''),'\','' ),'{','' ),'}','' ),...
                '.svg') )
        end










            % dims = size(rads);
            % fig = figure('Name','Root Locus');  % Defines figures
            % set( fig(ifa), 'Units', 'normalized', ...
            %     'Position', [0.1,0.1,0.8,0.8] ) ;
            % % Define Axes
            % hAxes = obj.defineAxes(1,1,fig,'cartesian');
            % % Make the axes tick marks and bounding box be really thick.
            % hAxes.LineWidth = 0.9;
            % hAxes.GridLineWidth = 1;
            % % Tiks
            % hAxes.XMinorTick = "on";
            % hAxes.YMinorTick = "on";
            % 
            % 
            % for i = 1:obj.nplotmax*0.5
            %     obj.plotting( 'cartesian',2*i-1,2*i,hAxes,i,...
            %         obj.linestl(i) );
            % 
            % end

        end
        
        
        % % % https://www.mathworks.com/matlabcentral/answers/233818-how-to-create-subplots-with-little-vertical-spacing
        % % load clown ;
        % % 
        % % nRows = 3 ;
        % % nCols = 5 ;
        % % 
        % % % - Create figure, set position/size to almost full screen.
        % % figure() ;
        % % set( gcf, 'Units', 'normalized', 'Position', [0.1,0.1,0.8,0.8] ) ;
        % % 
        % % % - Create grid of axes.
        % % [blx, bly] = meshgrid( 0.05:0.9/nCols:0.9, 0.05:0.9/nRows:0.9 ) ;
        % % hAxes = arrayfun( @(x,y) axes( 'Position', [x, y, 0.9*0.9/nCols, 0.9*0.9/nRows] ), blx, bly, 'UniformOutput', false ) ;
        % % 
        % % % - "Plot data", update axes parameters.
        % % for k = 1 : numel( hAxes )
        % %     axes( hAxes{k} ) ;
        % %     image( X ) ;
        % %     set( gca, 'Visible', 'off' ) ;
        % % end
        % % 
        % % colormap( map ) ;

        





        function [fig,ax] = plotPolar(obj,eigStr,lab)
        %plotPolar: function that plots a complex number in a polar plot
        FLEG = 16;
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


        fig = figure('Units', 'normalized', ...
                        'Position', [0.1,0.1,0.8,0.8] );
        axs = obj.defineAxes(1,2,fig,'polar');

        nvec = size(eigStr);


        md = abs(eigStr);
        phs = angle(eigStr);       % phase of each element
        for j = 1:nvec(2)
            for i = 1:nvec(1)
                hold(axs{j},'on')
                lin = polarplot(axs{j},[phs(i,j),phs(i,j)]*180/pi,[0,md(i,j)],...
                    'LineWidth',1.3,...
                    'Color',COLS(i,:),'LineStyle','-' );
                if ~isequal(lab{i},'-')
                    lin.DisplayName = lab{i};
                    flg = true;
                end
            end
            legend( axs{j},'Interpreter','latex','FontSize',FLEG  )
        end


        end
    end
end

