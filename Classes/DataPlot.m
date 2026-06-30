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
                229, 65, 56
                229, 151, 56
                220, 229, 56
                56, 220, 229
                65, 56, 229];
            obj.COLS(9:end,:) = obj.COLS(9:end,:)/255;

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
                        max( max( abs(obj.plotv(:,idx)) ) ),-1,1 ];
                end
                
                % for i = 1:2
                    ks = [ 0.1];
                    
                    dint = [ default(2) - default(1),default(4) - default(3) ];
                    if dint(1) < 1e-3
                        dint(1) = 1;
                    end
                    if dint(2) < 1e-4
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
        
        function [fig,obj,hAxes] = PlotPerImag(obj,nsbp,figTitle,plotFigTit,kind,SAVF)
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
                    lin = polarplot(ax,[phs,phs],[0,md],...
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
        [fig,obj,axR] = obj.PlotPerImag( [1,1],IMSAV,IMTIT,'cartesian',false );
        axR = axR{1};
        annIdx = nan(10,obj.nplotmax*0.5);      % Indices where the data must be displayed
        annIdx(1,:) = ones(1,obj.nplotmax*0.5);
        annIdx(end,:) = ones( 1,obj.nplotmax*0.5)*length(obj.plotv(:,1) );
        for i = 1:obj.nplotmax*0.5
            obj.definePlot( Reidx(i),imagIdx(i),1 )
           % Root Locus Branches
            obj.plotting( 'cartesian',obj.plotv,2*i-1,2*i,axR,'-',i,'-');
            obj.plotting( 'cartesian',obj.plotv(1,:),2*i-1,2*i,axR,'-',i,'ox');
            % Find Break-in and Crossover Points
            flg1 = false; flg2 = false; k = 1;
            for j = 2:length( obj.plotv(:,1) ) % Break in/away
                if j == 174
                    disp('a')
                end
                if ~(obj.plotv(j,2*i)*obj.plotv(j-1,2*i)>0) && any(obj.plotv(j-1:j,2*i) > 0,1)
                    annIdx(1+k,i) = j;
                    flg1 = true;
                    k = k+1;
                elseif ~(obj.plotv(j,2*i-1)*obj.plotv(j-1,2*i-1)>0) && any(obj.plotv(j-1:j,2*i-1) > 0,1) % Crossover
                    annIdx(1+k,i) = j;
                    flg2 = true;
                    k = k+1;
                end
            end

        end
        annIdx = annIdx( ~isnan(annIdx) );
        annIdx = unique(annIdx);

        % Adjust Limits
        axR.XLim = [ min(min(obj.plotv(:,Reidx))),max(max(obj.plotv(:,Reidx))) ];
        if min(min(obj.plotv(:,imagIdx))) == max(max(obj.plotv(:,imagIdx)))
            axR.YLim = [ -0.1,0.1 ];
        else
            axR.YLim = [ min(min(obj.plotv(:,imagIdx))),max(max(obj.plotv(:,imagIdx))) ];
        end
        % Notes
        ks = 1e-2;
        a = 30*pi/180; dl = axR.YLim(2) - axR.YLim(1) ;
        a = atan( dl/(axR.XLim(2) - axR.XLim(1) ) );
        dl = dl^2 + ( axR.XLim(2) - axR.XLim(1) )^2; dl = sqrt(dl)*ks;
        for j = 1:length( annIdx(:,1) )
            for i =1:obj.nplotmax*0.5
                
                if ~isnan( annIdx(j) )
                    xax = obj.plotv(annIdx(j),2*i-1)-cos(a)*dl;
                    yax = obj.plotv(annIdx(j),2*i)-sin(a)*dl;
                    plot(axR,[obj.plotv(annIdx(j),2*i-1),xax],...
                        [obj.plotv(annIdx(j),2*i),yax],'-k','LineWidth',0.6 );
                    text(axR,xax,yax ,strcat( Gain_name,' = ',...
                        num2str( Gain(annIdx(j) ) ) ),'Interpreter','Latex',...
                        'FontSize',obj.fontdim.FLAB);
                end
                obj.addModalProp(axR); % Add is damping and wn lines
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

        function axR = addModalProp(obj,axR)
        %ADDMODALPROP: function that adds the lines of constant damping
        % and angular frequency to a root locus/complex plane
        % eigenvalues plot
            tanb = linspace(0,axR.YLim(2),5);
            %zitav = cos(atan(tanb));
            zitav = 0:0.1:1;

            wn = linspace( 0.1,abs(axR.XLim(1)),10 );%max( max( abs( obj.plotv(Reidx,:) ) ) ),30 );
            x = [axR.XLim(1),0];
            %% Iso Zeta Lines
            for i = 1:length(zitav)
                tanb(i) = sqrt(1 - zitav(i)^2)/zitav(i);
                y = tanb(i) * x; % y = m*x line for constant damping
                plot(axR,x,y,'LineWidth',0.5,'Color',[1,1,1]*219/256);
                plot(axR,x,-y,'LineWidth',0.5,'Color',[1,1,1]*219/256);
                text( axR,(x(end)+x(1))*0.5,(y(end)+y(1))*0.5,strcat('$\zeta = $',...
                    num2str(round(zitav(i),3) ) ),'Interpreter','latex','FontSize',obj.fontdim.FLAB );
            end
            %% Iso Omega n Circles
            zitav = linspace(0.01,1,500);
            for i = 1:length(wn)
                x = -wn(i)*zitav; y = wn(i)*sqrt(1-zitav.^2);
                plot(axR,x,y,'LineWidth',0.5,'Color',[1,1,1]*219/256);
                plot(axR,x,-y,'LineWidth',0.5,'Color',[1,1,1]*219/256);
                % wn labels are alignet to a straight line connecting the
                % origin to the bottom left corner
                alab = axR.YLim(1)/axR.XLim(1);           % slope of the straight line
                xwn = max( x )*0.3 + min( x )*0.7;
                text( axR,xwn, alab*xwn ,...
                    strcat('$\omega_n = $',...
                    num2str(round(wn(i),2) ) ),'Interpreter','latex',...
                    'FontSize',obj.fontdim.FLAB );
            end
        end
        function [fig,hAx] = trajectoryPlot(obj,kind,vidx,GEO,AC)
            %TRAJECTORYPLOT: function that plots the trajectory of the
            % aircraft.
            %   INPUT
            %   - kind
            %   - varidx: columns of variables to plot in the plot vector
            %           property
            %   - GEO: geobject class object

            % Plot Waypoints
            %%Rw = wayid(); FINIREEEE AGGIUNGERE WAYPOINTS
            %%plot3(axG,Rw(:,1),Rw(:,2),Rw(:,3),'or','MarkerSize',5);              % Markers plot

            p = inputParser;

            function p = flagcheck(a)
                if ~strcmp(a,'ECEF') && ~strcmp(a,'NED') ...
                        && ~strcmp(a,'NEDi')
                    p = false;
                else
                    p = true;
                end
            end
            addRequired(p,'kind',@flagcheck);
            parse(p,kind);

            function q = quiverPlot(ax,R,U,iP)
            %QUIVERPLOT:
            %   INPUT
            %   - R: [Rx;Ry;Rz] Coordinates of origin
            %   - U: [Ux;Uy;Uz]
            % ----------------------------------------------------------- %
                nl = length( R(1,:) );
                if nl ~= length( U(1,:) )
                    error('R and U have different dimensions')
                end
                for iG = 1:nl
                    q(iG) = quiver3( ax,R(1,iG),R(2,iG),R(3,iG),...
                        U(1,iG),U(2,iG),U(3,iG),'LineWidth',1.5,'Color',vers(iP,:) );
                end

            end
            % Figure
            fig = figure();         % figure definition
            set( fig, 'Units', 'normalized', ...
                        'Position', [0.1,0.1,0.8,0.8] ) ;
            % efig = uifigure;         % figure definition
            % set( efig, 'Units', 'normalized', ...
            %             'Position', [0.1,0.1,0.8,0.8] ) ;
            % Initializzation
            vers = eye(3);          % Identity Matrix used for plot and versors
            scl = 10e2;             % Scale Factor
            %ptidx = choosePts(Rs);
            hAx = obj.defineAxes(1,1,fig,'normal');     % Axes Definition

            Rgt = GEO.LatLon2Vec( obj.plotv(:,vidx(1)) ,...
                obj.plotv(:,vidx(2)),0 );                               % Ground Track Trajectory
            Rs = GEO.LatLon2Vec( obj.plotv(:,vidx(1)) ,...
                obj.plotv(:,vidx(2)),obj.plotv(:,vidx(3)) );            % Trajectory in ECEF coordinates

            ptidx = obj.plotidx( vidx(7),vidx(1:6) );                   % Indices for plotting Ref. Sys

            switch p.Results.kind
                case 'ECEF'
                % Trajectory plot in ECEF coordinates, with NED and Body
                % Reference Systems
                %   INPUT
                %       - varidx: [Long,Lat,h,psi,teta,phi,t ]
                    if length(vidx) < 6
                        error('varidx vector must have 6 indices')
                    end
                    % Plot Attitude
                    
                    idxs = 1:length(ptidx);

                    obj.plotting3D(Rgt',1,2,3,hAx{1},'--');                         % Ground Track Plot
                    hold( hAx{1},'on')
                    obj.plotting3D(Rs',1,2,3,hAx{1},'-');                           % Trajectory Plot
                    

                %     g = geoglobe(efig,Basemap="darkwater",Terrain="none");
                %     geoplot3(g,obj.plotv(:,vidx(1))*180/pi ,...
                % obj.plotv(:,vidx(2))*180/pi,obj.plotv(:,vidx(3)),HeightReference="ellipsoid");
                %     hold(g,'on');
                %     geoplot3(g,obj.plotv(:,vidx(1))*180/pi ,...
                %         obj.plotv(:,vidx(2))*180/pi,obj.plotv(:,vidx(3))*0,'y',HeightReference="ellipsoid");
                %     campitch(g,-30)
                %     camheading(g,40)
                    % Reference System Versors
                    for j = ptidx
                        for i = 1:3
                            % Ground Track
                            UsN = GEO.NED2DIS( vers(:,i),'N2E',...
                                obj.plotv( j,vidx(1) ), ...
                                obj.plotv( j,vidx(2) ) );                       % Local NED Axes
                            q = quiverPlot( hAx{1},Rgt(:,j),UsN*scl,i );   % Plot Local NED and Origin on the ground track
                            % Trajectory
                            UsB = AC.body2NED(vers(:,i),'B2N',obj.plotv( j,vidx(4) ),...
                                obj.plotv( j,vidx(5) ),obj.plotv( j,vidx(6) ) );          % Body in NED
                            UsB = GEO.NED2DIS(UsB,'N2E',obj.plotv( j,vidx(1) ), ...
                                obj.plotv( j,vidx(2) ) );
                            q = quiverPlot( hAx{1},Rs(:,j),UsB*scl,i );            % Plot Body and CG
                        end
                    end
                    set( hAx{1},'FontSize',16,'FontName','Times New Roman' );
                    lms = nan(1,6);
                    lms([1,3,5]) = min([Rgt';Rs']); lms([2,4,6]) = max([Rgt';Rs']);
                    axis( lms )
                case 'NEDi'
                % Trajectory in initial NED Coordinates
                %   INPUT
                %       - x,y,z trajectory components in ECEF coordinates
                %       -
                % ---------------------------------------------------- %
                    is = 1; % Index of reference NED
                    % Trajectory
                    Rs = Rs - Rgt(:,is);                                     % Position Vector @ the NED at t0
                    Rgt = Rgt - Rgt(:,is);
                    % From ECEF to Initial NED
                    for j = 1:length( obj.plotv(:,1) )
                        Rs(:,j) = GEO.NED2DIS(Rs(:,j),'E2N',obj.plotv( 1,vidx(1) ), ...
                            obj.plotv( 1,vidx(2) ) );                   % Position in NED at t0
                        Rgt(:,j) = GEO.NED2DIS( Rgt(:,j),'E2N',obj.plotv(1,vidx(1)) ,...
                            obj.plotv(1,vidx(2)) );
                    end
                    
                    % Same with Ground Track
                    
                    
                    % Plot Trajectory and Ground Track
                    obj.plotting3D(Rgt',1,2,3,hAx{1},'--');                         % Ground Track Plot
                    hold( hAx{1},'on')
                    obj.plotting3D(Rs',1,2,3,hAx{1},'-');                           % Trajectory Plot
                    % Plot reference systems
                    for j = ptidx
                        for i = 1:3
                            % Ground Track
                            UsN = GEO.NED2DIS( vers(:,i),'N2E',...
                                obj.plotv( j,vidx(1) ), ...
                                obj.plotv( j,vidx(2) ) );                                   % Local NED Axes in ECEF Coordinates
                            UsN = GEO.NED2DIS( UsN,'E2N',...
                                obj.plotv( is,vidx(1) ), ...
                                obj.plotv( is,vidx(2) ) );                                   % Local NED Axes in Initial NED Coordinates

                            q = quiverPlot( hAx{1},Rgt(:,j),UsN*scl,i );                    % Plot Local NED and Origin on the ground track
                            % Trajectory
                            UsB = AC.body2NED(vers(:,i),'B2N',obj.plotv( j,vidx(4) ),...
                                obj.plotv( j,vidx(5) ),obj.plotv( j,vidx(6) ) );            % Body in Local NED
                            UsB = GEO.NED2DIS(UsB,'N2E',obj.plotv( j,vidx(1) ), ...
                                obj.plotv( j,vidx(2) ) );                                   % Body in ECEF Coordinates
                            UsB = GEO.NED2DIS( UsB,'E2N',...
                                obj.plotv( is,vidx(1) ), ...
                                obj.plotv( is,vidx(2) ) );                                   % Body in Initial NED
                            q = quiverPlot( hAx{1},Rs(:,j),UsB*scl,i );            
                        end
                    end
                    hAx{1}.ZDir = "reverse"; % Point Z axis up
                    hAx{1}.XDir = "reverse"; % Point Z axis up
                    set( hAx{1},'FontSize',16,'FontName','Times New Roman' );
                    lms = nan(1,6);
                    lms([1,3,5]) = min([Rgt';Rs']); lms([2,4,6]) = max([Rgt';Rs']);
                    axis( lms )
                    axis image
                    % hAx{1}.XLabel.FontSize = obj.fontdim.FLAB;
                    % hAx{1}.XLabel.FontName = 'Times New Roman';
                    % hAx{1}.
                % % 
                % % case 'NED'
                % % % Trajectory in Initial NED assumed as inertial frame
                % % %   INPUT
                % % %   - x,y,z: coordinates of the CG in the NED
                % % %   - psi,theta,phi: Euler angles from Body to NED
                % % % --------------------------------------------------- %
                % %     addParameter(p,'x',0,@isnumeric)
                % %     addParameter(p,'y',0,@isnumeric)
                % %     addParameter(p,'z',0,@isnumeric)
                % % 
                % %     parse(p,kind,GEO,varargin{:})
                % % 
                % % 
                % % 
                % % 
                % % 
                % % 
                % % 
                % % 
                % %     axG(2).ZDir = 'reverse';
            end
            % axis( [ min([Rs(1,:);Rgt(1,:)]),max([Rs(:,1);Rgt(:,1)]),...
            %     min([Rs(:,2);Rgt(:,2)]),max([Rs(:,2);Rgt(:,2)]),...
            %     min([Rs(:,3);Rgt(:,3)]),max([Rs(:,3);Rgt(:,3)])] );
        end

        function p = plotting3D(obj,plotv,xidx,yidx,zidx,ax,lst)

            p = plot3( ax,plotv(:,xidx),plotv(:,yidx),plotv(:,zidx),...
                'LineStyle',lst,'LineWidth',1.5 );

        end

        function idxs = plotidx(obj,xidx,yidx)
            dts = obj.plotv(:,yidx)*nan; ddts = dts;
            for i = 2:length(obj.plotv(:,1))
                dts(i,:) = abs( obj.plotv(i,yidx) - obj.plotv(i-1,yidx) )/...
                    ( obj.plotv(i,xidx) - obj.plotv(i-1,xidx) );
                ddts(i,:) = abs( dts(i,:) - dts(i-1,:) ) / ( obj.plotv(i,xidx) - obj.plotv(i-1,xidx) );
            end
            dts = 1 - dts./max(dts);
            ddts = 1 - ddts./max(ddts);
            xlast = obj.plotv(1,xidx);
            idxs(1) = 1;
            k = 2;
            for i = 1:length(obj.plotv(:,1))

                if ( any(ddts(i,:)<0.8,2) || (obj.plotv(i,xidx) - xlast) > 60 ) && ...
                        (obj.plotv(i,xidx) - xlast) > 10
                    idxs(k) = i;
                    k = k + 1;
                    xlast = obj.plotv(i,xidx);
                end
            end
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

