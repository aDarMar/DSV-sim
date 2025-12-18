close all; clearvars; clc

addpath('Classes')
addpath('C:\Users\Ospitio\Documents\Scripts\src') % TEMPORANEO SOLO PER PLOT2TIKZ

load('Data\aero_database_long.mat')

GRF(1).fig = 1;
GRF(1).x.idx = 1; GRF(1).x.label = '$\dot{s}$';
GRF(1).y.idx = [2,3];

plotv = aero_synt(:,1:3);

A = repmat({'-'},3,2)
B = repmat({'t'},3,3);
A = [A,B]
p = DataPlot(plotv);
p.definePlot(2,[1,2],1,'legend',{'test','pst'},'xlabel','$\theta$'...
    ,'ylabel','$\alpha$',title='testo',grid='minor',linestyle={'--','o','-.'})
p.definePlot(1,[1,2,3],1);
p.definePlot(1,[1,2],2);
p.figureCounter()
TIT = {'test','toast'};
p.PlotPerImag(2,TIT,TIT);



 load clown ;

 nRows = 3 ;
 nCols = 2 ;

 % - Create figure, set position/size to almost full screen.
 figure() ;
 set( gcf, 'Units', 'normalized', 'Position', [0.05,0.1,0.8,0.8] ) ;

 % - Create grid of axes.
 [blx, bly] = meshgrid( 0.05:0.9/nCols:0.9, 0.05:0.9/nRows:0.9 ) ;
 hAxes = arrayfun( @(x,y) axes( 'Position', [x, y, 0.9*0.9/nCols, 0.9*0.9/nRows] ), blx, bly, 'UniformOutput', false ) ;

 % - "Plot data", update axes parameters.
 for k = 1 : numel( hAxes )
    axes( hAxes{k} ) ;
    image( X ) ;
    set( gca, 'Visible', 'off' ) ;
 end

 colormap( map ) ;


matlab2tikz('test_out.tex');