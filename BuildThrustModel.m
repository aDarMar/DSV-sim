close all; clear; clc

%[Teta75,J,CT,CP,Eta]
%% Reading Data
PRP = readtable('DATA\PropCurv.csv');
PRD = table2array(PRP);
PRD = [PRD(1,:);PRD]; PRD(1,2) = 0; PRD(1,5) = 0;

RPM = 1212;                 % Propeller RPM
nrd = RPM/60;               % Round per second
D = 3.952;                  % Propeller's Diameter [m]

% Finding the Number of beta_075 
j = 2; n(1) = 1;
Beta(1) = PRD(1,1);
for k = 1:length(PRD(:,1))-1
   if PRD(k,1) ~= PRD(k+1,1)
      Beta(j) = PRD(k+1,1);
      n(j) = k+1;
      j = j + 1;
   end
end
% Find the Maximum etas accross al beta_075
n = [n,length(PRD(:,1))];
J0 = [nan,nan]; PMax = []; J0old = PRD(1,2)-0.1;
for b = 2:length(Beta)
    J0(1) = max( PRD(n(b-1),2),PRD(n(b),2) );
    J0(2) = min( PRD(n(b)-1,2),PRD(n(b+1)-1,2) );
    % Intersection between eta( beta_075(i-1) ) and eta( beta_075(i) )
    fnZ =@(J) interp1(PRD(n(b-1):n(b)-1,2),...
        PRD(n(b-1):n(b)-1,5),J,'linear')...
        - interp1(PRD(n(b):n(b+1)-1,2),PRD(n(b):n(b+1)-1,5),J,'linear');
    J0 = fzero(fnZ,J0); 
    
    Temp = PRD(PRD(:,1)==Beta(b-1),:);
    Temp = Temp(Temp(:,2)<J0,:);
    Temp = Temp(Temp(:,2)>J0old,:);
    J0old = J0;
    PMax = [PMax;Temp];
    
end

Temp = PRD(PRD(:,1)==Beta(b),:);
Temp = Temp(Temp(:,2)>J0,:);

PMax = [PMax;Temp];

plotv = nan( length( PMax(:,1) ),5 );
plotv(:,[1,4]) = [ PMax(:,2),PMax(:,5) ];
% [ J,V,V^3, eta(BEMT),eta(reg 1), eta(reg 2) ]

%% Piecewise Cubic Regerssion
% Eta Kv = a0 + a3 V^3
%idxs = [1,11,26,30,38,93];
idxs = [1,5,30,95]      % Indices of J interval
for k = 1:length(idxs)-1
    V = ( PMax(idxs(k):idxs(k+1),2)*nrd*D );
    etas = PMax(idxs(k):idxs(k+1),5).*(1+0.008*(V*3.6/100).^2 );
    p(k,:) = polyfit(V.^3,etas,1);      % Third order polinomial
    
    plotv( idxs(k):idxs(k+1),5 ) = p(k,1)*V(:).^3 + p(k,2);
    plotv( idxs(k+1),5 ) = nan; plotv( idxs(k),5 ) = nan;

end

V = (PMax(:,2)*nrd*D);
pp = polyfit(V,PMax(:,5),5);

plotv(:,6) = [ polyval( pp,V(:) ) ];
%% Global Regression

prop_data_simp = [p(end,:),V(1)/(nrd*D),V(end)/(nrd*D)];
prop_data_comp = pp;

%% Graphics
%plot( ax(1),V/(nrd*D),polyval(pp,V),'-.b' )
GRF = true;
if GRF
    plotv(:,2:3) = [ V(:),V(:).^3 ];
    % [ J, V, V^3, eta(BEMT), eta(reg 1), eta(reg 2) ]
    figS = [ [1;2],[1;3],...
         [ [4:6];[4:6] ] ];
    %
    IMTIT = {'Stability Derivatives','Propeller'};
    LG = repmat({'$\eta_P$ BEMT','$\eta_P K_V$ Pol. 3$^\circ$ Grado','$\eta_p$ Polinomio 6$^\circ$ grado'},2,1);

    TIT = {'a','b'}; TITP = {'-','-'};
    LINS = repmat({'o','--','-.'},2,1);
    XLAB = {'J','$V^3$ [m/s]'}; YLAB = repmat({'$\eta_{P}, \eta_P K_V$'},2,1);


    plotd = DataPlot(plotv);
    for ig = 1:length(XLAB)
        plotd = plotd.definePlot(figS(ig,2),figS(ig,3:end),figS(ig,1),'legend',LG(ig,:),...
            'grid','minor','xlabel',XLAB{ig},'title',TITP{ig},...
            'ylabel',YLAB{ig},'linestyle',LINS(ig,:));
    end

    plotd.PlotPerImag( 3,IMTIT,...
        TITP );
end

%% Save Results
SVF = false;
if SVF
   save('Data\propeller_data.mat','prop_data_comp','prop_data_simp') 
end
