close all; clear; clc
% BUILDTHRUSTMODEL: This script processes raw propeller curve data (J, Eta, etc.) 
% to determine the envelope of maximum efficiency (EtaMax) across different 
% blade pitch angles (Beta). It then applies piecewise linear regression 
% (against V^3) and global polynomial regression (against V) to model the 
% propeller efficiency curves. Finally, it prepares data for plotting and 
% optionally saves the regression coefficients.
% INPUTS:
%   PRP: Table loaded from 'DATA\PropCurv.csv' containing Beta_075, J, CT, CP, Eta.
%   RPM: Propeller Rotational Speed (Scalar, 1212 used).
%   D: Propeller Diameter (Scalar, 3.952 used).
%
% OUTPUTS:
%   prop_data_simp: Coefficients [a3, a0] from the final segment of the 
%                   piecewise linear regression (Eta*Kv = a0 + a3*V^3), 
%                   plus J_min and J_max.
%   prop_data_comp: Coefficients of the 5th-degree global polynomial regression (pp).
%   plotv: Matrix containing all original and modeled data points for plotting.
%
% ----------------------------------------------------------------------- %


%% --------------------------------------------------------------------- %% 
% Reading Data
% Reads the propeller curve data from a CSV file.
%[Teta75,J,CT,CP,Eta]
PRP = readtable('DATA\PropCurv.csv');
PRD = table2array(PRP);
PRD = [PRD(1,:);PRD]; PRD(1,2) = 0; PRD(1,5) = 0;

RPM = 1212;                 % Propeller RPM
nrd = RPM/60;               % Round per second
D = 3.952;                  % Propeller's Diameter [m]

% Finding the Number of beta_075 and their starting indices
j = 2; n(1) = 1;
Beta(1) = PRD(1,1);
for k = 1:length(PRD(:,1))-1
    % Checks if the Beta_075 pitch angle changes between row k and k+1.
   if PRD(k,1) ~= PRD(k+1,1)
      Beta(j) = PRD(k+1,1); % Stores the new pitch angle.
      n(j) = k+1;           % Stores the starting index for the new data set (new Beta).
      j = j + 1;
   end
end
% Find the Maximum etas accross all beta_075
n = [n,length(PRD(:,1))];       % Adds the last index of the data to the array n.
J0 = [nan,nan]; PMax = []; J0old = PRD(1,2)-0.1;
% Loops through all Beta pitch angles, starting from the second one.
for b = 2:length(Beta)
    % Defines an initial interval for the root-finding function (fzero).
    J0(1) = max( PRD(n(b-1),2),PRD(n(b),2) );
    J0(2) = min( PRD(n(b)-1,2),PRD(n(b+1)-1,2) );
    % Intersection between the efficiency curve eta( Beta_075(i-1) ) and eta( Beta_075(i) ).
    % Defines the function whose root is the intersection point J0:
    % fnZ = eta(Beta_i-1) - eta(Beta_i) @ given J
    fnZ =@(J) interp1(PRD(n(b-1):n(b)-1,2),...
        PRD(n(b-1):n(b)-1,5),J,'linear')...
        - interp1(PRD(n(b):n(b+1)-1,2),PRD(n(b):n(b+1)-1,5),J,'linear');
    % Finds J where the two efficiency curves intersect.
    J0 = fzero(fnZ,J0); 
    % Extracts data for the previous Beta (b-1).
    Temp = PRD(PRD(:,1)==Beta(b-1),:);
    % Keeps only the points where J is less than the intersection point J0.
    Temp = Temp(Temp(:,2)<J0,:);
    % Keeps only the points where J is greater than the previous J0 (J0old).
    Temp = Temp(Temp(:,2)>J0old,:);
    J0old = J0;
    % Save the eta max for the current beta075
    PMax = [PMax;Temp];
end
% Handles the data for the last Beta
Temp = PRD(PRD(:,1)==Beta(b),:);
Temp = Temp(Temp(:,2)>J0,:);
PMax = [PMax;Temp];

plotv = nan( length( PMax(:,1) ),8 );
plotv(:,[1,4]) = [ PMax(:,2),PMax(:,5) ];
% [ J,V,V^3, eta(BEMT),eta(reg 1), eta(reg 2) ]

%% --------------------------------------------------------------------- %% 
% Piecewise Cubic Regerssion
% Eta Kv = a0 + a3 V^3

% Indices defining the J intervals for the piecewise regression.
idxs = [1,5,30,95];     
for k = 1:length(idxs)-1
    % Calculates the flight velocity V [m/s] for the current interval: V = J * nrd * D.
    V = ( PMax(idxs(k):idxs(k+1),2)*nrd*D );
    % Corrects the efficiency with the Kv factor
    etas = PMax(idxs(k):idxs(k+1),5).*(1+0.008*(V*3.6/100).^2 );
    % Performs the linear regression between KVV^3 and etas
    % p(k,1) is the coefficient a3, p(k,2) is the constant term a0.
    p(k,:) = polyfit(V.^3,etas,1);      
    
    % Stroing Data for Graphics
    plotv( idxs(k):idxs(k+1),5 ) = p(k,1)*V(:).^3 + p(k,2);
    plotv( idxs(k+1),5 ) = nan; plotv( idxs(k),5 ) = nan;

end
% Save Coefficients
prop_data_simp = [p(end,:),V(1)/(nrd*D),V(end)/(nrd*D)];
%% --------------------------------------------------------------------- %% 
% GLOBAL REGRESSION
% Calculates the flight velocity V for all EtaPMax points.
V = (PMax(:,2)*nrd*D);
% Performs a global 5th-degree polynomial regression on Eta(V).
pp = polyfit(V,PMax(:,5),5);
% Save Coefficients
prop_data_comp = pp;
% Store Data for Graphics
plotv(:,6) = [ polyval( pp,V(:) ) ];
%% --------------------------------------------------------------------- %% 
% GRAPHICS GENERATION (Conditional)
GRF = true;
if GRF
    plotv(:,2:3) = [ V(:),V(:).^3 ]; 
    plotv = [plotv;nan(length( PRD(:,1) )-length( plotv(:,1)),8 )];
    plotv(:,7:8) = PRD(:,[2,5]);
    % [ J, V, V^3, eta(BEMT), eta(reg 1), eta(reg 2), etas ]
    Aux = nan( 2,max(1,length(Beta)) );
    figS = [ [1;2;3],[1;3;7],...
         [ [4:6];[4:6];[8,nan(1,2)] ] ];
    %
    IMTIT = {'Stability Derivatives','Propeller','Propeller Efficiency'};
    LG = repmat({'$\eta_P$ BEMT','$\eta_P K_V$ Pol. 3$^\circ$ Grado','$\eta_p$ Polinomio 6$^\circ$ grado'},3,1);
    LG(3,:) = repmat({'-'},1,3);
    TIT = {'a','b'}; TITP = {'-','-','-'};
    LINS = repmat({'o','--','-.'},3,1);
    LINS(3,:) = repmat({'o'},1,3);
    XLAB = {'J','$V^3$ [m/s]','J'}; YLAB = repmat({'$\eta_{P}, \eta_P K_V$'},3,1);
    YLAB(3) = {'$\eta_P$'};
    
    plotd = DataPlot(plotv);
    for ig = 1:length(XLAB)
        yidx = figS( ig,( ~isnan( figS(ig,:) ) ) );
        plotd = plotd.definePlot(figS(ig,2),yidx(3:end),figS(ig,1),'legend',LG(ig,:),...
            'grid','minor','xlabel',XLAB{ig},'title',TITP{ig},...
            'ylabel',YLAB{ig},'linestyle',LINS(ig,:));
    end

    plotd.PlotPerImag( 3,IMTIT,...
        TITP,'cartesian',false );
end

%% --------------------------------------------------------------------- %%  
% Save Results
SVF = false;
if SVF
   save('Data\propeller_data.mat','prop_data_comp','prop_data_simp') 
end
