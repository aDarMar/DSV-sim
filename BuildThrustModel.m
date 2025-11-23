close all; clear; clc

%[Teta75,J,CT,CP,Eta]
PRP = readtable('DATA\PropCurv.csv');
PRD = table2array(PRP);
PRD = [PRD(1,:);PRD]; PRD(1,2) = 0; PRD(1,5) = 0;

RPM = 1212; % RPM
nrd = RPM/60;
D = 3.952; % m

V = PRD(:,2)*D*RPM;
j = 2; n(1) = 1;
Beta(1) = PRD(1,1);
for k = 1:length(PRD(:,1))-1
   if PRD(k,1) ~= PRD(k+1,1)
      Beta(j) = PRD(k+1,1);
      n(j) = k+1;
      j = j + 1;
   end
end
n = [n,length(PRD(:,1))];
J0 = [nan,nan]; PMax = []; J0old = PRD(1,2)-0.1;
for b = 2:length(Beta)
    J0(1) = max( PRD(n(b-1),2),PRD(n(b),2) );
    J0(2) = min( PRD(n(b)-1,2),PRD(n(b+1)-1,2) );
    fnZ =@(J) interp1(PRD(n(b-1):n(b)-1,2),PRD(n(b-1):n(b)-1,5),J,'linear')...
        - interp1(PRD(n(b):n(b+1)-1,2),PRD(n(b):n(b+1)-1,5),J,'linear');
    J0 = fzero(fnZ,J0);
    
    Temp = PRD(PRD(:,1)==Beta(b-1),:);
    Temp = Temp(Temp(:,2)<J0,:);
    Temp = Temp(Temp(:,2)>J0old,:);
    J0old = J0;
    PMax = [PMax;Temp];
    
    
end
% Temp = PRD(PRD(:,1)==Beta(b),:);
% Temp = Temp(Temp(:,2)>J0,:);
% 
% PMax = [PMax;Temp];

Temp = PRD(PRD(:,1)==Beta(b),:);
Temp = Temp(Temp(:,2)>J0,:);

PMax = [PMax;Temp];
fig = figure();
ax = axes('Parent',fig);
plot(ax,PRD(:,2),PRD(:,5),'ob'); hold( ax,'on' );
plot(ax,PMax(:,2),PMax(:,5),'*r');

fig(2) = figure(); ax(2) = axes('Parent',fig(2));
plot(ax(2),(PMax(:,2)*D*nrd).^3,PMax(:,5),'or'); hold(ax(2),'on');

%idxs = [1,11,26,30,38,93];
idxs = [1,5,30,95]
for k = 1:length(idxs)-1
    V = (PMax(idxs(k):idxs(k+1),2)*nrd*D);
    etas = PMax(idxs(k):idxs(k+1),5).*(1+0.008*(V*3.6/100).^2 );
    p(k,:) = polyfit(V.^3,etas,1);
    
    plot( ax(2),V.^3,p(k,1)*V.^3+p(k,2),'--r' )
    plot( ax(2),V.^3,etas,'*b' )
    plot( ax(1),V/(nrd*D),p(k,1)*V.^3+p(k,2),'--r' )
end

prop_data_simp = [p(end,:),V(1)/(nrd*D),V(end)/(nrd*D)];

V = (PMax(:,2)*nrd*D);
pp = polyfit(V,PMax(:,5),5);
plot( ax(1),V/(nrd*D),polyval(pp,V),'-.b' )

prop_data_comp = pp;

%% Save Results
SVF = false;
if SVF
   save('Data\propeller_data.mat','prop_data_comp','prop_data_simp') 
end
