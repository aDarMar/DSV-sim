close all; clear; clc;
addpath('.\Data')
addpath('.\Classes')
nmdbs = 'aero_synt_database';
pth = strcat(nmdbs,'.mat');
actest = ACclass(pth);

CL = -0.2:0.05:1.5;
CD = actest.polar(0.32,11.5e6,CL);

alpha = -15:20; M = 0.1:0.1:0.5;
h = 0:500:8000;
fig(1) = figure();
ax(1) = subplot( 3,1,1,'parent',fig(1) );
ax(2) = subplot( 3,1,2,'parent',fig(1) );
ax(3) = subplot( 3,1,3,'parent',fig(1) );
fig(2) = figure();
axx(1) = subplot( 3,1,1,'parent',fig(2) );
axx(2) = subplot( 3,1,2,'parent',fig(2) );
%% Alpha Sweeps
nM = length(M); nH = length(h);
% for iM = 1:nM
%     for iH = 1:nH
%         Re = actest.ReCalc(h(iH),M(iM));
%         CL = actest.CL(M(iM),Re,alpha);
%         CD = actest.polar(M(iM),Re,CL);
%         CM = actest.CM(M(iM),Re,alpha);
%         CLda = actest.CLad(M(iM),Re,alpha);
%         CMda = actest.CMad(M(iM),Re,alpha);
%         lin = plot( ax(1),alpha,CL );
%         lin = plot( ax(2),alpha,CM );
%         lin = plot( ax(3),CL,CD );
% 
%         lin = plot( axx(1),alpha,CLda );
%         lin = plot( axx(2),alpha,CMda );
% 
%     end
% end
% plot(CL,CD)

x0 = [150,0,0,0,0,0,0,0,-2000,0,2*pi/180,0,15400];
actest.LinSystComp(x0);
