close all; clear; clc;
addpath('.\Data')
addpath('.\Classes')
nmdbs = 'aero_synt_database';
pth = strcat(nmdbs,'.mat');
actest = ACclass(pth);

CL = -0.2:0.05:1.5;
CD = actest.polar(0.32,11.5e6,CL);

plot(CL,CD)