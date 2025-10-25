close all; clear; clc

addpath('Functions\')
addpath('Longitudinal Waypoints\')

global wayReach
%% Aircraft Definition

%% Controllers Gains

%% Waypoints
% Per ora sono le grandezze sono nel sistema imperiale

xway = [ 728/3.3; 1; 25000/3.3]; % Vias [kts] h [ft] hdot [ft/min]
y = LongDynNoLin_Out(xway); UpdateBounds(y);
wayReach = false; 
[t,x] = ode45( @(t,x)LongDynNLwps(t,x,xway),[0,Tfin],xway(:,1) );