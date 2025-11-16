function [tlf,xlf] = EvalSys(Tfin,Ai,Bi,Ci,K1,K2,dx0)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[tlf,xlf] = ode45(@(t,x)LongDynLin(t,x,Ai,Bi,Ci,3,K1,K2),[0,Tfin],dx0);%ode45(@(t,x)LongDynLin(t,x,Acl,Bcl,Ci,2),[0,Tfin],dx);
xdot = nan(5,length(tlf));
yout = nan(4,length(tlf)); frc = nan(4,length(tlf));
for i = 1:length(tlf)
    xdot(:,i) = LongDynLin(tlf(i),xlf(i,:),Ai,Bi,Ci,2);
    yout(:,i) = Ci*xlf(i,:)';
    frc(:,i) = IptFor(tlf(i),2);
end
figure()
subplot(2,1,1); plot(tlf,yout(4,:),'-k',tlf,frc(4,:),'-r' )
end