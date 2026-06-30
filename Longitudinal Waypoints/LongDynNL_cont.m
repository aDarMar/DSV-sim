function dxdt = LongDynNL_cont(t,x,AC,x_way)
%UNTITLED4 Summary of this function goes here
%   x: state vector + auxiliary variables [Va,ga,h,Icl,It,wayReach,iWay,Khdot]
%global wayReach iWay
if x(6) == 1                                 % i-th waypoint reached
    x(7) = x(6) + 1;                        % looks for new waypoint
    bounds = UpdateBounds( x_way(:,iWay) );
    x(6) = -1;
end
uct = ControlOut(t,x,x_way,bounds);
dxdt = LonDynNoLin(t,x,uct,AC);

end