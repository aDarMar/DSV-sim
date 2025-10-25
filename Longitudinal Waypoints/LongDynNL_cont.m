function dxdt = LongDynNL_cont(t,x,AC,x_way)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
global wayReach iWay
if wayReach                                 % i-th waypoint reached
    iWay = iWay + 1;                        % looks for new waypoint
    bounds = UpdateBounds( x_way(:,iWay) );
    wayReach = false;
end
uct = ControlOut(t,x,x_way,bounds);
dxdt = LonDynNoLin(t,x,uct,AC);

end