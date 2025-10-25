function [outputArg1,outputArg2] = LongControlOut(t,x,x_way,bounds)
%LongControlOut Summary of this function goes here
%   INPUT
%   - bounds: [vbound,hbound,V/g] in SI units
global wayReach
y = LongDynNoLin_Out(x);            % State output
y_way = LongDynNoLin_Out(x_way);    % Reference state output
err = y_way(:) - y(:);              % Error definition
if norm(err,'inf') < tol            % Waypoint reached
    wayReach = true;
end
flg = ZoneIdf(err,bounds);          % Identifies the zone in which the aircraft is                 Flag vector [ slow,fast,Low,High,lowenergy]
switch flg
    case 7
        % Thrust and CL control 
        

end
end

function ID = ZoneIdf(err,bounds)
%flg = [0,0,0,0,0];                  % Flag vector
%g = 9.81;                           % Gravity [m/s^2]
%Vbound = 5.144;                     % 10 kts

%hbound = 152.4;                     % 500 ft
if err(1) > bounds(1)
    % Aircraf is Slow
    if err(3) > bounds(2)
        % Aircraft is Low
        ID = 1;
    elseif err(3) < - bounds(2)
        % Aircraft is High
        ID = 25;
    else
        % Aircraft is neither High nor Low
    %elseif err(3) > - bounds(3)*err(1) % Check, dovrebbe essere low emergy = true
        ID = 2;
    %else
        %error('No flight region found')
    end
else
    % Aircraft is not slow
    if err(1) < -bounds(1)
        % Aircraft is Fast
        if err(3) > bounds(2)
            % Aircraft is Low
            ID = 55;
        elseif err(3) < -bounds(2)
            % Aircraft is High
            ID = 4;
        else
            % Aircraft is neither Low nor High
            ID = 5;
        end
        % Aircraft is not Fast
    elseif err(3) > bounds(2)
        % Aircraft is Low
        ID = 6;
    elseif err(3) < -bounds(2)
        % Aircraft is High
        ID = 3;
    else
        % Aicraft is neither Low nor High
        ID = 7;
    end
end
end





% if err(1) < -Vbound
%     % Aircraft is Fast: Vref - V < -Vbound
%     flg(2) = true;
%     if
% end
% if err(1) > Vbound
%     % Aircraft is Slow
%     flg(1) = true;
% end
% if err(3) < -bounds(2)
%     % Aircraft is High
%     flg(3) = true;
% end
% if err(3) > hboun
%     % Aircraft is Low
%     flg(4) = false;
% end
% if err(3) > - x_way(1)/g*err(1)
%     % Aircraft is High Energy
%     flg(5) = true;
% end
% end
