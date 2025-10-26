close all; clear; clc

addpath('Functions\')
addpath('Longitudinal Waypoints\')

main()
% We define main function to allow the nested functions definition to store
% additional data through the integration
function main()  
    %% Aircraft Definition
    
    %% Controllers Gains
    
    %% Waypoints
    % Per ora sono le grandezze sono nel sistema imperiale

    
    function st = storefun(x_store)
    % Nested function used to store some values and recalling it during ode
    % integration. Calling storefun(x) returns a pointer to a function that
    % returns the value stored when the function was first called.
        st = @store;
        function stored = store()
            stored = x_store;
        end
    end

    addata = [-1;1;-1;0]; %[wayReach;iway;Khdot,tlast]
    x_add = storefun(addata);
    out_temp = storefun(addata); % CONTROLLARE SE FUNZIONANO E INIZIALIZZARE
    out_step = storefun(addata);
    %% ODE RHS definition

    function dxdt = LongDynNL_cont(t,x,AC,y_way,bounds)
    %LongDynNL_cont Summary of this function goes here
    %   x: state vector + auxiliary variables [Va,ga,h,Icl,It]
    %global wayReach iWay
        addt = x_add();                                 % [wayReach;iway;Khdot,tlast]
        %stdata = out_step()
        % if addt(1) == 1                                    % i-th waypoint reached
        %     addt(2) = addt(2) + 1;                            % looks for new waypoint
        %     bounds  = UpdateBounds( x_way(:,iWay) );
        %     addt(1) = -1;
        % end
        [uct,uout,dxdt_c] = LongControlOut(t,x,y_way,bounds);
        dxdt = LonDynNoLin(t,x,uct,AC);
        x_add = storefun(addt);
        out_temp = storefun(uout); % CREARE UN VETTORE DI OUTPUT DA SALVARE
        dxdt = [dxdt(:);dxdt_c(:)];
    end

    % event function: stops the integration when a waypoint is reached
    function [position,isterminal,direction] = wayReachedEvent(t,x,y_way)
        stop = 10;
        y = LongDynNoLin_Out(x);
        err = y_way(:) - y(:); tol = 1;
        %err = 1 - y(:)./y_way(:); % Relative error
        %err = err(isnan(err)); % Removes nan like commanded values equal to 0
        if norm(err,'inf') < tol
            stop = 0;   % stop is set equal to 0 if the maximum error is within tolerance
        end
        position = stop;    % When stop = 0 the integration stops
        isterminal = 1;     % Halt integration
        direction = 0;      % Zero can be approached from either directions
    end
    
    % output function: it is called by the ode solver only after a
    % successful time step integration
    function status = myout(t, y, flag)
    %https://stackoverflow.com/questions/38120741/using-persistent-variable-to-pass-out-extra-parameters-using-ode45-from-the-matl
        % Don't ever halt the integration
        status = 0;
        switch flag
            case "init"
            % Initialization (apparently useless).
                param = -2;
            case "done"
                % Nothing to do.
            otherwise
            % Main code to update param.
                out = out_step(); % Retrives the successfull output vector up to now
                outup = out_temp();% Retrives the current timestep output
                out = [out;outup(:)']; out_step = storefun(out); % Updates teh successful output step vector

        end

    end

    %% Simulation

    options = odeset('RelTol',1e-5,'AbsTol',1e-4,'OutputFcn',@myout);%,...
   %'Events',@events);
    x_way = [ 728/3.3; 1; 25000/3.3]; % Vias [kts] h [ft] hdot [ft/min]

    te = 0;                                              % Starting time
    ye = x_way(:,1);                                     % Initial condition
    tres = []; xres = [];                                % Storage vectors
    for iway = 2:length( x_way(1,:) )
    % event function: stops the integration when a waypoint is reached. It
    % is defined in the loop so that is SHOULD update the terminator
    % condition at each waypoint
        y_way = LongDynNoLin_Out( x_way(:,iway) );      % Defining the y desired output
        bounds = UpdateBounds( x_way(:,iway) );
        wayevent =@(t,x)wayReachedEvent(t,x,y_way);     % Defining the termination event for the ith waypoint
        options.Events = wayevent;                      % Updates events function with new waypoint
        [t,x,te,ye,ie] = ode45( @(t,x)LongDynNL_cont( t,x,AC,y_way,bounds ),...
            [te,Tfin],ye,options );
        tres = [tres;t]; xres = [xres;x];               % Saves the ith waypoint results
    end
end