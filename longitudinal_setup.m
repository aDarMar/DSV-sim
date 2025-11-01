close all; clear; clc

addpath('Functions\')
addpath('Longitudinal Waypoints\')
addpath('Classes')
addpath('Data')

main()
% We define main function to allow the nested functions definition to store
% additional data through the integration
function main()  
    CfrLin = true;
    %% Aircraft Definition
    nmfl = "Q100.aero"; AC = ACclass(nmfl);
    
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

    %addata = [T0,T0]; %[wayReach;iway;Khdot,tlast]
    %x_add = storefun(addata);
    addata = nan(1,6);         % ID,Vc,hdotc,Kh,CL,T
    out_store = storefun([nan,addata]); % CONTROLLARE SE FUNZIONANO E INIZIALIZZARE
    out_step = storefun([nan,addata]);
    %% ODE RHS definition
    
    function dxdt = LongDynNLConSimp(t,x,AC,y_way)
    %   x: state vector + auxiliary variables [Va,ga,h,Icl,It]
    %   addt: [hdotc]
        addt = x_add();                                 % [t(n-1),t@dt<0]
        if ~(t - addt(1)) > 0                           % The integration has "gone back" in time
            addt(1) = addt(2);
            addt(2) = t;
        end

        [uct,uout,dxdt_c] = LongControlOutSimple(t,x,y_way,addt,AC);
        dxdt = LonDynNoLin(t,x,uct,AC);
        addt(1) = t;
        x_add = storefun(addt);
        out_step = out_step();
        out_step = storefun([out_step;[t,uout(:)',uct(:)']]); % CREARE UN VETTORE DI OUTPUT DA SALVARE
        dxdt = [dxdt(:);dxdt_c(:)];
    end
    
    function dxdt = LongDynLinCon(t,x)
       
       dxdt =  LongDynLin(t,x,Acl,Bcl,y_way);
        
    end

    function dxdt = LongDynNL_cont(t,x,AC,y_way,bounds)
    %LongDynNL_cont Summary of this function goes here
    %   x: state vector + auxiliary variables [Va,ga,h,Icl,It]
    %global wayReach iWay
        addt = x_add();                                 % [t(n-1),t@dt<0]
        if ~(t - addt(1)) > 0                           % The integration has "gone back" in time
            addt(1) = addt(2);                          
            addt(2) = t;
        end
        %stdata = out_step()
        % if addt(1) == 1                                    % i-th waypoint reached
        %     addt(2) = addt(2) + 1;                            % looks for new waypoint
        %     bounds  = UpdateBounds( x_way(:,iWay) );
        %     addt(1) = -1;
        % end
        [uct,uout,dxdt_c] = LongControlOut(t,x,y_way,addt,bounds);
        dxdt = LonDynNoLin(t,x,uct,AC);
        addt(1) = t;
        x_add = storefun(addt);
        out_temp = storefun([uout(:)',uct(:)']); % CREARE UN VETTORE DI OUTPUT DA SALVARE
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
    function status = myoutSimple(t, x, flag)
        % Forse è inutile per questo scopo perchè passa un vettore di tempi
        % mentre la funzione di output ha bisogno dei singoli tempi a
        % causa del dt
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
            % Main code to update param
                nt = length(t);
                store_temp = nan(nt,7); % [t,ID,Vc,hdotc,Kc,CL,T]
                store = out_store(); addt = x_add();
                for it = 1 :nt
                    [uct,uout,~] = LongControlOutSimple(t,x,y_way,addt,AC);
                    store_temp(it,1) = t(it);
                    store_temp(it,2:5) = uout(:)'; store_temp(it,6:7) = uct(:)';
                end
                out_store = storefun([store;store_temp]); 
                %out = out_step(); % Retrives the successfull output vector up to now
                %outup = out_temp();% Retrives the current timestep output
                %out_step = storefun([out;outup(:)']); % Updates teh successful output step vector

        end

    end

    %% Simulation
    CHS = 'ctrl'; 
    options = odeset('RelTol',1e-6,'AbsTol',1e-5);%,'OutputFcn',@myout);%,...
   %'Events',@events);
   x_way = [ 728/3.3; 1; 25000/3.3]; % Vias [kts] h [ft] hdot [ft/min]
   switch CHS
       case 'ctrl'
           CTR = 'Vc'; % Cambia qua e la variabile TEST in LongControlOutSimple
           
           Tfin = 40;
           
           
           load('Data\test_conditions.mat','xref','x0');
           options.OutputFcn = @myoutSimple;
           % Nonlinear System
           te = 0; 
           xref = x0(36,:);
           ye = xref(1:4)'; ye = [ye;0;0];
           y_way = LongDynNoLin_Out( xref(:) );      % Defining the y desired output
           y_way = zeros(4,1);
           
           switch CTR
               case 'hc'
                   hdotc = 1000;               % ft/min RoC
                   y_way(4) = hdotc;
                   x_add = storefun([te;te;hdotc]);
                   k = 1;
               case 'Vc'
                   Vc = 200;                   % kts IAS
                   y_way(1) = Vc;
                   x_add = storefun([te;te;Vc]);
                   k = 2;
               case'b'
                   Vc = 200;
                   hdotc = 100;
                   y_way(1) = Vc;
                   y_way(4) = hdotc;
                   x_add = storefun([te;te;Vc]);
                   k = 3;
           end
            
            
            [t,x] = ode113( @(t,x)LongDynNLConSimp( t,x,AC,y_way ),...
                                [te,Tfin],ye,options );
            x_aux = out_store(); %x_aux = x_aux(2:end,:);       % Left the first nan column because the @myout function is called from the first successfull timestep after t0
            x_debug = out_step();
            
            % Linearized System
            if CfrLin
                % Compares the linear and non linear responses
                [A,B,C,u0] = AC.LongLinSys( ye,ye(4) );
                A = [A,B;zeros(2,5)]; B = [B,zeros(3,2);zeros(2,2),eye(2)];
                C = [C,zeros(4,2)];
                Acl = A - B*[AC.Kp(:,:,k)+AC.Kb(:,:,k);AC.Ki(:,:,k)]*C;
                Bcl = B*[AC.Kp(:,:,k);AC.Ki(:,:,k)];
                [tl,xl] = ode45( @LongDynLinCon,[0,Tfin],...
                    zeros(5,1) );
                plot_results(AC,'cfr',t,x,x_aux,x_debug,tl,xl,Acl,C,[ye',u0']);
            else
                plot_results(AC,CHS,t,x,x_aux,x_debug);
            end
        case 'waypoint'
            te = 0;                                              % Starting time
            ye = x_way(:,1);                                     % Initial condition
            tres = []; xres = [];                                % Storage vectors
            for iway = 2:length( x_way(1,:) )
                % event function: stops the integration when a waypoint is reached. It
                % is defined in the loop so that is SHOULD update the terminator
                % condition at each waypoint
                x_add = storefun([te;te]);
                y_way = LongDynNoLin_Out( x_way(:,iway) );      % Defining the y desired output
                bounds = UpdateBounds( x_way(:,iway) );
                wayevent =@(t,x)wayReachedEvent(t,x,y_way);     % Defining the termination event for the ith waypoint
                options.Events = wayevent;                      % Updates events function with new waypoint
                [t,x,te,ye,ie] = ode45( @(t,x)LongDynNL_cont( t,x,AC,y_way,bounds ),...
                    [te,Tfin],ye,options );
                tres = [tres;t]; xres = [xres;x];               % Saves the ith waypoint results
            end
    end
end