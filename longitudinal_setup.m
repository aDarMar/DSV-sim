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

    function dxdt = LongDynNLCon(t,x,AC,y_way,bounds)
    %LongDynNL_cont Summary of this function goes here
    %   x: state vector + auxiliary variables [Va,ga,h,Icl,It]
    %global wayReach iWay
        % flg = false;
        addt = x_add();                                 % [t(n-1),t@dt<0,dt,Kh@t(n-1),Vd@t(n-1)]

        [uct,uout,dxdt_c] = LongControlOut(t,x,y_way,addt,bounds,AC);
        dxdt = LonDynNoLin(t,x,uct,AC);

        dxdt = [dxdt(:);dxdt_c(:)];

        out_step = out_step();
        out_step = storefun([out_step;[t,uout(:)',uct(:)',addt(1),addt(2)]]); % CREARE UN VETTORE DI OUTPUT DA SALVARE

    end

    % event function: stops the integration when a waypoint is reached
    function [position,isterminal,direction] = wayReachedEvent(t,x,y_way)
        %WAUREACNEDEVENT: function that stops the integration when the
        %error is inside an ellypse in the h-IAS error plane centered
        %around 0 error.

        y = LongDynNoLin_Out(x);
        err = y_way(:) - y(:); 
        dst = ( 20*err(1)/bounds(1) )^2 + ( 15*err(3)/bounds(2) )^2 - 1; % The point must be inside an ellipse in the h-IAS plane

        position = dst;         % When stop = 0 the integration stops
        isterminal = 1;         % Halt integration
        direction = -1;         % The fnction must be decreasing
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
    
    function status = myout(t, x, flag)
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
                % Initialize the addt variables
                
                if iway == 2
                    % Executes this code only if it is teh first waypoint,
                    % otherwise it takes the values from previous
                    % integration
                    addt = x_add();        % x_add = [ID,VIAS,dVd/dt,x(7),hdot]
                    [~,uout,dxdt] = LongControlOut(t(1),x,y_way,addt,bounds,AC);
                    addt(1) = uout(1);
                    addt(2) = uout(2); addt(3) = dxdt(3);
                    addt(4) = x(7);  addt(5) = uout(4);     % Kh
                    x_add =  storefun(addt);
                end
            case "done"
                % Nothing to do.
            otherwise
            % Main code to update param
                nt = length(t);
                store_temp = nan(nt,7); % [ID,VIAS,hdotc,Kc,Vd,CL,T]
                store = out_store(); addt = x_add();        % x_add = [ID,VIAS,dVd/dt,x(7),Kh]
                for it = 1 :nt
                    [uct,uout,dxdt] = LongControlOut(t,x,y_way,addt,bounds,AC);
                    %store_temp(it,1) = t(it);
                    l1 = length(uout(:)'); l2 = length(uct(:)');
                    store_temp(it,1:l1) = uout(:)'; store_temp(it,1+l1:l1+l2) = uct(:)';
                    
                end
                
                if uout(1) ~= addt(1) || ( uout(1) ~= 3 && uout(1) ~= 6 )
                % This means that at the previous time step we were in zone
                % 3 or 6 and now we are still in the same zone
                    addt(2) = uout(2); % Update with current IAS
                end
                if addt(3) ~= 0 && dxdt(3) == 0
                    addt(4) = x(7);% Storing the value of Vd when it becomes constant
                end
                addt(1) = uout(1);
                addt(3) = dxdt(3);
                addt(5) = uout(4);% Kh
                x_add = storefun(addt); 

                out_store = storefun([store;store_temp]); 
                %out = out_step(); % Retrives the successfull output vector up to now
                %outup = out_temp();% Retrives the current timestep output
                %out_step = storefun([out;outup(:)']); % Updates teh successful output step vector

        end

    end
    %% Simulation
    CHS = 'waypoint'; 
    options = odeset('RelTol',1e-6,'AbsTol',1e-5);%,'OutputFcn',@myout);%,...
   %'Events',@events);

   switch CHS
       case 'ctrl'
           CTR = 'Vc'; % Cambia qua e la variabile TEST in LongControlOutSimple
           
           Tfin = 40;
           addata = nan(1,6);         % ID,Vc,hdotc,Kh,CL,T
           out_store = storefun([nan,addata]); % CONTROLLARE SE FUNZIONANO E INIZIALIZZARE
           out_step = storefun([nan,addata]);
           
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
               case 'b'
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
            addata = nan(1,6);         
            out_store = storefun(nan(1,7)); % CONTROLLARE SE FUNZIONANO E INIZIALIZZARE
            out_step = storefun(nan(1,10));     % ID,Vc,hdotc,Kh,Vd,CL,T,x*,x*@dt<0
            %x_way = [ 222.1,90,120;0,0,0;7625,1300,4300]; % Vias [kts] h [ft] hdot [ft/min]
            %x_way = [130,100;0,0;4000,3000;15400,15400];
            tstt = 4;
            switch tstt
                case 1
                    % Climb from zone 6 to zone 7 and then climb again
                    x_way = [120,126,125;0,0,0;3000,4000,4500;15400,15400,15400];
                    Tfin = 1600;
                    %x_way = x_way(:,1:2);
                case 2
                    % Accelerate from region 2 to region 7
                    x_way = [120,130,110;0.0,0,0;3000,3000,3000;15400,15400,15400];
                    Tfin = 1600;
                case 3
                    % Decelerate and descend from region 25
                    x_way = [110,130,110;0.0,0,0;4000,3000,4000;15400,15400,15400];
                    Tfin = 1600;
                case 4
                    % Decelerate and descend from region 25
                    x_way = [110,130,110;0.0,0,0;3000,4000,3000;15400,15400,15400];
                    Tfin = 1600;
                    %x_way = x_way(:,1:2);
            end
                                                                 % Final time
            te = 0;                                              % Starting time
            ye = [x_way(:,1);0;0;0];                             % Initial condition


            [~,~,~,ye(5:6)] = AC.LongLinSys(x_way(:,1),x_way(4,1));
            

            tres = []; xres = []; x_aux_res = [];                % Storage vectors
            options.OutputFcn = @myout;                          % Set output function TODOOOO CHANGE FUN
            nway = length( x_way(1,:) );
            store_way = nan(nway,11); store_way(:,4:7) = x_way';

            y_str = LongDynNoLin_Out( x_way(:,1) ); 
            temp = [0,y_str(1),zeros(1,2),-1]; % [ID,VIAS,dVd/dt,x(7),Kh]
            x_add = storefun( temp );%[te;te;0;-1;0;zeros(8,1)]);

            for iway = 2:nway
                % event function: stops the integration when a waypoint is reached. It
                % is defined in the loop so that is SHOULD update the terminator
                % condition at each waypoint
                
                y_way = LongDynNoLin_Out( x_way(:,iway) );      % Defining the y desired output. The bounds are in kts and ft
                bounds = UpdateBounds( x_way(:,iway) );
                
                
                if iway == 2
                    [u,temp,~] = LongControlOut(te,[x_way(:,iway-1);0;0;0],y_way,temp,bounds,AC);
                    if temp(1) ~= 7
                        ye(6) = u(2);
                    end
                else
                    % Assigns the initial value for thrust in It
                    %AC.Thrust_Law(x(6),x(3))
                    temp = out_store();
                    ye(5) = temp(end,6); % CL last
                    ye(6) = temp(end,7); % CL last
                end
                
                
                
                wayevent =@(t,x)wayReachedEvent(t,x,y_way);     % Defining the termination event for the ith waypoint
                options.Events = wayevent;                      % Updates events function with new waypoint
                options.OutputFcn = @myout;
                [t,x,te,ye,ie] = ode113( @(t,x)LongDynNLCon( t,x,AC,y_way,bounds ),...
                    [te,Tfin],ye,options );
                
                tres = [tres;t(1:end-1,:)]; xres = [xres;x(1:end-1,:)];               % Saves the ith waypoint results
                if isempty(te)
                    te = Tfin;
                end
                store_way(iway,1:3) = [te,bounds(:)'];
            end
            tres = [tres;t(end,:)]; xres = [xres;x(end,:)];
            x_aux = out_store();
            x_debug = out_step();
            plot_results(AC,CHS,tres,xres,x_aux,x_debug...
                ,[],[],[],[],[],store_way);
    end
end