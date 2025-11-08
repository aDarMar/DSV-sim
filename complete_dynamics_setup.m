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
    GEO = GeoClass();
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

    function dxdt = CompDynNLCon(t,x,AC,y_way,bounds)
    %COMPDYNNLCON Summary of this function goes here
    %   x: state vector + auxiliary variables 
    %       [Va.ga,h,m,CL,T,Vd,Psi,Phi,p,mu,lng]
    
        % Recover data from previous successful time step
        addt = x_add();                                                             % [ID,VIAS,dVd/dt,x(7),Kh]
        % Calculate Wind
        Vw = WindMap(x);
        % Define current state output
        y = CompleteDynNoLin_Out(x,Vw,gm);
        % Controlled Outputs
        [uct,uout,dxdt_c] = LongControlOut(t,x,y_way(1:4),addt,bounds,AC,y(1:4));   % Longitudinal Control
        a = LatDirControlOut(t,x,y,y_way,AC);                                       % Latero-Directional COntrol

        dxdt = LonDynNoLin(t,x,uct,AC);                                             %[Va,ga,h,m,-,-,-,psi,phi,p]

        dxdt(5:7) = dxdt_c;

       % out_step = out_step();
       % out_step = storefun([out_step;[t,uout(:)',uct(:)',addt(1),addt(2)]]); % [ID,VIAS,hdotc,Kh,Vd]

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
                    %[ID,VIAS,hdotc,Kh,Vd,CL,T]
                    [uld,uadd] = LatDirControlOut(t,x,y_way,x_add,bounds,AC,GEO);

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


        end

    end
    %% Simulation
    CHS = 'waypoint'; 
    options = odeset('RelTol',1e-6,'AbsTol',1e-5);%,'OutputFcn',@myout);%,...
   %'Events',@events);

   switch CHS
       case 'ctrl'
            
        case 'waypoint'
            addata = nan(1,6);         
            out_store = storefun(nan(1,7)); % CONTROLLARE SE FUNZIONANO E INIZIALIZZARE
            out_step = storefun(nan(1,10));     % ID,Vc,hdotc,Kh,Vd,CL,T,x*,x*@dt<0
            % x: [Va.ga,h,m,CL,T,Vd,Psi,Phi,p,mu,lng]
            tstt = 1;
            switch tstt
                case 1
                    % Two segments
                    Vs = [120,120,120];
                    gas = [0,0,0];
                    hs = [4500,4500,4500];
                    ms = ones(3,1)*15400;

                    ways(1).ID = "Init"; ways(2).ID = "T2F"; ways(3).ID = "C2F";
                    i = 1; ways(i).lat = 35.33; ways(i).lng = 17.34;
                    i = 2; ways(i).lat = 45.58; ways(i).lng = 27.93;
                    i = 3; ways(i).lat = 50.68; ways(i).lng = 26.41;
                    ways(i).Az = 110;
            end
            Tfin = 1500;                                            % Final time
            te = 0;                                                 % Starting time
            [x_way,y_way,ye,bounds] = BuildWaypoint(GEO,AC,Vs,gas,hs,...
                ms,ways,te,true);                                   % Initial condition
            
            %ye = x_way(:,1);                                        


            %[~,~,~,ye(5:6)] = AC.LongLinSys(x_way(:,1),x_way(4,1)); % CL,T at t0
            

            tres = []; xres = [];                                % Storage vectors
            options.OutputFcn = @myout;                          % Set output function TODOOOO CHANGE FUN
            nway = length( x_way(1,:) );
            % TEMPPP store_way = nan(nway,11); store_way(:,4:7) = x_way';

            %y_str = LongDynNoLin_Out( x_way(:,1) ); 
            %temp = [0,y_str(1),zeros(1,2),-1]; % [ID,VIAS,dVd/dt,x(7),Kh]
            %x_add = storefun( temp );%[te;te;0;-1;0;zeros(8,1)]);

            for iway = 2:nway

                if iway > 2
                    % Assigns the initial value for thrust in It
                    %AC.Thrust_Law(x(6),x(3))
                    temp = out_store();
                    ye(5) = temp(end,6); % CL last
                    ye(6) = temp(end,7); % CL last
                    y_way = LongDynNoLin_Out( x_way(:,iway) );      % Defining the y desired output. The bounds are in kts and ft
                    bounds = UpdateBounds( x_way(:,iway) );
                end

                wayevent =@(t,x)wayReachedEvent(t,x,y_way);         % Defining the termination event for the ith waypoint
                options.Events = wayevent;                          % Updates events function with new waypoint
                options.OutputFcn = @myout;
                [t,x,te,ye,ie] = ode113( @(t,x)CompDynNLCon( t,x,AC,y_way,bounds ),...
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