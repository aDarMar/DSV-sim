close all; 
clear; clc

addpath('Functions\')
addpath('Longitudinal Waypoints\')
addpath('Latero-Directional Waypoints\')
addpath('Classes')
addpath('Data')

main()
% We define main function to allow the nested functions definition to store
% additional data through the integration
function main() 
%MAIN Executes the complete aircraft trajectory simulation
%   This function initializes the simulation environment, defines the
%   aircraft model, sets up the waypoints, and manages the ODE integration
%   loop to simulate the flight trajectory under complete nonlinear
%   dynamics and control.
%
%   It uses nested functions (storefun, CompDynNLCon, wayReachedEvent,
%   myoutcomp) to share data and manage the integration process.
% ----------------------------------------------------------------------- %

    %% ----------------------------------------------------------------- %%
    % AIRCRAFT DEFINITION
    nmfl = "Q100_simp.aero"; AC = ACclass(nmfl);
    % Latero Directional modal properties 
    % [2*zeta*wn; wn^2; K]
    AC.lat = [2*1*1;1;2]; 
    GEO = GeoClass(); 
    % Load wind data map from file
    wind_data = 'Wind_Simp'; %'No_Wind';
    load( [wind_data,'.mat'],'windpx','windpy' ); 
    %% Waypoints

    
    function st = storefun(x_store)
    % STOREFUN Creates a persistent, callable storage for data
    %   ST = STOREFUN(X_STORE) returns a function handle ST. Calling ST() 
    %   will return the value of X_STORE at the time STOREFUN was called. 
    %   Used to store and retrieve data across ODE solver steps.
    % INPUT
    %  - x_store: The value (array or scalar) to be stored.
    %
    % OUTPUT
    %  - st: A function handle (pointer) that, when called, returns the 
    %        value of x_store at the moment STOREFUN was initially executed.
    % ------------------------------------------------------------------- %
        st = @store;
        function stored = store()
            stored = x_store;
        end
    end
    
    %% ODE RHS definition

    function dxdt = CompDynNLCon(t,x,AC,y_way,bounds)
    % COMPDYNNLCON Computes the derivatives for the complete nonlinear dynamics (RHS)
    %   Calculates the rate of change of the state vector by computing wind, 
    %   determining control inputs (longitudinal and lateral-directional), 
    %   and applying the full nonlinear dynamic model.
    % INPUT
    %  - t: Current time (scalar).
    %  - x: Complete state vector (11 elements) + auxiliary variables 
    %       [Va, ga, h, m, CL_i, T_i, Vd_i, Psi, Phi, p, mu, lng]
    %  - AC: Aircraft object (ACclass) containing dynamics data.
    %  - y_way: Desired output vector at the target waypoint.
    %  - bounds: Control saturation limits for the current segment.
    %
    % OUTPUT
    %  - dxdt: Derivative of the state vector, dX/dt.
    % ------------------------------------------------------------------- %
        % Initialize variables
        uct = nan(3,1); 
        % Recover data from previous successful time step
        % [ID,VIAS,dVd/dt,x(7),Kh]
        addt = x_add();                                                      
        % Calculate Wind
        Vw = WindMap(windpx,windpy,x);
        % Define current state output
        % y: [IAS, M, h, hdot, Psi, p, Phi, VGT, PsiGT]
        y = CompleteDynNoLin_Out(x,Vw);                                                        
        % Controlled Outputs
        [uct(1:2),~,dxdt_c] = LongControlOut(t,x,y_way(1:4),...
            addt,bounds,AC,y(1:4));                         % Longitudinal Control
        [uct(3),~] = LatDirControlOut(t,x,y,y_way,AC,GEO);  % Latero-Directional Control 
        % Ground Velocity components in inertial frame
        V = nan(2,1);
        V(1) = y(8)*cos(y(9)); V(2) = y(8)*sin(y(9));       % Vxi,Vyi
        dxdt = DynNoLinComp(t,x,uct,AC,GEO,V);              % [Va,ga,h,m,-,-,-,psi,phi,p]
        % Derivatives of Control Variables
        % [dICL/dt dIT/dt,dVd/dt]
        dxdt(5:7) = dxdt_c;
    end

    % event function: stops the integration when a waypoint is reached
    function [position,isterminal,direction] = wayReachedEvent(t,x,y_way)
    % WAYREACHEDEVENT Defines the integration termination event
    %   The event is triggered when the aircraft state is within an
    %   elliptical 'capture halo' around the desired waypoint state
    %   (y_way). The integration stops when POSITION equals zero.
    % INPUT
    %  - t: Current time (scalar).
    %  - x: Current state vector.
    %  - y_way: Desired output vector at the target waypoint.
    % OUTPUT
    %  - position: Value of the function that defines the event (0 when event is triggered).
    %  - isterminal: Flag (1=terminate, 0=continue).
    %  - direction: Flag (-1=event is triggered on a decreasing function).
    % -------------------- %
        dst = CaptureHalo(x,y_way,bounds,windpx,windpy,GEO);
        position = dst;         % When stop = 0 the integration stops
        isterminal = 1;         % Halt integration
        direction = -1;         % The fnction must be decreasing
    end
    
    % output function: it is called by the ode solver only after a
    % successful time step integration
    
    function status = myoutcomp(t, x, flag)
    % MYOUTCOMP Output function for the ODE solver (e.g., ode113)
    %   Used to store results, update persistent control
    %   parameters (via x_add), and manage data passing between successful steps.
    % INPUT
    %  - t: Vector of time points for the current successful step.
    %  - x: State matrix corresponding to t.
    %  - flag: String indicating the solver's current phase ("init", "done", or main step).
    %
    % OUTPUT
    %  - status: Status flag (0 = continue integration).
    % https://stackoverflow.com/questions/38120741/using-persistent-variable-to-pass-out-extra-parameters-using-ode45-from-the-matl
    % -------------------- %
        % Don't ever halt the integration
        status = 0;
        switch flag
            case "init"
            % Initialization (apparently useless).
                param = -2;
                % Initialize the addt variables
                if iway == 1
                    % Executes this code only if it is the very first waypoint,
                    % otherwise it takes the values from previous
                    % integration
                    % Retrives the storage vector from prevous integration
                    % cycle
                    % x_add = [ID,VIAS,dVd/dt,x(7),hdot]
                    addt = x_add();        
                    [~,uout,dxdt] = LongControlOut(t(1),x,y_way(1:4,iway+1),addt,bounds,AC);
                    addt(1) = uout(1); % Control ID
                    addt(2) = uout(2); % Commanded IAS (VIAS)
                    addt(3) = dxdt(3); % dVd/dt
                    addt(4) = x(7);    % Integral Vd
                    addt(5) = uout(4); % Kh
                    % Update storage
                    x_add =  storefun(addt); 
                end
            case "done"
                % Nothing to do.
            otherwise
            % Main code to update param
                nt = length(t);
                % Initialize temporary storage for this block of steps
                % store = [ID,VIAS,hdotc,Kc,Vd,CL,T,Roll,PhiD,PsiD,dR]
                store_temp = nan(nt,ns); 
                store = out_store(); 
                % x_add = [ID,VIAS,dVd/dt,x(7),Kh]
                addt = x_add();  
                % This code is useful only for storage and post-processing
                for it = 1 :nt
                    % Output Vector
                    Vw = WindMap(windpx,windpy,x);
                    y = CompleteDynNoLin_Out(x,Vw);
                    % Longitudinal Output [ID,VIAS,hdotc,Kh,Vd,CL,T]
                    [uct,uout,dxdt] = LongControlOut(t,x,y_way(1:4,iway+1),...
                        addt,bounds,AC);
                    % Latero-Directional Output
                    [uld,uadd] = LatDirControlOut(t,x,y,y_way(:,iway+1),...
                        AC,GEO);
                    % Store Data
                    store_temp(it,:) = [uout(:)',uct(:)',uld(:)',uadd(:)'];
                    % [ID,VIAS,hdotc,Kc,Vd,CL,T,Roll,PhiD,PsiD]
                    
                end
                % This code is necessary during the integration as it
                % updates the starting value of Vd

                % Check if the control mode has changed OR if it was NOT zone 3 or 6 
                % (These zones use Vc, others use Vd from the waypoint)
                if uout(1) ~= addt(1) || ( uout(1) ~= 3 && uout(1) ~= 6 ) %/* \circsl \label{cod:complete_dnamics_setup_c2}*/
                % This means that at the previous time step we were in zone
                % 3 or 6 and now we are still in the same zone
                    addt(2) = uout(2); % Update with current IAS
                end
                if addt(3) ~= 0 && dxdt(3) == 0 %/* \starcirc \label{cod:complete_dnamics_setup_c1}*/
                    addt(4) = x(7);% Store the final value of Vc
                end
                addt(1) = uout(1);
                addt(3) = dxdt(3);
                addt(5) = uout(4);% Kh
                x_add = storefun(addt); 

                out_store = storefun([store;store_temp]); 


        end

    end
    %% Simulation
    CHS = 'complete';
    options = odeset('RelTol',1e-6,'AbsTol',1e-5);
    
    ns = 11;
    %  store = [ID,VIAS,hdotc,Kc,Vd,CL,T,Roll,PhiD,PsiD,dR]
    out_store = storefun(nan(1,ns));
    tstt = 5;
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
        case 2
            Vs = [120,140,120];
            gas = [0,0,0];
            hs = [4500,4500,4750];
            ms = ones(3,1)*15400;
            ways(1).ID = "Init"; ways(2).ID = "T2F"; ways(3).ID = "C2F";
            i = 1; ways(i).lat = 35.33; ways(i).lng = 17.34;
            i = 2; ways(i).lat = 35.82; ways(i).lng = 17.78;
            i = 3; ways(i).lat = 36.5; ways(i).lng = 17.2;
            ways(i).Az = -70;
        case 3
            Vs = [120,120,120];
            gas = [0,0,0];
            hs = [4500,4500,4500];
            ms = ones(3,1)*15400;
            ways(1).ID = "Init"; ways(2).ID = "C2F";
            ways(3).ID = 'R2F';
            i = 1; ways(i).lat = 36.2; ways(i).lng = 17.9;
            i = 2; ways(i).lat = 36.5; ways(i).lng = 17.2;
            ways(i).Az = -70;
            i = 3; ways(i).lat = 36.55; ways(i).lng = 17.22;
            ways(i).t = 1; ways(i).PsiE = 20;
        case 4
            % Longitudinal only
            Vs = [100,120,140,140];
            gas = [0,0,0,0];
            hs = [4500,4500,5500,4500];
            ms = ones(4,1)*15400;
            ways(1).ID = "Init"; ways(2).ID = "C2F";
            ways(3).ID = 'C2F'; ways(4).ID = 'C2F';
            i = 1; ways(i).lat = 36.2; ways(i).lng = 17.9;
            i = 2; ways(i).lat = 37.0; ways(i).lng = 17.9; ways(i).Az = 0;
            i = 3; ways(i).lat = 38.0; ways(i).lng = 17.9; ways(i).Az = 0;
            i = 4; ways(i).lat = 39; ways(i).lng = 17.9; ways(i).Az = 0;
        case 5
            % Lateral Only
            Vs = [120,120,120,120];
            gas = [0,0,0,0];
            hs = [4500,4500,4500,4500];
            ms = ones(4,1)*15400;
            ways(1).ID = "Init"; ways(2).ID = "T2F"; ways(3).ID = "C2F";
            % ways(4).ID = 'R2F';
            i = 1; ways(i).lat = 36.2; ways(i).lng = 17.9;
            i = 2; ways(i).lat = 37; ways(i).lng = 17.9;
            i = 3; ways(i).lat = 38; ways(i).lng = 17;
            ways(i).Az = 0;
            % i = 4; ways(i).lat = 38; ways(i).lng = 16.45;
            % ways(i).t = -1; ways(i).PsiE = 20;
    end
    Tfin = 2500;                                        % Final time
    te = 0;                                             % Starting time
    [x_way,y_way,ye,bounds,temp] = BuildWaypoint(GEO,AC,Vs,gas,hs,...
    ms,ways,te,true);                                   % Initial condition

    tres = []; xres = [];                               % Storage vectors
    options.OutputFcn = @myoutcomp;                     % Set output function 
    nway = length( y_way(1,:) )-1;                      % Number of waypoints ( y_way contains also initial condition)

    x_add = storefun( temp );                           %[te;te;0;-1;0;zeros(8,1)]);
    iway = 1;
    store_way(iway,1:3) = [te,bounds(:)'];
    while  ~(iway > nway) && te < Tfin
    
    if iway > 1
        % Assigns the initial value for thrust in It
        %AC.Thrust_Law(x(6),x(3))
        temp = out_store();                         % [ID,IASd,hdotc,Vd,CL,T,Roll,PhiD,PsiD,dR]
        ye(5) = temp(end,6);                        % CL last. The state variable contains only the integral parto of CL
        ye(6) = temp(end,7);                        % T last
        % y_way = LongDynNoLin_Out( x_way(:,iway) );            % Defining the y desired output. The bounds are in kts and ft
        bounds = UpdateBounds( x_way(:,iway) );
    end
    % Define the event function for the current waypoint
    wayevent =@(t,x)wayReachedEvent(t,x,y_way(:,iway+1));   
    % Updates events function with new waypoint
    options.Events = wayevent;                                  
    % ODE integration: Solves the differential equations up to Tfin or the event
    [t,x,te,ye,ie] = ode113( @(t,x)CompDynNLCon( t,x,AC,...
        y_way(:,iway+1),bounds ),[te,Tfin],ye,options );
    
    tres = [tres;t(1:end-1,:)]; xres = [xres;x(1:end-1,:)];     % Saves the ith waypoint results
    if isempty(te)
        % If the event was not triggered (e.g., Tfin was reached)
        te = min(t(end),Tfin);
    end
    iway = iway+1;
    store_way(iway,1:3) = [te,bounds(:)'];      % After increasing iway because first row is initial cond.
    end
    tres = [tres;t(end,:)]; xres = [xres;x(end,:)];
    % x_aux = store = [ID,VIAS,hdotc,Kc,Vd,CL,T,Roll,PhiD,PsiD,dR]
    x_aux = out_store();

    % Plot the simulation results
    plot_results(AC,CHS,tres,xres,x_aux,y_way...
    ,store_way,windpx,windpy,GEO);
    end