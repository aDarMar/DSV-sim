function [u,u_out,dxdt] = LongControlOut(t,x,y_way,x_add,bounds,AC)
%LONGCONTROLOUT Summary of this function goes here
%   INPUT
%   - x_add: [ID,VIAS,dVd/dt,x(7),Kh] evaluated at previous successful iteration
%   - bounds: [vbound,hbound,V/g] in SI units
%   OUTPUT
%   - u_out: [ID,VIAS,hdotc,Kh,Vd]
    KHS = 1; FPMc = 500; ER1 = 2.3;     % Khdot 1 [1/s] RoD custom 500 fpm ER1 Energy ratio in Reg. 1
    y = LongDynNoLin_Out(x(:));         % State output
    dxdt = zeros(3,1);                  % dxdt for CL_i,T_i,Vd
    err = y_way(:) - y(:);              % Error definition
    u_out = nan(1,5);                   % Initialize output vector
    u_out(5) = y_way(1);
    u_out(1) = ZoneIdf(err,bounds);     % Identifies the zone in which the aircraft is                 Flag vector [ slow,fast,Low,High,lowenergy]
    u = zeros(2,1);                     % Force Vector [CL,T]
    switch u_out(1)
        case 7
        % Steady Level Flight
        % In this region Vd = Vc but we save Vd as VIAS because it is the
        % starting value for Vd in speed only controlled regions
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...
                CLcontrol('h',t,x,y,err,x_add,AC,3,nan);             % CL control of hdot
            [u(2),dxdt(2),u_out(2) ] = Tcontrol(AC,3,x,y,y_way,'lev');    % Thrust control of IAS
            u_out(2) = y(1);                                        % Vd is not defined so it saved as Vc
            u_out(5) = y_way(1);
        case 2
            % Accelerating Flight
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
                CLcontrol('h',t,x,y,err,x_add,AC,1,nan);
            dxdt(2) = 0;                                            % There is no integral control of T
            u(2) = AC.Thrust_Law(1,x(3),'ipt');                     % Throttle set to full VEDER SE SOSTITUIRE CON mxC
            u_out(2) = y(1);                                        % Vd is not defined and it is equal to V
            %ContCLOnly('h',t,x,y,err,x_add,AC,1,nan,[],[])         % TODO:
            %FARE FUNZIONE CHE RIPETA TUTTI I CASI
        case 25
            % Descending & Accelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
                CLcontrol('h',t,x,y,err,x_add,AC,1,KHS);            % Khdot is 1 1/min
            dxdt(2) = 0;                                            % There is no integral control of T
            u(2) = AC.Thrust_Law(1,x(3),'idl');                     % Throttle set to idle
            u_out(2) = y(1);                                            % Vd is not defined and it is equal to V
            %ContCLOnly('h',t,x,y,err,x_add,AC,k,Kh,hdotcust,y_way)
        case 5
        % Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
                CLcontrol('h',t,x,y,err,x_add,AC,1,nan);
            dxdt(2) = 0;                                            % There is no integral control of T
            u(2) = AC.Thrust_Law(1,x(3),'idl');                     % Throttle set to full VEDER SE SOSTITUIRE CON mxC
            u_out(2) = y(1);                                            % Vd is not defined and it is equal to V

        case 55
        % Climbing & Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
                CLcontrol('hc',t,x,y,err,x_add,AC,1,KHS,FPMc);      % Khdot is 1 1/min 
            dxdt(2) = 0;
            u(2) = AC.Thrust_Law(1,x(3),'ipt');                     % Throttle set to full VEDER SE SOSTITUIRE CON mxC
            u_out(2) = y(1);                                            % Vd is not defined and it is equal to V

        case 3
        % Descending
        % In this region Vc (speed of the waypoint) is different from Vd
        % (speed used by the controller)
            [u(1),dxdt(1),u_out(4),temp ] = ...                 % CL control of V Vd is taken from the Vc function
                CLcontrol('V',t,x,y,err,x_add,AC,2,[],[],y_way);
            dxdt(2) = 0; dxdt(3) = temp(1); u_out(5) = temp(2);
            u(2) = AC.Thrust_Law(1,x(3),'idl');        
            u_out(2) = x_add(2); % CONTOLLS SE SERVE
        case 6
        % Climbing
        % In this region Vc (speed of the waypoint) is different from Vd
        % (speed used by the controller)
            [u(1),dxdt(1),u_out(4),temp ] = ...                 % CL control of V Vd is taken from the Vc function
                CLcontrol('V',t,x,y,err,x_add,AC,2,[],[],y_way);
            dxdt(2) = 0; dxdt(3) = temp(1); u_out(5) = temp(2);
            u(2) = AC.Thrust_Law(1,x(3),'ipt');                     % SOLITA COSA DI TMAX
            u_out(2) = x_add(2);                                    % In this case we are storing the VIAS at the beginning of the region
        case 1
        % Climbing & Accelerating
            
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
                CLcontrol('hER',t,x,y,err,x_add,AC,1,1,ER1,y_way); 
            dxdt(2) = 0;
            u(2) = AC.Thrust_Law(1,x(3),'idl');
            u_out(2) = y(1);                                            % Vd is not defined and it is equal to V

        case 4
        % Descending & Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
                CLcontrol('hER',t,x,y,err,x_add,AC,1,4,ER1,y_way); 
            dxdt(2) = 0;
            u(2) = AC.Thrust_Law(1,x(3),'idl');
            u_out(2) = y(1);                                            % Vd is not defined and it is equal to V
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


function [uCL,dxdt,Kh,comm] = CLcontrol...
    (flg,t,x,y,err,x_add,AC,k,Kh,hdotcust,y_way)
% CL control of altitude or speed
% UNTESTED SPECIFICARE CONALT e capire come gestire conalt tra regione 7 e
% 25
    
    switch flg
        case 'h'
            [comm,Kh] = hdot_des(y,err,x_add,Kh); % ADD conalt. Returns the state vector with ONLY x(8) updated (Khdot)
            dxdt = AC.Ki(1,:,k)*[0;0;0;comm-y(4)];
            uCL = AC.Kp(1,:,k)*( [0;0;0;comm-y(4)] ) - AC.Kb(1,:,k)*[0;0;0;y(4)] + x(5);
        case 'hc' % hdot custom: hdot is the maximum between the value in Kh and the one calculated
            [comm,Kh] = hdot_des(y,err,x_add,Kh);
            comm = max( comm,abs(hdotcust) ); % hdotc is the maximum between the one calculated and the one given CONTROLLA IL SEGNO FORSE ROD (<0)
            dxdt = AC.Ki(1,:,k)*[0;0;0;comm-y(4)];
            uCL = AC.Kp(1,:,k)*( [0;0;0;comm-y(4)] ) - AC.Kb(1,:,k)*[0;0;0;y(4)] + x(5);
        case 'hER' % hdot is calculated using entry rate (it is given as hdotcust)

            switch Kh
                case 1
                    T = AC.Thrust_Law(1,y_way(3),'ipt');
                case 4
                    T = AC.Thrust_Law(1,y_way(3),'idl');
                otherwise
                    error('Controlelr used in wrong region')
            end
            % EXP: usiamo i valori desiderati: nel documento dice che è la
            % derivata d?????
            Kh = -1; % TEMPORANEOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
            [a,~,~,rho] = atmosisa(y_way(3)*0.305);
            V = IAS2TAS(y_way(1),y_way(3),'ktsh'); q = 0.5*rho*V^2;               % CL is TAS @ des point
            M = V/a; Re = AC.ReCalc(x(3),M);
            D = AC.polar(M,Re,x(4)*9.81/(q*AC.Sw)); 
            D = D*q*AC.Sw; 
            comm = (T - D)*x(1)/( x(4)*9.81*(1+hdotcust) )*60/0.305;

            dxdt = AC.Ki(1,:,k)*[0;0;0;comm-y(4)];
            uCL = AC.Kp(1,:,k)*( [0;0;0;comm-y(4)] ) - AC.Kb(1,:,k)*[0;0;0;0] + x(5);
        case 'V'
            % if x_add(12) == 4 || x_add(12) == 5  % Checks if the ID @ previous time steo is in the V control zone
            %     x_add(6) = y(1);
            % end
            
            if abs( err(1) )< 5
               Vd = y_way(1);
               comm = 0;
            else
                comm = V_des(err(1)); % comm is now dVd/dt 
                Vd = x(7) + x_add(2) - x_add(4); 
            end
            Kh = -1; 
            dxdt = AC.Ki(1,:,k)*[Vd-y(1);0;0;0]; comm = [comm;Vd];
            uCL = AC.Kp(1,:,k)*( [Vd-y(1);0;0;0] ) - AC.Kb(1,:,k)*[y(1);0;0;y(4)] + x(5); % HO AGGIUNTO IL FEEDBACK DI HDOT
    end
    uCL = AC.Aerodynamic_Mod('t',x(3),y(2),uCL); % Checks if the aircraft can attain the required CL
end

function [uT,dxdt,Vc] = Tcontrol(AC,k,x,y,y_way,flg,x_add)
%TCONTROL: function that returns the thrust used to control speed (IAS)
%   INPUT
%   - AC: ACClass aircraft object
%   - k: index that specifies the gain of the controller in AC
%   - x: state vector [V,ga,h,m] in SI units
%   - y: output vector [IAS,M,h,hdot] in [kts,-,ft,fpm]
%   - y_way: output vector of reference state
%   - flg: specifies which Vc should be used
%   OUTPUT
%   - uT: controlled Thrust [N]
%   - dxdt: integral thrust controller
%   - Vc: commanded speed [IAS]
% UNTESTED
switch flg
    case 'climb' % TEMPORANEO per ora non viene mai chiamato 
        Vc = V_des(y_way(1)-y(1),x_add);
    otherwise
        Vc = y_way(1);
end
    uT = AC.Kp(2,:,k)*[Vc-y(1);0;0;0] - AC.Kb(2,:,k)*[y(1);0;0;0] + x(6);     % Calculates the required thrust to attain the Vc
    dxdt = AC.Ki(2,:,k)*[Vc-y(1);0;0;0];
    uT = AC.Thrust_Law(uT,x(3));                                       % Checks if teh engien can provide the required thrust
end

function [hdotc,Kh] = hdot_des(y,err,x_add,Kh)
% HDOT_DES: Function that calculates the desired vertical speed, in order
% to reduce the altitude error.
%   INPUT
%   - x_add: additional values from the previous time step [t(n-1),t@dt<0,dt,Kh@t(n-1),Vc@t(n-1)]
% hdot di transizione e' COSTANTE PER ORA 
% UNTESTED
    %if  % Khdot is NOT given as input 
        %if conalt % NON HA SENSO FORSE; COMUNQUE PREVEDERE CHE Khdot diventi -1
        % Is in altitude control region
        %if addt(3) > 0
        if isnan(Kh)                            % Kh is not given as input so it must be evaluated
            if x_add(5) == -1                   % Kh at previous tiem step is -1
                % It is entering from a uncontrolled RoC
                Kh = y(4)/err(3);               % Temporary Khdot
            elseif err(3) < 70                  % 70 ft tolerance
                Kh = 7;                         % [1/min]
            else
                Kh = x_add(5);                  % In this case Kh is the Kh calculated at the beginning of teh transition region
            end
        end
        %else
        % 
            %Kh = addt(4);   %If dt = 0 we take the Kh from the previous iteration
        %end
        %else
        % It is not in altitude control region
        %Kh = -1;
    %end
    %end
    hdotc = Kh*err(3); %y_way(3)-y(3) );               % hdot = Kdot (hc-h)
end

function  dVddt = V_des(err)
%V_DES: Function that returns the desired Vc in climb and descend, where
%the normal control law would produce responses too great.
%   INPUT
%   - err: speed error
%   - x:add: additional values from the previous time step [t(n-1),t@dt<0,dt,Kh@t(n-1),Vc@t(n-1)]
% UNTESTED - PUO DARE PROBLEMI a causa delal dipendenza da dt di Vc
% x_add(5) is Vd at the previous time step. The code is written in such a
% way that if we are not in the speed only controleld zones, Vd is equal to
% the actual VIAS, so that x_add is always equal to VIAS or Vc@t(n-1)
    dVddt = ( err(1) )/abs( err(1) ); % PROBLEMA: ODE45 ha il passo adattivo, quindi t va avanti e indietro
end

function [] = ContCLOnly(flg,t,x,y,err,x_add,AC,k,Kh,hdotcust,y_way)
    [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
        CLcontrol(flg,t,x,y,err,x_add,AC,k,Kh,hdotcust,y_way);
    dxdt(2) = 0;                                            % There is no integral control of T
    u(2) = AC.Thrust_Law(1,x(3),'ipt');                     % Throttle set to full VEDER SE SOSTITUIRE CON mxC
    u_out(2) = y(1);                                        % Vd is not defined and it is equal to V

end
 