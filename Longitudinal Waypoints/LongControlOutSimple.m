function [u,u_out,dxdt] = LongControlOutSimple(t,x,y_way,x_add,AC)
%LONGCONTROLOUTSIMPLE Summary of this function goes here
%   INPUT
%   - x : [V,ga,h,m]
%   - bounds: [vbound,hbound,V/g] in SI units
    u_out = nan(1,4);                   % Output vector [ID,Vc,hdotc,Kh]
    y = LongDynNoLin_Out(x(:));       % State output
    err = y_way(:) - y(:);              % Error definition
    % u_out(1) = ZoneIdf(err,bounds);     % Identifies the zone in which the aircraft is                 Flag vector [ slow,fast,Low,High,lowenergy]
    u = zeros(2,1);                     % Force Vector [CL,T]
    % DEBUG 
    TEST = 2;
    switch TEST
        case 7
        % Steady Level Flight
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...
               CLcontrol('h',t,x,y,y_way,err,x_add,AC,nan);     % CL control of hdot
            [u(2),dxdt(2),u_out(2) ] = Tcontrol(x,y,y_way); % Thrust control of IAS  
        case 2
        % Accelerating Flight
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...         % CL control of hdot
                CLcontrol('h',t,x,y,y_way,err,x_add,AC,nan);         
            dxdt(2) = 0;                                    % There is no integral control of T
            u(2) = AC.Thrust_Law(1,x(3),'ipt');                % Throttle set to full VEDER SE SOSTITUIRE CON mxC
        case 25
        % Descending & Accelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...         % CL control of hdot
                CLcontrol('h',t,x,y,y_way,err,x_add,AC,KHS);  % Khdot is 1 1/min 
            dxdt(2) = 0;                                    % There is no integral control of T
            u(2) = AC.Thrust_Law(1,x(3),'idl');                % Throttle set to idle
        case 5
        % Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...         % CL control of hdot
                CLcontrol('h',t,x,y,y_way,err,x_add,AC,nan);
            dxdt(2) = 0;                                    % There is no integral control of T
            u(2) = AC.Thrust_Law(1,x(3),'idl');                % Throttle set to full VEDER SE SOSTITUIRE CON mxC
        case 55
        % Climbing & Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...         % CL control of hdot
                CLcontrol('hc',t,x,y,y_way,err,x_add,AC,KHS,FPMc);      % Khdot is 1 1/min 
            dxdt(2) = 0;
            u(2) = AC.Thrust_Law(1,x(3),'ipt');                % Throttle set to full VEDER SE SOSTITUIRE CON mxC
        case 3
        % Descending
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...         % CL control of V
                CLcontrol('V',t,x,y,y_way,err,x_add,AC);
            dxdt(2) = 0;
            u(2) = AC.Thrust_Law(1,x(3),'idl');        
        case 6
        % Climbing
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...         % CL control of V
                CLcontrol('V',t,x,y,y_way,err,x_add,AC);
            u(2) = AC.Thrust_Law(1,x(3),'ipt');                % SOLITA COSA DI TMAX
        case 1
        % Climbing & Accelerating
            
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...         % CL control of hdot
                CLcontrol('hc',t,x,y,y_way,err,x_add,AC,nan,ER1); 
            dxdt(2) = 0;
            u(2) = Thrust_Law(1,x(3),'idl');
        case 4
        % Descending & Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...         % CL control of hdot
                CLcontrol('hc',t,x,y,y_way,err,x_add,AC,nan,ER1); 
            dxdt(2) = 0;
            u(2) = Thrust_Law(1,x(3),'idl');
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
    (flg,t,x,y,y_way,err,x_add,AC,Kh,hdotcust)
% CL control of altitude or speed
% UNTESTED SPECIFICARE CONALT e capire come gestire conalt tra regione 7 e
% 25

    switch flg
        case 'h'
            %[comm,Kh] = hdot_des(y,y_way,err,x_add,Kh); % ADD conalt. Returns the state vector with ONLY x(8) updated (Khdot)
            comm = x_add(3); Kh = -1;
            dxdt = AC.Ki(1,:,1)*[0;0;0;comm-y(4)];
            uCL = AC.Kp(1,:,1)*( [0;0;0;comm-y(4)] ) - AC.Kb(1,:,1)*[0;0;0;y(4)] + x(5); % MODIFICA OVUNQUEEEE x(4) -> x(5)
        
        
        
        
        
        
        
        
        case 'hc' % hdot custom: hdot is the maximum between the value in Kh and the one calculated
            [comm,Kh] = hdot_des(y,y_way,err,x_add,Kh);
            comm = max( comm,abs(hdotcust) ); % hdotc is the maximum between the one calculated and the one given CONTROLLA IL SEGNO FORSE ROD (<0)
            dxdt = Ki(1,:)*[0;0;0;comm-y(4)];
            uCL = Kp(1,:)*( [0;0;0;comm-y(4)] ) - Kb(1,:)*[0;0;0;y(4)] + x(4);
        case 'hER' % hdot is calculated using entry rate (it is given as hdotcust)
            if ~isnan(Kh)
                error('To use CL control of hdot with entry rate Kh must be passed as nan')
            end
            % EXP: usiamo i valori desiderati
            [~,~,~,rho,~,~] = atmosisa(y_way(3));
            V = IAS2TAS(y_way(1),y_way(3)); q = 0.5*rho*V^2;               % CL is TAS @ des point
            [~,D] = AC.Aerodynamic_Mod(CL = AC.m*9.81/(q*AC.Sw)); 
            D = D*q*AC.Sw; T = AC.Thrust_Law(1,y_way(3),'ipt');
            comm = (T - D)*x(1)/( AC.m*9.81*(1+hdotcust) );

            dxdt = Ki(1,:)*[0;0;0;comm-y(4)];
            uCL = Kp(1,:)*( [0;0;0;comm-y(4)] ) - Kb(1,:)*[0;0;0;y(4)] + x(4);
        case 'V'
            comm = V_des(t,y,y_way);
            Kh = nan;
            dxdt = Ki(1,:)*[Vc-y(1);0;0;0];
            uCL = Kp(1,:)*( [comm-y(1);0;0;0] ) - Kb(1,:)*[y(1);0;0;0] + x(4);
    end
    uCL = AC.Aerodynamic_Mod('t',x(3),y(2),uCL); % Checks if the aircraft can attain the required CL
end

function [uT,dxdt,Vc] = Tcontrol(x,y,y_way)
% T control of IAS
% UNTESTED
    [Kp,Ki,Kb] = Control_Gains(x);                                  % Controller Gains for specific flight condition
    Vc = V_des(t,y,y_way);
    uT = Kp(2,:)*[Vc-y(1);0;0;0] - Kb(2,:)*[y(1);0;0;0] + x(5);     % Calculates the required thrust to attain the Vc
    dxdt = Ki(2,:)*[Vc-y(1);0;0;0];
    uT = Thrust_Law(uT,x(3));                                       % Checks if teh engien can provide the required thrust
end

function [hdotc,Kh] = hdot_des(y,y_way,err,x_add,Kh)
% Function that calculates the desired rate of climb/descend
% UNTESTED
    if isnan(Kh) % Khdot is NOT given as input so it must be evaluated
        %if conalt % NON HA SENSO FORSE; COMUNQUE PREVEDERE CHE Khdot diventi -1
        % Is in altitude control region
        if x_add(3) == -1
            % It is entering from a uncontrolled RoC
            Kh = y(4)/err(3); % Temporary Khdot
        elseif err(3) < 21.35 % 70 ft tolerance
            Kh = 0.1167; % 7/60 1/s
        end
    else
        % It is not in altitude control region
        Kh = -1;
    end
    %end
    hdotc = Kh*( y_way(3)-y(3) ); % hdot = Kdot (hc-h)
end

function  Vc = V_des(t,y,y_way,x_add)
% UNTESTED - PUO DARE PROBLEMI a causa delal dipendenza da dt di Vc
    Vc = 0.51444*( y_way(1)-y(1) )/abs( y_way(1)-y(1) ) * ( t-x_add(1) ); % PROBLEMA: ODE45 ha il passo adattivo, quindi t va avanti e indietro
end


