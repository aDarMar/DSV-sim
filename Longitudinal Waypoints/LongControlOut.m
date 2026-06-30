function [u,u_out,dxdt] = LongControlOut(t,x,y_way,x_add,bounds,AC,y)
%LONGCONTROLOUT: This function calculates teh current zone to select the
%   appropriate control strategy (CL, Thrust) based on the altitude and
%   speed errors relative to the waypoint (y_way) and predefined bounds.
%   INPUT
%   - x: longitudinal state vector
%   - y:way: longitudinal waypoint
%   - x_add: [ID,VIAS,dVd/dt,x(7),Kh] evaluated at previous successful iteration
%   - bounds: [vbound,hbound,V/g] in SI units
%   - y: state output (optional)
%   OUTPUT
%   - u_out: [ID,VIAS,hdotc,Kh,Vc]
% ----------------------------------------------------------------------- %
    KHS = 1; FPMc = 500; ER1 = 2.3;     % Khdot 1 [1/s] RoD custom 500 fpm ER1 Energy ratio in Reg. 1
    dTidl = 0.6;                        % Idle Thrust Settings
    if nargin < 7                       % Evaluate state output only if it is not given as input
        y = LongDynNoLin_Out(x(:));     % State output
    end
    dxdt = zeros(3,1);                  % dxdt for CL_i,T_i,Vd
    err = y_way(:) - y(:);              % Error definition
    u_out = nan(1,5);                   % Initialize output vector
    u_out(5) = nan;                     % Vc/Vd
    u_out(1) = ZoneIdf(err,bounds);     % Identifies the zone in which the aircraft is                 Flag vector [ slow,fast,Low,High,lowenergy]
    u = zeros(2,1);                     % Force Vector [CL,T]
    switch u_out(1)
        case 7
        % Steady Level Flight
        % In this region Vd = Vc but we save Vc as VIAS because it is the
        % starting value for Vd in speed only controlled regions
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...
                CLcontrol('h',t,x,y,err,x_add,AC,3,nan);                % CL control of hdot
            [u(2),dxdt(2),u_out(2),dxdt(3) ] = ...
                Tcontrol(AC,3,x,y,y_way,'lev',x_add);                   % Thrust control of IAS
            u_out(2) = y(1);
            u_out(5) = y_way(1);                                % Vc
        case 2
            % Accelerating Flight
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...             % CL control of hdot
                CLcontrol('h',t,x,y,err,x_add,AC,1,nan);
            dxdt(2) = 0;                                        % There is no integral control of T
            u(2) = AC.Thrust_Model( x(3),x(1),1,'complx' );     % Throttle set to full
            u_out(2) = y(1);                                    % Vd is not defined and it is equal to V
        case 25
            % Descending & Accelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...             % CL control of hdot
                CLcontrol('h',t,x,y,err,x_add,AC,1,KHS);        % Khdot is 1 1/min
            dxdt(2) = 0;                                        % There is no integral control of T
            u(2) = AC.Thrust_Model( x(3),x(1),dTidl,'complx' ); % Throttle set to idle
            u_out(2) = y(1);                                    % Vd is not defined and it is equal to V
        case 5
        % Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...             % CL control of hdot
                CLcontrol('h',t,x,y,err,x_add,AC,1,nan);
            dxdt(2) = 0;                                        % There is no integral control of T
            u(2) = AC.Thrust_Model( x(3),x(1),dTidl,'complx' ); % Throttle set to full 
            u_out(2) = y(1);                                    % Vd is not defined and it is equal to V
        case 55
        % Climbing & Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...             % CL control of hdot
                CLcontrol('hc',t,x,y,err,x_add,AC,1,KHS,FPMc);  % Khdot is 1 1/min 
            dxdt(2) = 0;
            u(2) = AC.Thrust_Model( x(3),x(1),1,'complx' );     % Throttle set to full 
            u_out(2) = y(1);                                    % Vd is not defined and it is equal to V
        case 3
        % Descending
        % In this region Vd (speed of the waypoint) is different from Vc
        % (speed used by the controller)
            [u(1),dxdt(1),u_out(4),temp ] = ...                 % CL control of V Vd is taken from the Vc function
                CLcontrol('V',t,x,y,err,x_add,AC,2,[],[],y_way);
            dxdt(2) = 0;        % No integral Control of Thrust
            % temp=[dVd/dt, Vc]
            dxdt(3) = temp(1);  % dVc/dt
            u_out(5) = temp(2); % Vc
            u(2) = AC.Thrust_Model( x(3),x(1),dTidl,'complx' );% Throttle set to idle     
            u_out(2) = x_add(2); % Stores the VIAS that has received as input
             % In this case we are storing the VIAS at the beginning of the region
        case 6
        % Climbing
        % In this region Vc (speed of the waypoint) is different from Vd
        % (speed used by the controller)
            [u(1),dxdt(1),u_out(4),temp ] = ...                 % CL control of V Vd is taken from the Vc function
                CLcontrol('V',t,x,y,err,x_add,AC,2,[],[],y_way);
            dxdt(2) = 0;        % No integral Control of Thrust
            % temp=[dVd/dt, Vc]
            dxdt(3) = temp(1);  % dV/dt
            u_out(5) = temp(2); % Vc
            u(2) = AC.Thrust_Model( x(3),x(1),1,'complx' );
            u_out(2) = x_add(2); % Stores the VIAS that has received as input
            % In this case we are storing the VIAS at the beginning of the region
        case 1
        % Climbing & Accelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
                CLcontrol('hER',t,x,y,err,x_add,AC,1,1,ER1,y_way); 
            dxdt(2) = 0;
            u(2) = AC.Thrust_Model( x(3),x(1),1,'complx' );%Thrust_Law(1,x(3),'ipt');
            u_out(2) = y(1);                                            % Vd is not defined and it is equal to V

        case 4
        % Descending & Decelerating
            [u(1),dxdt(1),u_out(4),u_out(3) ] = ...                 % CL control of hdot
                CLcontrol('hER',t,x,y,err,x_add,AC,1,4,ER1,y_way); 
            dxdt(2) = 0;
            u(2) = AC.Thrust_Model( x(3),x(1),dTidl,'complx' );%Thrust_Law(1,x(3),'idl');
            u_out(2) = y(1);                                            % Vd is not defined and it is equal to V
    end
end

function ID = ZoneIdf(err,bounds)
%ZONEIDF: Identifies the current flight regime (Zone ID) based on speed and 
% altitude error.
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
%CLcontrol Implements Lift Coefficient (CL) control laws.
%   Controls either the altitude rate (hdot) or the commanded speed (Vd)
%   by setting the Lift Coefficient (CL) command.
% INPUT:
%   - flg: Control mode selector ('h', 'hc', 'hER', 'V')
%   - k: Index specifying the gains to use (AC.Kp(1,:,k), etc.)
%   ... (other inputs as in LongControlOut)
% OUTPUT:
%   - uCL: Commanded Lift Coefficient (CL)
%   - dxdt: Derivative of the CL integral state (d(CL_i)/dt)
%   - Kh: Khdot value used in the control law (or -1 if N/A)
%   - comm: Commanded value (hdot_c or [dVd/dt; Vd])
% ----------------------------------------------------------------------- %    
    switch flg
        case 'h'
            [comm,Kh] = hdot_des(y,err,x_add,Kh); % comm = hdot_c
            % Integral control term:
            dxdt = AC.Ki(1,:,k)*[0;0;0;comm-y(4)];
            % CL command
            uCL = AC.Kp(1,:,k)*( [0;0;0;comm-y(4)] ) - AC.Kb(1,:,k)*[0;0;0;y(4)] + x(5);
        case 'hc' 
        % hdot custom: hdot is the maximum between the value in Kh and the one calculated
            [comm,Kh] = hdot_des(y,err,x_add,Kh);
            % Limit commanded hdot to be at least |hdotcust|
            comm = max( comm,abs(hdotcust) ); % hdotc is the maximum between the one calculated and the one given CONTROLLA IL SEGNO FORSE ROD (<0)
            dxdt = AC.Ki(1,:,k)*[0;0;0;comm-y(4)];
            uCL = AC.Kp(1,:,k)*( [0;0;0;comm-y(4)] ) - AC.Kb(1,:,k)*[0;0;0;y(4)] + x(5);
        case 'hER' 
        % hdot is calculated using entry rate (it is given as hdotcust)
            switch Kh
                case 1
                    T = AC.Thrust_Model( x(3),x(1),0.6,'complx' );
                case 4
                    T = AC.Thrust_Model( x(3),x(1),0.6,'complx' );
                otherwise
                    error('Controller used in wrong region')
            end
            Kh = -1; % Reset Kh to signal non-Khdot based control
            [a,~,~,rho] = atmosisa(y_way(3)*0.305);
            % Use current state to find Drag
            V = IAS2TAS(y_way(1),y_way(3),'ktsh'); q = 0.5*rho*V^2;               % CL is TAS @ des point
            M = V/a; Re = AC.ReCalc(x(3),M);
            D = AC.polar(M,Re,x(4)*9.81/(q*AC.Sw)); 
            D = D*q*AC.Sw; 
            % Calculate hdot_c in [fpm]
            comm = (T - D)*x(1)/( x(4)*9.81*(1+hdotcust) )*60/0.305;

            dxdt = AC.Ki(1,:,k)*[0;0;0;comm-y(4)];
            uCL = AC.Kp(1,:,k)*( [0;0;0;comm-y(4)] ) - AC.Kb(1,:,k)*[0;0;0;y(4)] + x(5); % VEDERE SEGBOOOO
        case 'V'
            % Check if the speed error is small enough to set Vd = Vc
            if abs( err(1) )< 5
               Vd = y_way(1);           % Saves IAS as Vd but it is not used for actual control
               comm = 0;
            else
                comm = V_des(err(1));               % comm is now dVd/dt 
                % x_add(4) is the value the variable x(7) has when it
                % enters the region where the desired speed is given by Vd
                % and updates (in the ode output function) only when it
                % recognises that Vd is no longer linearly varying.
                % Vd = IAS + integral of dVd/dt - Vd(t0)
                Vd = x(7) + x_add(2) - x_add(4);  
            end
            Kh = -1; 
            dxdt = AC.Ki(1,:,k)*[Vd-y(1);0;0;0]; comm = [comm;Vd];
            uCL = AC.Kp(1,:,k)*( [Vd-y(1);0;0;0] ) - AC.Kb(1,:,k)*[y(1);0;0;y(4)] + x(5); 
    end
    % Check if the commanded CL is achievable
    uCL = AC.Aerodynamic_Mod('t',x(3),y(2),uCL);
end

function [uT,dxdt,Vc,comm] = Tcontrol(AC,k,x,y,y_way,flg,x_add)
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
% ----------------------------------------------------------------------- %
if y_way(1)-y(1) > 50 % NOT USED
% This was a test to reduce the commanded thrust but it causes a sharp drop
% of thrust when entering this region from a region where thrust is set to
% max and causes the state to jump continously from the previous state to this
        comm = V_des(y_way(1)-y(1));
        Vc = x(7) + x_add(2) - x_add(4);
else
        Vc = y_way(1);
        comm = 0;
end
    % Calculates the required thrust
    uT = AC.Kp(2,:,k)*[Vc-y(1);0;0;0] - AC.Kb(2,:,k)*[y(1);0;0;0] + x(6); 
    % Integral Thrust derivative
    dxdt = AC.Ki(2,:,k)*[Vc-y(1);0;0;0];        
    % Check if the engine can develop the required thrust
    uT = AC.Thrust_Model( x(3),x(1),1,'complx',uT );                                                   
end

function [hdotc,Kh] = hdot_des(y,err,x_add,Kh)
% HDOT_DES: Function that calculates the desired vertical speed, in order
% to reduce the altitude error.
% INPUT:
%   - y: Output vector [VIAS, M, h, hdot] in [kts, -, ft, fpm].
%   - err: Error vector [VIAS_err, M_err, h_err] (where $h_{err} = h_{way} - h$).
%   - x_add: Additional state vector from the previous time step 
%            [ID_prev, VIAS_prev, dVd/dt_prev, x(7)_prev, Kh_prev] 
%            Note: Kh_prev is stored in x_add(5).
%   - Kh: Khdot value passed from the main controller (can be NaN initially).
%
% OUTPUT:
%   - hdotc: Commanded Rate of Climb/Descent [fpm].
%   - Kh: Proportional gain for altitude rate control, $K_h$ [1/min or $1/\text{ft}$].
% ----------------------------------------------------------------------- %
        if isnan(Kh)                            % Kh is not given as input so it must be evaluated
            if x_add(5) == -1                   % Kh at previous tiem step is -1
                % It is entering from a uncontrolled RoC
                if err(3) == 0
                    % This check avoids to have a division by zero
                    % introducing a 1ft error (30 cm)
                    err(3) = 1;
                end
                Kh = y(4)/err(3);               % Temporary Khdot (*)
            elseif abs(err(3)) < 70             % 70 ft tolerance
                Kh = 7;                         % [1/min]
            else
            % In this case Kh is the Kh calculated at the beginning of the transition region
            % as the temporary Khdot (*)
                Kh = x_add(5);                 
            end
        end
    hdotc = Kh*err(3); % hdot = Kdot (hc-h)
end

function  dVddt = V_des(err)
%V_DES: Function that returns the desired Vd in climb and descend, where
%the normal control law would produce responses too great.
% it commands a constant 
%   acceleration or deceleration until the speed error is reduced.
%   INPUT
%   - err: speed error
% OUTPUT:
%   - dVddt: Commanded rate of change of the commanded speed V_d kts/s.
% ----------------------------------------------------------------------- %
    dVddt = ( err(1) )/abs( err(1) ); 
end
