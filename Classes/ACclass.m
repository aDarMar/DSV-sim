classdef ACclass
    %ACCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Name
        
        MTOM                                                            % Maximum TakeOff Mass [kg]
        MTO                                                             % Max. TO Weight
        MLND                                                            % Max LND Weight
        MZF                                                             % Max Zero Fuel [Kg]
        
        Sw
        bw
        cref
        ARw
        
        
        CD0
        K
        CLcdm
        CLmax
        polar
    end
    
    methods
        function obj = ACclass(file_name)
            %ACCLASS Construct an instance of this class
            %   Detailed explanation goes here
            %   INPUT
            %   - data_name: name of .mat file containing the aerodynamic
            %               database
            %   - MSs: vector containing all the required masses
            %               [MTOM,MZF]
            %   - Wng: vector containing [Sw,bw] for the wing
            %obj.Property1 = inputArg1 + inputArg2;
            
            % Masses
            [obj,data_name] = obj.read_input(file_name);
            obj.ARw = obj.bw^2/obj.Sw;
            % Definition of Interpolating Functions
            load(data_name,'aero_synt')           % TODO: includere anche condizioni T/O e Land
            obj.CD0 = scatteredInterpolant(aero_synt(:,1),...           % CD0 interpolation
                aero_synt(:,2),aero_synt(:,3) );
            obj.K = scatteredInterpolant(aero_synt(:,1),...             % CL@CDmin interpolation
                aero_synt(:,2),aero_synt(:,4) );
            obj.CLcdm = scatteredInterpolant(aero_synt(:,1),...         % CL@CDmin interpolation
                aero_synt(:,2),aero_synt(:,5) );
            obj.CLmax = scatteredInterpolant(aero_synt(:,1),...         % CLmax interpolation
                        aero_synt(:,2),aero_synt(:,8) );  
                    
            obj.polar =@(M,Re,CL) obj.CD0(M,Re) + obj.K(M,Re)*( CL(:) - obj.CLcdm(M,Re) ).^2;
        end
        
        function [obj,data_name] = read_input(obj,dataFileName)
            %READ_INPUT: Reads aircraft data from the txt file
            %   Detailed explanation goes here
            function test_block(f_id,test)
                temp = fgetl(f_id);
                if ~strcmp(temp,test)
                    error( strcat('Expected',test,'fields') );
                end
            end
            
            [f_id,err_txt] = fopen(dataFileName,'r');                             % Read Input File
            if f_id == -1
                disp( strcat('Error Opening ',dataFileName,'. ',err_txt) );
            else
                try
                disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
                lin = fgetl(f_id); strtok(lin,{'%'}) % Skips firt line
                lin = fgetl(f_id); lin = strtok(lin,sprintf(' \t%')); obj.Name = lin; % Name
                lin = fgetl(f_id); lin = strtok(lin,sprintf(' \t%')); data_name  = lin;
                % Masses
                test_block(f_id,'MASSES');
                lin = fscanf(f_id,'%f '); obj.MTOM = lin; fgetl(f_id);
                lin = fscanf(f_id,'%f '); obj.MTO = lin; fgetl(f_id);
                lin = fscanf(f_id,'%f '); obj.MLND = lin; fgetl(f_id);
                lin = fscanf(f_id,'%f '); obj.MZF = lin; fgetl(f_id);
                % Wing
                test_block(f_id,'GEOMETRY');

                test_block(f_id,'WING');
                lin = fscanf(f_id,'%f '); obj.Sw = lin; fgetl(f_id);
                lin = fscanf(f_id,'%f '); obj.bw = lin; fgetl(f_id);
                lin = fscanf(f_id,'%f '); obj.cref = lin; fgetl(f_id);
                
                fclose(f_id);
                catch ERR
                    warning(['Failed Reading File Fields',ERR.message])
                   fclose(f_id);
                end
            end
        end

        function T = Thrust_Law(obj,Treq,h,flg)
        %THRUST_LAW Function that models the thrust produced by the engine
        %   T = Thrust_Law(Treq,h): checks if the required thrust passed with Treq
        %       is actually available at the given altitude h. If that is the case,
        %       the function returns T, otherwise it returns the maximum available
        %       thrust at the given altitude
        %   T = Thrust_Law(Treq,h,flag): if flag is passed, Treq is the admission
        %       rate of the engine at the given altitude and teh funcion returns the
        %       thrust developed by the engine(s) for the given combination of
        %       admisison rate and altitude. 
        %           (i) ipt: the admission rate is taken from the one in input
        %          (ii) idl: engine is set to idle
        %         (iii) mxT: engine is set to max TO
        %          (iv) mxC: engine is set to max continous
        
        % TODO - FARE MODELLO MOTORE per ora i numeri sono temporanei
            Tmax = 10000; % N
            if nargin == 2
                T = Treq;
                if Treq > Tmax
                    T = Tmax;
                end
            else
                if Treq > 1 || Treq < 1
                    error('Admission rate must be between [0,1]')
                end
                switch flg
                    case 'ipt' % Admission rate is given as input
                        T = Tmax*Treq;
                    case 'idl'
                        T = 0.5*Tmax; % IDLE FINIREEEE
                    case 'mxT'
                        % VEDERE in che modo implementare il motore, potranno essere
                        % anche sostituiti da Tmax(h)
                    case 'mxC'
            
                end
            end
        end

        function [CL,CD] = Aerodynamic_Mod(obj,flag,h,M,inp)
        %AERODYNAMIC_MOD Function that models the aerodynamic characteristics of
        %the aircraft
        %   L = Aerodynamic_Mod("t",h,inp): checks if the required CL 
        %       passed with inp is actually available at the given altitude h.
        %       If that is the case, the function returns T, otherwise it returns 
        %       the maximum available lift at the given altitude
        %       
        %       
        %   [L,D] = Aerodynamic_Mod("alpha",h,inp): calculates all teh aerodynamic
        %           data for a given combination of alpha and altitude
        
        % TODO - FARE MODELLO AERODINAMICO per ora i numeri sono temporanei
        
            Re = obj.ReCalc(h,M);       % Flight Reynolds Number
            
            %CLmax = 1.8; % TEMPORANEOOO
            % [M,Re,CD0,K,CL@min(Cd),CLa,CL0,CLmax,AoA@CLmax]
            switch flag
                case "t"
                    CLm = obj.CLmax(M,Re);                                      
                    CL = inp(:);
                    if any( inp > CLm )
                        CL(inp > CLm) = CLm;
                    end
                    obj.polarcalc(CL)
                    CD = obj.polar(M,Re,CL);
            end
        end
        
        function RE = ReCalc(obj,h,M)
            %RECALC: calculates Reynolds Numer at a given Mach and altitude.
            [T, a, ~, rho] = atmosisa(h);
            mu = 1.458e-6 * T^(3/2)/(T+110.4); % Sutherland Law used for retrocompatibility with older matlab versions
            V = M*a; RE = V*obj.cref*rho/mu;
        end
        
        function [A,B,C,u0] = LongLinSys(obj,x0,m)
            %LONGLINSYS: Builds the linearized dynamic model around x0
            %   INPUT
            %   x0 - Vettore delle varibili di stato longitudinali nella cond. di rif.
            %       [Va,ga,h]
            %   m - Aircraft mass around condition
            %   OUTPUT
            %   A
            %   C - Output matrix, returns [VIAS;M;h;hdot] in imperial
            %       units: VIAS in kts, h in ft, hdot fpm
            
            g = 9.81; gm = 1.4;
            [T, a, p, rho] = atmosisa([0,x0(3)]);
            
            M = x0(1)/a(2); Re = obj.ReCalc(x0(3),M);
            CL = m*g*2/(rho(2)*obj.Sw*x0(1)^2); % Trim CL
            
            u0 = zeros(2,1); u0(1) = CL; 
            u0(2) = rho(2)*0.5*obj.Sw*...
                x0(1)^2*obj.polar(M,Re,CL);
            %u0 = u0*rho(2)*0.5*AC.Sw*x0(1)^2;
            
            A = zeros(3);
            A(1,1) = -rho(2)*x0(1)*obj.Sw*( obj.polar(M,Re,CL) )/m; % dfVa/dVa
            A(1,2) = -g*cos(x0(2)); % dfVa/dga
            A(2,1) = rho(2)*obj.Sw*CL/(2*m) + g/(x0(1)^2)*cos(x0(2)); % dfga/dVa
            A(2,2) = g/x0(1) * sin(x0(2)); % dfga/dga
            A(3,1) = sin(x0(2));% dfh/dVa
            A(3,2) = x0(1)*cos(x0(2));% dfh/dga
            
            B = zeros(3,2);
            B(1,1) = -rho(2)*x0(1)^2*obj.Sw*obj.K(M,Re)*(CL - obj.CLcdm(M,Re))/m; % dfVa/dCL
            B(1,2) = 1/m; % dfVa/dT
            B(2,1) = rho(2)*x0(1)*obj.Sw/(2*m); % dfGa/dCL
            
            C = zeros(4,3);
            M = x0(1)/a(2); MMS = (1+0.5*(gm-1)*M^2)^(1/(gm-1));
            C(1,1) = a(1)/a(2) * p(2)/p(1) * M*MMS*( p(2)/p(1)*( MMS^gm - 1 ) + 1 )^(-1/gm);
            C(1,1) = C(1,1)/ sqrt( 2/(gm-1)*( ( p(2)/p(1)*( MMS^gm - 1 ) + 1 )^(1-1/gm) -1 ) ); % dgVIAS/dVa
            C(1,1) = C(1,1)/0.51444; % VIAS in kts
            C(2,1) = 1/a(2); % dgM/dVa
            C(3,3) = 1/0.305;  % dgh/dh in feet
            C(4,2) = A(3,2)*60/0.305; % dghdot/dga fpm
            
        end

    end
end

