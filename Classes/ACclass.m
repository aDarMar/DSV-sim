classdef ACclass
    %ACCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
        CD0
        K
    end
    
    methods
        function obj = ACclass(inputArg1,inputArg2)
            %ACCLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
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

        function [CL,CD] = Aerodynamic_Mod(obj,flag,h,inp)
        %AERODYNAMIC_MOD Function that models the aerodynamic characteristics of
        %the aircraft
        %   L = Aerodynamic_Mod("CL",h,inp): checks if the required CL 
        %       passed with Treq is actually available at the given altitude h.
        %       If that is the case, the function returns T, otherwise it returns 
        %       the maximum available lift at the given altitude
        %       
        %       
        %   [L,D] = Aerodynamic_Mod("alpha",h,inp): calculates all teh aerodynamic
        %           data for a given combination of alpha and altitude
        
        % TODO - FARE MODELLO AERODINAMICO per ora i numeri sono temporanei
            CLmax = 1.8; % TEMPORANEOOO
            switch flag
                case "CL"
                    CL = inp;
                    if inp > CLmax
                        CL = CLmax;
                    end
                    CD = obj.CD0 + obj.K*CL^2;
            
            end
        end


    end
end

