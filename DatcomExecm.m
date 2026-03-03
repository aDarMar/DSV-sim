close all; clearvars; clc;

% Execute DATCOM analysis for the Q100_clean input file
INP = DatcomExec('Q100_clean');

function INP = DatcomExec(iptfile)
% DATCOMEXEC Executes DATCOM analyses and processes results
% This function has two modes:
%   EXEC = 1: Runs DATCOM analyses for different Mach numbers and altitudes
%   EXEC = 2: Reads DATCOM output files and extracts aerodynamic derivatives
% 
% Note: The function assumes the .dcm file has specific formatting with
% blank spaces to insert flight condition parameters.
    
    % Define analysis parameters
    nM = 4;        % Number of Mach numbers to analyze
    nh = 9;        % Number of altitudes to analyze
    EXEC = 2;      % Execution mode: 1 = run analyses, 2 = read results
    EQL = false;   % Equilibrium polar flag
    
    % Create Mach number and altitude arrays
    M = linspace(0.1, 0.5, nM);   % Mach numbers from 0.1 to 0.5
    h = linspace(0, 8000, nh);    % Altitudes from 0 to 8000 meters
    
    % Define array sizes for data storage
    na = 20;   % Number of alpha points
    nde = 9;   % Number of delta elevator points
    
    switch EXEC
        case 1
            % =================================================================
            % EXECUTION MODE: Run longitudinal DATCOM analyses
            % =================================================================
            for i = 1:nM
                for j = 1:nh
                    % Create unique filename for each flight condition
                    optfile = strcat(iptfile, "_", num2str(round(M(i)*10, 0)), "_", num2str(h(j)));
                    
                    % Copy template DATCOM input file
                    copyfile(['Data\', iptfile, '.dcm'], strcat('Data\DATCOM\', optfile, '.dcm'));
                    
                    % Create DATCOM input file lines with flight conditions
                    nnline = ['         NALT=1.,ALT(1)=', num2str(round(h(j), 0)), '.,\n'];
                    nline = [' $FLTCON NMACH=1.0, MACH(1)=.', num2str(round(M(i)*10, 0)), ',\n'];
                    
                    % Open and modify the DATCOM input file
                    fid = fopen(strcat('Data\DATCOM\', optfile, '.dcm'), 'r+');
                    line = fgetl(fid);
                    
                    % Find the $FLTCON line in the input file
                    while ~contains(line, "$FLTCON")
                        line = fgetl(fid);
                    end
                    
                    % Write the new flight condition parameters
                    fprintf(fid, nline);
                    fprintf(fid, nnline);
                    fclose(fid);
                    fclose('all');  % Close all open files
                    
                    % Execute DATCOM from the command line
                    prjpath = pwd;
                    status = system(strcat('cd', {' '}, [prjpath, '\Data\DATCOM\'], ' &', strcat(optfile, '.dcm')));
                end
            end
            
        case 2
            % =================================================================
            % READING MODE: Extract aerodynamic derivatives from DATCOM outputs
            % =================================================================
            
            % Initialize data arrays
            % Longitudinal data: [alpha, M, h, CL, CD, CM, CLad, CMad, CLq, CMq]
            SLNG = nan(na*nM*nh, 10);
            
            % Lateral-directional data: [alpha, M, h, Cyb, Cnb, Clb, Clp, Cnp, Clr, Cnr, Cyp]
            SLAT = nan(na*nM*nh, 11);
            
            % Control derivatives: [deltaE, M, h, dCLde, dCMde]
            CTR = nan(na*nM*nh, 5);
            
            % Initialize counters
            ne = 0;   % Counter for longitudinal data
            nec = 0;  % Counter for control derivatives
            
            % Process each DATCOM output file
            for i = 1:nM
                for j = 1:nh
                    % Calculate indices for data storage
                    ns = ne + 1;
                    ne = ns + na - 1;
                    nsc = nec + 1;
                    nec = nsc + nde - 1;
                    
                    % Construct output filename
                    optfile = strcat(iptfile, "_", num2str(round(M(i)*10, 0)), "_", num2str(h(j)));
                    
                    % Import DATCOM output data
                    TEMP = datcomimport(strcat('Data\DATCOM\', optfile, '.out'));
                    
                    %% Process longitudinal stability derivatives
                    k = 1;
                    SLNG(ns:ne, k) = TEMP{1}.alpha(:);     k = k + 1;  % Angle of attack
                    SLNG(ns:ne, k) = ones(na, 1)*TEMP{1}.mach; k = k + 1;  % Mach number
                    SLNG(ns:ne, k) = ones(na, 1)*TEMP{1}.alt;  k = k + 1;  % Altitude
                    
                    if EQL       %/* $\leftarrow \starcirc$  */
                        % Equilibrium polar (with trim corrections)
                        SLNG(ns:ne, k) = ReckNaN(TEMP{1}.cl + TEMP{1}.dcl_trim); 
                        k = k + 1;  % Lift coefficient with trim
                        
                        SLNG(ns:ne, k) = ReckNaN(TEMP{1}.cd + TEMP{1}.dcdi_trim + TEMP{1}.dcdmin_trim); 
                        k = k + 1;  % Drag coefficient with trim
                        
                        SLNG(ns:ne, k) = ReckNaN(TEMP{1}.cm*0); 
                        k = k + 1;  % Moment coefficient (zeroed for equilibrium)
                    else
                        % Standard coefficients (no trim corrections)
                        SLNG(ns:ne, k) = ReckNaN(TEMP{1}.cl); k = k + 1;  % Lift coefficient
                        SLNG(ns:ne, k) = ReckNaN(TEMP{1}.cd); k = k + 1;  % Drag coefficient
                        SLNG(ns:ne, k) = ReckNaN(TEMP{1}.cm); k = k + 1;  % Moment coefficient
                    end
                    
                    % Dynamic derivatives
                    SLNG(ns:ne, k) = ReckNaN(TEMP{1}.clad); k = k + 1;  % CL alpha-dot
                    SLNG(ns:ne, k) = ReckNaN(TEMP{1}.cmad); k = k + 1;  % CM alpha-dot
                    SLNG(ns:ne, k) = ReckNaN(TEMP{1}.cmq);  k = k + 1;  % CM pitch rate
                    SLNG(ns:ne, k) = ReckNaN(TEMP{1}.clq);  k = k + 1;  % CL pitch rate
                    
                    %% Process control derivatives (elevator)
                    k = 1;
                    CTR(nsc:nec, k) = ReckNaN(TEMP{1}.delta);          k = k + 1;  % Elevator deflection
                    CTR(nsc:nec, k) = ones(nde, 1)*TEMP{1}.mach; k = k + 1;  % Mach number
                    CTR(nsc:nec, k) = ones(nde, 1)*TEMP{1}.alt;  k = k + 1;  % Altitude
                    CTR(nsc:nec, k) = ReckNaN(TEMP{1}.dcm_sym);        k = k + 1;  % Delta CM due to elevator
                    CTR(nsc:nec, k) = ReckNaN(TEMP{1}.dcl_sym);                % Delta CL due to elevator
                    
                    %% Process lateral-directional derivatives
                    k = 1;
                    SLAT(ns:ne, k) = TEMP{1}.alpha(:);           k = k + 1;  % Angle of attack
                    SLAT(ns:ne, k) = ones(na, 1)*TEMP{1}.mach;   k = k + 1;  % Mach number
                    SLAT(ns:ne, k) = ones(na, 1)*TEMP{1}.alt;    k = k + 1;  % Altitude
                    SLAT(ns:ne, k) = ReckNaN(TEMP{1}.cyb);       k = k + 1;  % Side force due to sideslip
                    SLAT(ns:ne, k) = ReckNaN(TEMP{1}.cnb);       k = k + 1;  % Weathercock stability (yaw moment due to sideslip)
                    SLAT(ns:ne, k) = ReckNaN(TEMP{1}.clb);       k = k + 1;  % Dihedral effect (roll moment due to sideslip)
                    SLAT(ns:ne, k) = ReckNaN(TEMP{1}.clp);       k = k + 1;  % Roll damping (roll moment due to roll rate)
                    SLAT(ns:ne, k) = ReckNaN(TEMP{1}.cnp);       k = k + 1;  % Yaw moment due to roll rate
                    SLAT(ns:ne, k) = ReckNaN(TEMP{1}.clr);       k = k + 1;  % Roll moment due to yaw rate
                    SLAT(ns:ne, k) = ReckNaN(TEMP{1}.cnr);       k = k + 1;  % Yaw damping (yaw moment due to yaw rate)
                    SLAT(ns:ne, k) = ReckNaN(TEMP{1}.cyp);               % Side force due to roll rate
                end
            end
            
            % Save extracted data to MAT file
            save('Data\longitudinal_database_trimmed.mat', 'CTR', 'SLNG', 'SLAT');
    end
end

function veci = ReckNaN(veci)
% RECKNaN Replaces faulty values from DATCOM output with NaN
% This function identifies unrealistically large values (typically errors
% from REDFROMDATCOM) and replaces them with NaN.
    veci(veci > 100) = nan;  % Replace values > 100 with NaN
end