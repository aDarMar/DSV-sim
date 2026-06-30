close all; clear; clc;
% BUILDAEROMODEL: Script that builds the .mat files used by the ACclass 
% to define the aerodynamic properties of the aircraft.
%
% OUTPUT FILES:
%   - aero_synt:   [M, Re, CD0, K, CL@CDmin, CLmax, alphamax, CLa, CL0, 
%                   CMa, CM0, CLda_a, CLda0, CMda_a, CMda0, CMq, CLq, 
%                   Cyb, Cnb, Clb, Clp, Cnp, Clr, Cnr, Cyp]
%   - aero_synt_C: [M, Re, CMe, CM0e, CLe, CL0e, min(deltaE), max(deltaE)]
%   - aero_synt_M: [alpha, CLm, CL0m, CMm, CM0m]
%-----------------------------------------------------------------------%

% Add required directories to MATLAB path
addpath('Data')
addpath('Classes')
addpath('Functions')
% Load longitudinal database containing aerodynamic data
load('longitudinal_database_trimmed.mat')

nmfl = "Q100_simp.aero";        % Aircraft configuration file name
AC = ACclass(nmfl);

nA = 20;                        % Number of angle of attack points
ndE = 9;                        % Number of elevator deflection points
nM = 4;                         % Number of Mach numbers
nh = 9;                         % Number of altitude points

% Initialize counters and flags
k = 1; 
h = 1; 
kk = 1;
GRF = false;                    % Graphics flag: Controls plotting
DBG = true;                     % Debug flag: For development purposes

% Initialize output arrays
aero_synt = nan(nM*nh,31);      % Main aerodynamic database
aero_synt_C = nan(nM*nh,8);     % Control derivatives database

% Define linearity limit for angle of attack (degrees)
alphal = 8;         
%% --------------------------------------------------------------------- %%
% COEFFICIENT vs. ANGLE OF ATTACK FITTING
% Loop through all Mach numbers and altitudes to extract and fit 
% aerodynamic coefficients
for iM = 1:nM
    for iH = 1:nh
        % Calculate indices for extracting data from the database
        % --------------------------------------------------------------- %
        % Stability Derivatives
        iS = nA*(iH-1) + nA*nh*(iM-1) + 1;          % Starting Index
        iE = nA*iH + nA*nh*(iM-1);                  % Ending index
        % Longitudinal Control
        iSe = ndE*(iH-1) + ndE*nh*(iM-1) + 1;       % Starting Index
        iEe = ndE*iH + ndE*nh*(iM-1);               % Ending index
        % Latero-Directional Control

                                        % End of linearity alpha 
        Re = AC.ReCalc( SLNG(iS,3),SLNG(iS,2) );    % Reynolds Number

        %% Longitudinal 
        % Stability Derivatives
        [CLa,CL0,CLmax,alpham] = CAvsAoa(SLNG(iS:iE,1),...
            SLNG(iS:iE,4),alphal);                          % alpha,CL,alpha lienar limit
        [CD0,K,CLcdm] = polar(SLNG(iS:iE,4),SLNG(iS:iE,5),...
            SLNG(iS:iE,1),alpham);                          % CL,CD,alpha,alpha at max CL
        % Store Parameters
        aero_synt(h,1:9) = [SLNG(iS,2),Re,CD0,K,CLcdm,...   
            CLmax,alpham,CLa,CL0];
        aero_synt(h,10:15) = AeroLinRegs( SLNG(iS:iE,1),... 
            SLNG(iS:iE,6:8),alphal,alpham );                % [CM,CLad,CMad]
        aero_synt(h,16:17) = SLNG(iS,9:10);                 % [CMq,CLq]
        % Control Derivatives
        aero_synt_C(h,1:2) = [SLNG(iS,2),Re];               % [M,Re]
        aero_synt_C(h,3:6) = AeroLinRegs( CTR(iSe:iEe,1),...
            CTR(iSe:iEe,4:5),15 );                          % [CMe,CM0e,CLe,CL0e]
        aero_synt_C(h,7:8) = [min(CTR(iSe:iEe,1))...
            ,max(CTR(iSe:iEe,1)) ];       % [min( deltaE ),max( deltaE ) ]
        
        %% LATERO-DIRECTIONAL
        % Stability Derivatives
        aero_synt(h,18:19) = SLAT(iS,4:5);                  % Cyb,Cnb
        aero_synt(h,20:31) = AeroLinRegs( SLNG(iS:iE,1),...
            SLAT(iS:iE,6:11),alphal,alpham);                % [Clb,Clp,Cnp,Clr,Cnr]
%/* \label{cod:BuildAeroModel1} (continua...) */
        % GRAPHICS GENERATION (Conditional)
        if GRF
            % Aerodynamic Coefficients vs Alpha from DATCOM
            plotv = [SLNG(iS:iE,[1,4:8]),SLAT(iS:iE,6:11)]; % alpha,CL,CD,CM,CLda,CMda
            auxp = CTR(iSe:iEe,[1,4:5]);
            
            % Aerodynamic Coefficients from Regressions
            CL = aero_synt(h,8)*SLNG(iS:iE,1) + aero_synt(h,9);
            CD = aero_synt(h,3) + aero_synt(h,4)*...
                (SLNG(iS:iE,4)-aero_synt(h,5)).^2;
            % Longitudinal Stability
            for iP = 1:3
                graf(:,iP) = aero_synt(h,9+2*iP-1)*SLNG(iS:iE,1) ...
                    + aero_synt(h,9+2*iP);
            end
            % Latero Directional Stability
            for iP = 1:6
                graf(:,iP+3) = aero_synt(h,19+2*iP-1)*SLNG(iS:iE,1) ...
                    + aero_synt(h,19+2*iP);
            end
            % Longitudinal Control
            for iP = 1:2
                graf2(:,iP) = aero_synt_C(h,2+2*iP-1)*CTR(iSe:iEe,1) ...
                    + aero_synt_C(h,2+2*iP);
            end
            % Check DImensions
            if ndE > nA
                plotv = [plotv,auxp,CL,CD,graf,graf2;nan(  ndE-nA,length(plotv(1,:)) ) ]; % FINIRE
            else
                plotv = [ plotv,CL,CD,graf,[auxp,graf2;...
                    nan( nA-ndE,length( [auxp(1,:),graf2(1,:)] ) ) ] ];
            end

            figS = [ ones(13,1),[1,2,ones(1,9),ones(1,2)*24]',...
                [(2:12),25:26]',[(13:23),27:28]' ];

            IMTIT = {'Stability Derivatives'};
            LG = repmat({'-'},13,2);
            
            TIT = {'C$_L$ - $\alpha$','Polar','C$_\mathcal{M}$ - $\alpha$','C$_L_{\dot{\alpha}}$ - $\alpha$',...
                    'C$_{\mathcal{m}_{\dot{\alpha}}}$ - $\alpha$','Cl_\beta - \alpha','Cl_p - \alpha',...
                    'Cn_p - \alpha','Cl_r - \alpha','Cn_r - \alpha','C_{Y_p} - \alpha','CM_{\delta_E} - \delta_E',...
                    'CL_{\delta_E} - \delta_E'};
            LINS = repmat({'o','--'},13,1);
            XLAB = repmat({'$\alpha$ [deg]'},11,1); XLAB = [XLAB;repmat({'$\delta_e$ [deg]'},2,1)]; 
            XLAB{2} = '$C_L$';
            YLAB = {'$C_L$','$C_D$','C$_\mathcal{M}$','C$_{L_{\dot{\alpha}}}$','C$_{\mathcal{M}_{\dot{\alpha}}}$',...
                'C$_{\mathcal{L}_\beta}$','C$_{\mathcal{L}_p}$','C$_{\mathcal{N}_p}$','C$_{\mathcal{L}_r}$',...
                'C$_{\mathcal{N}_r}$','C$_{Y_p}$','C$_{L_{\delta_e}}$','C$_{\mathcal{M}_{\delta_e}}$' };
            plotd = DataPlot(plotv);
            for ig = 1:length(XLAB)
                plotd = plotd.definePlot(figS(ig,2),figS(ig,3:end),figS(ig,1),'legend',LG(ig,:),...
                    'grid','minor','xlabel',XLAB{ig},...
                    'ylabel',YLAB{ig},'linestyle',LINS(ig,:));
            end

            plotd.PlotPerImag( 3,IMTIT,...
                {strcat( 'M = ',num2str(SLNG(iS,2)),' Re = ',num2str( round( Re,0 ) ) )},'cartesian',false );

        end
        h = h + 1;
    end
    
end
%% --------------------------------------------------------------------- %%
%/* (continua da \ref{cod:BuildAeroModel1}) */
% COEFFICIENT vs. MACH NUMBER FITTING
aero_synt_M = nan(nA,4);
h = 1; iM = 1:nM;
for iA = 1:nA
    % Calculate indices for same angle of attack across all Mach numbers
    q = iA + nA*nh*(iM-1);
    aero_synt_M(h,2:5) = AeroLinRegs( SLNG(q,2),... 
        SLNG(q,[4,6]) );                % [CL,CM]
    aero_synt_M(h,1) = SLNG(iA,1);

    if GRF
        plotvsM( alpha(iA),M,CL,CM );
    end
    h = h + 1;
end

%% --------------------------------------------------------------------- %%
% Save Results
SVF = false;
if SVF
    save('Data\aero_database_long.mat','aero_synt','aero_synt_M','aero_synt_C');
end

%% --------------------------------------------------------------------- %%
% Auxiliary Functions 

function [CAa,CA0,CAmax,alpham] = CAvsAoa(alpha,CA,alphalin,alpham)
% CAVSAOA: Extracts linear region characteristics from coefficient vs. 
%          angle of attack data
%
% INPUTS:
%   alpha    - Angle of attack vector [deg]
%   CA       - Aerodynamic coefficient vector
%   alphalin - Linearity limit for angle of attack [deg]
%   alpham   - (Optional) Angle of attack at maximum coefficient
%
% OUTPUTS:
%   CAa      - Slope of linear region (dCA/dalpha)
%   CA0      - Zero-angle intercept
%   CAmax    - Maximum coefficient value
%   alpham   - Angle of attack at maximum coefficient
% ----------------------------------------------------------------------- %
    CAmax = max(CA);    % Max value of CA. For variables different from CL it has no meaning
    if nargin < 4
        % If alpha_max is not passed it is evaluated. NOTE: the results is
        % meaningful only if CA represents CL
        alpham = alpha( CA==CAmax );
    end
    % Removing NaNs
    CA = CA( ~isnan(CA) );
    alpha = alpha( ~isnan(CA) );
    % Find index of maximum coefficient
    i = 1:length(CA);
    im = i(alpha==alpham);
    % Handle edge cases for empty arrays or missing maximum
    if isempty(im) && isempty(CA)
        % If CA is empty the values are set as NaN
        CA = nan;
        alpha = nan;
    elseif isempty(im)
        % If the value is not found BUT CA is not empty than use last alpha
        im = i(end);
        CAmax = CA(im);
        alpha = alpha(~(i>im)); % Excludes post stall
        CA = CA(~(i>im));
    end
    % Filter data to CL linear region only
    CA = CA(all([~(alpha>alphalin),~(alpha<-alphalin)],2)); 
    alpha = alpha(all([~(alpha>alphalin),~(alpha<-alphalin)],2));
    % Set polynomial order for linear fit
    ord = 1;
    % Perform linear regression
    if ~(length(CA)>ord)
        % The order of the polinomial is greater than the number of points
        % Works only if ord = 1
        p2 = polyfit(alpha,CA,min( ord,length(CA)-1 ) );
        CAa = nan; CA0 = p2;
    else
        p2 = polyfit(alpha,CA,ord);
        CAa = p2(1); CA0 = p2(2);
    end
end

function [CD0,K,CLcdm,plr] = polar(CL,CD,alpha,alpham)
% POLAR: Extracts parabolic drag polar characteristics
%
% INPUTS:
%   CL     - Lift coefficient vector
%   CD     - Drag coefficient vector
%   alpha  - Angle of attack vector [deg]
%   alpham - Maximum angle of attack for linear region [deg]
%
% OUTPUTS:
%   CD0    - Zero-lift drag coefficient
%   K      - Induced drag factor
%   CLcdm  - Lift coefficient at minimum drag
%   plr    - Function handle for drag polar calculation
% ----------------------------------------------------------------------- %
    plr = @polbuild;
    CL = CL(alpha<alpham); CD = CD(alpha<alpham);
    p1 = polyfit(CL,CD,2);
    % Extract polar parameters from quadratic coefficients
    K = p1(1); CLcdm = -0.5*p1(2)/p1(1); CD0 = p1(3) - K*CLcdm^2;
    % Define function handle for drag polar calculation
        function out = polbuild(CL)
            out = p1(1)*CL.^2 + p1(2)*CL + p1(3);
        end
end

function reslin = AeroLinRegs(alpha,CA,alphal,alpham)
% AEROLINREGS: Performs linear regression on multiple aerodynamic
%              coefficients vs. angle of attack
%
% INPUTS:
%   alpha  - Angle of attack vector [deg]
%   CA     - Matrix of aerodynamic coefficients (each column a different coefficient)
%   alphal - Linearity limit for angle of attack [deg]
%   alpham - Maximum angle of attack for linear region [deg]
%
% OUTPUTS:
%   reslin - Vector of linear regression coefficients [slope1, intercept1,
%            slope2, intercept2, ...]
% ----------------------------------------------------------------------- %
    % Set default linearity limits if not provided
    if nargin <3
        alphal = max(alpha); alpham = max(alpha);
    elseif nargin < 4
        alpham = alphal;
    end
    % Determine number of coefficients to process
    nipt = length( CA(1,:) );
    reslin = nan(1,nipt*2);
    for iL = 1:nipt
        % [Clb,Clp,Cnp,Clr,Cnr]
        [reslin(2*iL-1),reslin(2*iL),~,~] = CAvsAoa(alpha,CA(:,iL),...
            alphal,alpham);
    end
end