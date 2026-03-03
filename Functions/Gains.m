function [Kp,Ki,Kb,lam] = Gains(wn,zita,Ai,Bi,Ci,D,CHS,p)
%Gains Calculates controller gains (Kp, Ki, Kb) using pole placement.
% This function determines the proportional (Kp), integral (Ki), and
%   feedback (Kb) gain matrices for longitudinal control laws based on 
%   desired natural frequency (wn) and damping ratio (zita). The gains
%   are calculated by solving a system of algebraic equations derived from
%   matching the closed-loop characteristic polynomial with the desired one.
%
% INPUT:
%   - wn: Desired natural frequency of the closed-loop system [rad/s]
%   - zita: Desired damping ratio of the closed-loop system [-]
%   - Ai: Linearized State Matrix A
%   - Bi: Linearized Input Matrix B
%   - Ci: Linearized Output Matrix C
%   - D: Linearized Feedforward Matrix D (unused in the provided code)
%   - CHS: Control Scheme Selector ('CLcHd', 'CLcV', or 'CLTcVh')
%   - p: Vector of desired locations for the remaining closed-loop poles (or a single pole for 'CLcV').
%
% OUTPUT:
%   - Kp: Proportional gain matrix (2x4)
%   - Ki: Integral gain matrix (2x4)
%   - Kb: Feedback gain matrix (2x4)
%   - lam: Vector of all closed-loop eigenvalues (poles)
% ----------------------------------------------------------------------- %
    % Initialize gain matrices
    Kb = zeros(2,4); Ki = zeros(2,4); Kp = zeros(2,4);                          % Gain Matrices
    % Desired characteristic polynomial terms (phugoid)
    wns = wn^2; zwn = 2*zita*wn;

switch CHS
    case 'CLcHd' %/*$\leftarrow$ \autoref{sub:CLh}  */
        % --- Control Law: CL for hdot ---
        GN = [ Bi(2,1)*Ci(4,2),0,1; ...
            Ci(4,2)*( Ai(2,1)*Bi(1,1) - Ai(1,1)*Bi(2,1) ), Bi(2,1)*Ci(4,2) , zwn; ...
            0, Ci(4,2)*( Ai(2,1)*Bi(1,1) - Ai(1,1)*Bi(2,1) ), wns ];
        % The target characteristic polynomial coefficients
        RHS = [ Ai(1,1)+zwn;wns + Ai(1,2)*Ai(2,1); p];
        % Solve for gains kcl and store them
        kcl = GN\RHS; Kp(1,4) = kcl(1); Ki(1,4) = kcl(2);
        % Root Locus analysis for the determined gains
        rLocusDef(Ai,Bi,Ci,4,1);                                                   % Kp14
        Kit = zeros(4,4); Kit(1,4) = kcl(1); Ain = Ai - Bi*Kit*Ci;
        rLocusDef(Ain,Bi,Ci,4,3);
        Kit = zeros(4,4); Kit(3,4) = kcl(2); Ain = Ain - Bi*Kit*Ci;                % Ki14
        lam = eig(Ain);

    case 'CLcV'%/*$\leftarrow$ \autoref{sub:CLV}  */
        % --- Control Law: CL for V  ---
        C2 = -p + zwn; C3 = -p*zwn + wns; C4 = -p*wns;
        
        Kb(1,4) = ( C3 - ( Ai(1,2)*Bi(2,1)*C2/Bi(1,1) - Ai(1,2)*Ai(2,1) ...
            + Ai(1,1)*Ai(1,2)*Bi(2,1)/Bi(1,1) + C4*Bi(1,1)/( Ai(1,2)*Bi(2,1) ) ) )...
            /( Ai(2,1)*Ci(4,2)*Bi(1,1) - Ai(1,2)*Bi(2,1)*Ci(4,2)*Bi(2,1)/Bi(1,1)...
            - Ai(1,1)*Ci(4,2)*Bi(2,1) ); % V <-> h
        Kp(1,1) = ( C2-Ci(4,2)*Bi(2,1)*Kb(1,4)+Ai(1,1) )/(Bi(1,1)*Ci(1,1));% V <-> VIAS proportional

        Ki(1,1) = C4/( Ai(1,2)*Bi(2,1)*Ci(1,1) ); % V <-> VIAS integral

        rLocusDef(Ai,Bi,Ci,4,1);                                                   % Kp14
        Kit = zeros(4,4); Kit(1,4) = Kb(1,4); Ain = Ai - Bi*Kit*Ci;
        rLocusDef(Ain,Bi,Ci,1,1);
        Kit = zeros(4,4); Kit(1,1) = Kp(1,1); Ain = Ain - Bi*Kit*Ci;                % Ki14
        rLocusDef(Ain,Bi,Ci,1,3);
        Kit = zeros(4,4); Kit(3,1) = Ki(1,1); Ain = Ain - Bi*Kit*Ci; 
        lam = eig(Ain);

    case 'CLTcVh'%/*$\leftarrow$ \autoref{sub:CLTVh}  */
        % --- Control Law: CL and Thrust for V and h ---
        C2 = zwn - p(1)-p(2); C3 = wns - zwn*(p(1)+p(2)) + p(1)*p(2);
        C4 = zwn*p(1)*p(2) - wns*(p(1)+p(2)); C5 = wns*p(1)*p(2);

        ks = fsolve(@sysol,[1e-5,1e-3,1e-3,1e-3]);
        Kp(2,1) = ks(1); Kp(1,4) = ks(2); Ki(1,4) = ks(3); Ki(2,1) = ks(4);
        rLocusDef(Ai,Bi,Ci,4,1);                                                   % Kp14
        Kit = zeros(4,4); Kit(1,4) = Kb(1,4); Ain = Ai - Bi*Kit*Ci;
        rLocusDef(Ain,Bi,Ci,4,3);
        Kit = zeros(4,4); Kit(3,4) = Ki(1,4); Ain = Ain - Bi*Kit*Ci;                % Ki14
        rLocusDef(Ain,Bi,Ci,1,3);
        Kit = zeros(4,4); Kit(2,1) = Kp(2,1); Ain = Ain - Bi*Kit*Ci; 
        rLocusDef(Ain,Bi,Ci,1,4);
        Kit = zeros(4,4); Kit(4,1) = Ki(2,1); Ain = Ain - Bi*Kit*Ci; 
        lam = eig(Ain);
end

    function F = sysol(x)
        % x(1) -> Kp(2,1)
        % x(2) -> Kp(1,4)
        % x(3) -> Ki(1,4)
        % x(4) -> Ki(2,1)
        F = nan(4,1);
        F(1) = C2 + Ai(1,1) - Bi(1,2)*Ci(1,1)*x(1) - Bi(2,1)*Ci(4,2)*x(2);
        F(2) = C3 + Ai(1,1)*Bi(2,1)*Ci(4,2)*x(2) - ...
            Bi(1,2)*Ci(1,1)*Bi(2,1)*Ci(4,2)*x(1)*x(2) - Bi(2,1)*Ci(4,2)*x(3)...
            + Ai(2,1)*Ai(1,2) - Ai(2,1)*Bi(1,1)*Ci(4,2)*x(2) - Bi(1,2)*Ci(1,1)*x(4);
        F(3) = C4 - Bi(1,2)*Bi(2,1)*Ci(4,2)*Ci(1,1)*x(2)*x(4) - ...
            Ai(2,1)*Bi(1,1)*Ci(4,2)*x(3) + Bi(2,1)*Ci(4,2)*Ai(1,1)*x(3)-...
            Bi(2,1)*Ci(4,2)*Bi(1,2)*Ci(1,1)*x(1)*x(3);
        F(4) = C5-Ci(1,1)*Bi(1,2)*Bi(2,1)*Ci(4,2)*x(4)*x(3);
    end

end

