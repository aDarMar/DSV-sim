function [wn,zita] = OmZitaCalc(lam,A)
    lam = lam(abs(lam)~=0); nlam = length(lam);
    wn = nan(nlam,1); zita = wn;
    for ic = 1:nlam
        if imag(lam(ic)) ~=0 % Check if underdamped
            zita(ic) = imag(lam(ic))/real(lam(ic));
            zita(ic) = 1/sqrt( 1 + zita(ic)^2 );
            wn(ic) = -real(lam(ic))/zita(ic);
        else
            % TODO overdamped and critically damped
            disp('a')
        end
    end
end