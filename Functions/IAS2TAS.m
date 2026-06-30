function TAS = IAS2TAS(IAS,h,unit)
%IAS2TAS Returns the true airspeed given the indicated airspeed at given
%height
%   - IAS: indicated airpeed in kts or m/s
%   - h: altitude in ft or m
%   -unit: imperial or SI units
% Tested comparing IAS obtained from LongDynNoLin_Out
    if strcmp(unit, 'ktsh')
        IAS = IAS*0.5144;   % from kts to m/s
        h = h*0.305;        % from ft to m
    end
    gm = 1.4;
    [~,a,P,~,] = atmosisa([0,h]);
    TAS = 2/(gm-1)*( ( P(1)/P(2)*( ( 1+0.5*(gm-1)*(IAS/a(1))^2 )^(gm/(gm-1)) ...
        - 1 ) + 1 )^(1-1/gm) - 1 );
    TAS = sqrt(TAS)*a(2);

end

