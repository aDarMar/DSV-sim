function bounds = UpdateBounds(x_way)
%UPDATEBOUNDS Updates the bounds for the various fligh regions in the V-h
%error diagram
%   INPUT
%   - y_way: state vector of reference condition
%   OUTPUT
%   - bounds: [Vbound,hbound] in [kts,ft]

    bounds = nan(2,1);
    
    bounds(1) = 10; % 10 kts of IAS %5.1444; % 10 kts IAS
    bounds(2) = x_way(1)/9.81 * IAS2TAS(5.1444,x_way(3))/0.305; % (Va/g)*dV(TAS) in ft

end

function TAS = IAS2TAS(IAS,h)
% Tested comparing IAS obtained from LongDynNoLin_Out
    gm = 1.4;
    [~,a,P,~,] = atmosisa([0,h]);
    TAS = 2/(gm-1)*( ( P(1)/P(2)*( ( 1+0.5*(gm-1)*(IAS/a(1))^2 )^(gm/(gm-1)) ...
        - 1 ) + 1 )^(1-1/gm) - 1 );
    TAS = sqrt(TAS)*a(2);
end