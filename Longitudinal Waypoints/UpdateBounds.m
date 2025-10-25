function bounds = UpdateBounds(x_way)
%UPDATEBOUNDS Updates the bounds for the various fligh regions in the V-h
%error diagram
%   INPUT
%   - x_way: state vector of reference condition
%   OUTPUT
%   - bounds: [Vbound,hbound,V/g]

%y_way = LongDynNoLin_Out(x_way);
bounds = nan(3,1);
bounds(1) = IAS2TAS(5.1444,x_way(3));
bounds(3) = - x_way(1)/9.81;        % Va/g @x_way
bounds(2) = - bounds(3) * bounds(1); % dh

end

function TAS = IAS2TAS(IAS,h)
% Tested comparing IAS obtained from LongDynNoLin_Out
    gm = 1.4;
    [~,a,P,~,~,~] = atmosisa([0,h]);
    TAS = 2/(gm-1)*( ( P(1)/P(2)*( ( 1+0.5*(gm-1)*(IAS/a(1))^2 )^(gm/(gm-1)) ...
        - 1 ) + 1 )^(1-1/gm) - 1 );
    TAS = sqrt(TAS)*a(2);
end