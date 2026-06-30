function bounds = UpdateBounds(x_way)
%UPDATEBOUNDS Updates the bounds for the various fligh regions in the V-h
%error diagram
%   INPUT
%   - y_way: state vector of reference condition
%   OUTPUT
%   - bounds: [Vbound,hbound] in [kts,ft]

    bounds = nan(2,1);
    
    bounds(1) = 10; % 10 kts of IAS %5.1444; % 10 kts IAS
    bounds(2) = x_way(1)/9.81 * IAS2TAS(5.1444,x_way(3),'SI')/0.305; % (Va/g)*dV(TAS) in ft

end

