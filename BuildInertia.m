close all; clear;clc
%BuilInertia: script taht calculates the Inertia Tensor used by the ACclass
% and the CG position with respect to the constructive frame used in the
% CAD.

SLG = 14.59390; % 1 slug = 14 kg

IMP = readtable('Data\Test2.csv');
Comp = table2array(IMP(1:end-1,:)); CG = table2array(IMP(end,2:4));
nE = length(Comp(:,1));
IC = nan(3,3,nE);
for i = 1:nE
    MC2Cc = reshape(Comp(i,7:15),3,3);  % DCM From Central to Constructive
    ICc = diag(Comp(i,4:6));            % Inertia Tensor in Central Reference
    IC(:,:,i) = MC2Cc*ICc*(MC2Cc');     % Inertia Tensor in Constructive Reference and also in Body
end

I = zeros(3);

for i = 1:3
    for j = i:3
        for k = 1:nE
            vD =  Comp(k,1:3) - CG(1:3);
            I(i,j) = I(i,j) + 0*IC(i,j,k)*Comp(k,16) + Comp(k,17)*(...
                norm( vD,2 )^2*dlt(i,j) - ...
                vD(i)*vD(j) );
            TST(i,j,k) = Comp(k,17)*(...
                norm( vD,2 )^2*dlt(i,j) - ...
                vD(i)*vD(j) );
        end
    end
end
I(2,1) = I(1,2); I(3,1) = I (1,3); I(3,2) = I(2,3);
%% Save Data
SVF = false;
if SVF
Inertia_Tensor = I;
save('Data\inertia_tensor.mat','Inertia_Tensor','CG')
end
%% Auxiliary Functions

function d = dlt(i,j)
%DLT: Kroeneker delta
    d = 0;
    if i == j
        d = 1;
    end
end

