close all; clear;clc
% BUILDINERTIA: This script calculates the total Inertia Tensor (I) of an
% aircraft assembly based on the Inertia tensors of its components and the 
% position of its Center of Mass.
% The calculation uses data for multiple individual components (mass, inertia,
% position, and orientation) relative to a Constructive Frame (CAD frame).
% The final Inertia Tensor is relative to the calculated CG and expressed in the
% Body Frame using the Parallel Axis Theorem.
% INPUTS:
%   'Data\Test2.csv': CSV file containing component data.
%                     - Component rows (1 to end-1): Position, Inertia, DCM, Flags.
%                     - Last row: Global CG position.
% OUTPUTS:
%   I: The total 3x3 Inertia Tensor matrix relative to the CG (in the Constructive Frame).
%   CG: The Center of Gravity position vector [x, y, z] relative to the Constructive Frame.
%
SLG = 14.59390; % 1 slug = 14 kg

IMP = readtable('Data\InertiaData.csv');
% Comp: Component data (all rows except the last).
% CG: Global Center of Gravity position (last row, columns 2-4).
Comp = table2array(IMP(1:end-1,:)); CG = table2array(IMP(end,2:4));
nE = length(Comp(:,1));                 % Number of components.
% Pre-allocation for the Inertia Tensor of each component in the Constructive Frame.
IC = nan(3,3,nE); 
%% --------------------------------------------------------------------- %% 
% Transform Component Inertia Tensors to Constructive Frame
for i = 1:nE
    MC2Cc = reshape(Comp(i,7:15),3,3);  % DCM From Central to Constructive
    ICc = diag(Comp(i,4:6));            % Inertia Tensor in Central Reference
    % Transforms the Inertia Tensor from Central to Constructive Frame
    % using the rotation formula: I_Constructive = M * I_Central * M'
    IC(:,:,i) = MC2Cc*ICc*(MC2Cc');     % Inertia Tensor in Constructive Reference and also in Body
end
%% --------------------------------------------------------------------- %% 
% Calculate Total Inertia Tensor (I) using the Parallel Axis Theorem
I = zeros(3);

for i = 1:3             % Row index (x, y, z)
    for j = i:3         % Column index (x, y, z) - calculates only upper triangle (I_ij = I_ji).
        for k = 1:nE    % Loop over each component.
            vD =  Comp(k,1:3) - CG(1:3);
            I(i,j) = I(i,j) + IC(i,j,k)*Comp(k,16) + Comp(k,17)*(...
                norm( vD,2 )^2*dlt(i,j) - ...
                vD(i)*vD(j) );
        end
    end
end
% Fills the lower triangular part of the symmetric Inertia Tensor matrix.
I(2,1) = I(1,2); I(3,1) = I (1,3); I(3,2) = I(2,3);
%% --------------------------------------------------------------------- %% 
% SAVE DATA
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

