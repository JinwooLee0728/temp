%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% File for Logical Occupancy Matrix Generation for Map 2

% 1. Importation of MATLAB "complexMap"
load exampleMaps.mat

% 2. Logical Occupancy Matrix
temp=complexMap;
map.logicalMat=zeros(map.resolution*length(temp(:, 1)), map.resolution*length(temp(1, :)));

for i=1:length(temp(:, 1))
    for j=1:length(temp(1, :))
        if temp(i, j)==1
            for k=map.resolution*(i-1)+1:map.resolution*i
                for l=map.resolution*(j-1)+1:map.resolution*j
                    map.logicalMat(k, l)=1;   % Mark as obstacle
                end
            end
        end
    end
end

clear complexMap emptyMap simpleMap ternaryMap temp i j