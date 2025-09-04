%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% File for Logical Occupancy Matrix Generation for Map 3

% 1. Generation of a Logical Matrix for the L-corridor

mapSize=20;          % Map size in [m]
corridorWidth=2.0;   % Corridor width in [m]
wallThickness=3;     % Wall thickness in [m]

logicalMatSize=round(mapSize*map.resolution);
corridorGrids=round(corridorWidth*map.resolution);
wallGrids=round(wallThickness*map.resolution);

map.logicalMat=zeros(logicalMatSize, logicalMatSize);

% Upper Wall
for i=1:wallGrids
    for j=1:logicalMatSize
        map.logicalMat(i, j)=1;
    end
end

% Right Wall
for i=1:logicalMatSize
    for j=logicalMatSize-wallGrids:logicalMatSize
        map.logicalMat(i, j)=1;
    end
end

% Lower Left Corner
for i=wallGrids+corridorGrids:logicalMatSize
    for j=1:logicalMatSize-(wallGrids+corridorGrids)
        map.logicalMat(i, j)=1;
    end
end

clear i j mapSize corridorWidth wallThickness logicalMatSize corridorGrids wallGrids