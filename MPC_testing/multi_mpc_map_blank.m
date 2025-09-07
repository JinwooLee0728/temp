%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% File for Generating a Blank Map Object

mapSize=10;    % Map height and width, in [m]

logicalMatSize=round(mapSize*map.resolution);
map.logicalMat=ones(logicalMatSize, logicalMatSize);

for i=2:logicalMatSize-1
    for j=2:logicalMatSize-1
        map.logicalMat(i, j)=0;
    end
end