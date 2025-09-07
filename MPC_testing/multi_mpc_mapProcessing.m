%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% Processing of the Logical Occupancy Object into Map Related Objects

addpath('C:\Users\jinwo\OneDrive\바탕 화면\LARR\Program_Files\casadi-3.7.0-windows64-matlab2018b')
import casadi.*
% NOTE: casADi used to generate a symbolic signed-distance map

showMapReport=true;

% 1. Generation of a Binary Occupancy Map and an Inflated Binary Occupancy Map
map.binaryMap=...
    binaryOccupancyMap(map.logicalMat, Resolution=map.resolution, LocalOriginInWorld=map.OriginInWorldCoord);

map.binaryMap_inflated=...
    binaryOccupancyMap(map.logicalMat, Resolution=map.resolution, LocalOriginInWorld=map.OriginInWorldCoord);
if map.inflation>0
    inflate(map.binaryMap_inflated, 0.5*map.inflation);
end

map.logicalMat_inflated=occupancyMatrix(map.binaryMap_inflated);

tic;

fprintf("Generating the Numerical Signed Distance Map...\n")

% 2. Numerical Signed-Distance Map
map.signedDistMap=....
    signedDistanceMap(map.logicalMat, Resolution=map.resolution, LocalOriginInWorld=map.OriginInWorldCoord);

timeReport.mapProcessing_signedDist=toc;   % Measure time to generate signed distance map

fprintf("Generation of the Numerical Signed Distance Map completed in %.4f seconds! \n\n",...
    timeReport.mapProcessing_signedDist)


% 3. Symbolic Signed-Distance Map
tic;

fprintf("Generating the Symbolic Signed Distance Map...\n")

southwestPosition=grid2world(map.binaryMap, [length(map.logicalMat(:, 1)), 1]);
northeastPosition=grid2world(map.binaryMap, [1, length(map.logicalMat(1, :))]);

x_grid=southwestPosition(1):map.symbolicInterpolationInterval:northeastPosition(1);
y_grid=southwestPosition(2):map.symbolicInterpolationInterval:northeastPosition(2);

signedDistData=zeros(length(x_grid), length(y_grid));   % Matrix contianing numerical signed distance values across the map
obstThickness=sqrt(2)*0.5/map.resolution;   % Obstacle thickness. Must be subtracted from signed distance values.
for i=1:length(x_grid)
    for j=1:length(y_grid)
        signedDistData(i, j)=...
            distance(map.signedDistMap, [x_grid(i), y_grid(j)])-obstThickness;
    end
end

map.signedDistMap_symbolic=...
    casadi.interpolant('signedDistMap_symbolic', 'linear', {x_grid, y_grid}, double(signedDistData(:)));

timeReport.mapProcessing_symbolic=toc;   % Measure time to generate signed distance map

fprintf("Generation of the Symbolic Signed Distance Map completed in %.4f seconds! \n\n",...
    timeReport.mapProcessing_symbolic)

% 5. Showing Results of Map Object Generation
if showMapReport
    figure("Name", "Map Report")
    hold on
    show(map.signedDistMap, BoundaryColor=[0, 0, 0], Colorbar='on')
    title("The Map")
    hold off
end

clearvars -except params map user timeReport selectMap