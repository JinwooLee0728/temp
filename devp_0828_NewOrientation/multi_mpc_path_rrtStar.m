%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% Construction of the RRT* Planner

tic;   % Start Timer

testPlanner=true;

% 1. Generation of an SE(2) State Space Object
x_range=map.binaryMap.XWorldLimits;
y_range=map.binaryMap.YWorldLimits;

stateBounds=...
    [x_range(1), x_range(2);...
    y_range(1), y_range(2);...
    -pi(), +pi()];
weightXY=1;
weightTheta=0.1;

stateSpace=stateSpaceSE2;
stateSpace.StateBounds=stateBounds;
stateSpace.WeightXY=weightXY;
stateSpace.WeightTheta=weightTheta;

% 2. Generation of an Occupancy Valication Object
occupancyValidator=validatorOccupancyMap(stateSpace);
occupancyValidator.Map=map.binaryMap_inflated;
occupancyValidator.ValidationDistance=0.2;

% 3. Construction of the RRT Planner
rrtStarPlanner=...
    plannerRRTStar(stateSpace, occupancyValidator,...
    ContinueAfterGoalReached=false, BallRadiusConstant=10, MaxIterations=5e+4, MaxConnectionDistance=10.0);

timeReport.pathStarPlannerBuild=toc;   % End timer


% Testing the Planner
if testPlanner
    pose1=user.pose_init';
    pose2=user.pose_fnal';
    [pathObject, solInfo]=plan(rrtStarPlanner, pose1, pose2);

    figure("Name", "Test Results for RRT* Planner")
    hold on
    show(map.binaryMap_inflated);
    plot(solInfo.TreeData(:, 1), solInfo.TreeData(:, 2), '-b')
    plot(pathObject.States(:, 1), pathObject.States(:, 2), '-ro', LineWidth=1.5)
    plot(pose1(1), pose1(2), 'Color', 'red', 'Marker', 'o', 'MarkerFaceColor','red', 'MarkerSize', 10)
    plot(pose2(1), pose2(2), 'Color', 'red', 'Marker', 'square', 'MarkerFaceColor','red', 'MarkerSize', 10)
    hold off
    title("Test Results for RRT* Planner")
    subtitle("RRT* planner results using starting and ending positions on an inflated map")
end

clear occupancyValidator pathObject pose1 pose2 solInfo stateSpace stateBounds
clear testPlanner weightTheta weightXY x_range y_range