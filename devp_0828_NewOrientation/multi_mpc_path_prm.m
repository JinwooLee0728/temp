%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% Construction of the PRM Planner

tic;   % Start Timer

testPlanner=true;

prmPlanner=mobileRobotPRM(map.binaryMap_inflated, 700);
prm.ConnectionDistance=10.0;

timeReport.pathPlannerBuild=toc;   % End timer

if testPlanner
    posi1=user.pose_init(1:2)';
    posi2=user.pose_fnal(1:2)';
    path=findpath(prmPlanner, posi1, posi2);

    figure("Name", "Test Results for PRM Planner")
    hold on
    show(prmPlanner)
    plot(path(:, 1), path(:, 2), '-r', LineWidth=3)
    plot(posi1(1), posi1(2), 'Color', 'red', 'Marker', 'o', 'MarkerFaceColor','red', 'MarkerSize', 10)
    plot(posi2(1), posi2(2), 'Color', 'red', 'Marker', 'square', 'MarkerFaceColor','red', 'MarkerSize', 10)
    title("Test Results for PRM Planner")
    subtitle("PRM planner results using starting and ending positions on an inflated map")
    hold off
    
end