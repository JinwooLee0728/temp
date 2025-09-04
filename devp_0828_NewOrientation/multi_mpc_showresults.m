%% ############ Multiagent Transportation Project: 2025.08.13 #############
%% Presentation of Results

close all

% Time array
time=zeros(1, length(mpc_results.up(1, :)));
for i=1:length(time)
    time(i)=(i-1)*params.con.t_delta;
end

% Toggle if should show figure
showfig.Animation=true;
showfig.MapAndTraj=true;
showfig.StatesAgainstTime=true;
showfig.PseudoinputsAgainstTime=true;
showfig.YawRates=false;
figshow.Headings=true;
showfig.InputsAgainstTime=true;
showfig.ComputationTime=true;
showfig.Verification=true;
showfig.MinSignedDistances=true;

%close all
%% Figure: Animation of Movements Along the Trajectory
if showfig.Animation==true
    %gifFile='MotionAnimation.gif';
    figure('Name', 'Animation')
    hold on
    % 1. Plotting Trajectories
    show(map.signedDistMap, BoundaryColor=[0, 0, 0],Colorbar='on')
    plot(user.pose_init(1, 1), user.pose_init(2, 1),...
        'c', 'Marker', 'square', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'DisplayName', 'Initial Point')
    plot(user.pose_fnal(1, 1), user.pose_fnal(2, 1),...
        'c', 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'DisplayName', 'Final Point')
    plot(ref.x, ref.y,...
        '-c', 'LineWidth', 3, 'DisplayName', 'Voronoi Ref. Traj.')
    plot(mpc_results.state(1, :), mpc_results.state(2, :),...
        '-b', 'LineWidth', 1.5, 'DisplayName', 'Actual Traj.')

    % 2. Plotting Animation of Motions
    for i=1:length(time)
        posiB=mpc_results.state(1:2, i); thetaB=mpc_results.state(3, i);
        Rot_BtoW=...
            [cos(thetaB), -sin(thetaB);...
            sin(thetaB), cos(thetaB)];
        
        % Vertices of carts
        p1=posiB+Rot_BtoW*[-params.sys.cart_wdt/2; -params.sys.cart_hgt/2];
        p2=posiB+Rot_BtoW*[+params.sys.cart_wdt/2; -params.sys.cart_hgt/2];
        p3=posiB+Rot_BtoW*[+params.sys.cart_wdt/2; +params.sys.cart_hgt/2];
        p4=posiB+Rot_BtoW*[-params.sys.cart_wdt/2; +params.sys.cart_hgt/2];

        % Circles representing robots
        angle=0:0.1:2*pi();
        circles=params.sys.robo_sze*[cos(angle); sin(angle)];

        robot1_center=posiB+Rot_BtoW*params.sys.r_BtoR(:, 1);
        robot2_center=posiB+Rot_BtoW*params.sys.r_BtoR(:, 2);
        robot3_center=posiB+Rot_BtoW*params.sys.r_BtoR(:, 3);
        robot4_center=posiB+Rot_BtoW*params.sys.r_BtoR(:, 4);

        vert_robot1=circles+[robot1_center(1)*ones(1, length(circles(1, :))); robot1_center(2)*ones(1, length(circles(2, :)))];
        vert_robot2=circles+[robot2_center(1)*ones(1, length(circles(1, :))); robot2_center(2)*ones(1, length(circles(2, :)))];
        vert_robot3=circles+[robot3_center(1)*ones(1, length(circles(1, :))); robot3_center(2)*ones(1, length(circles(2, :)))];
        vert_robot4=circles+[robot4_center(1)*ones(1, length(circles(1, :))); robot4_center(2)*ones(1, length(circles(2, :)))];

        if i>1
            delete(patch_cart)
            delete(patch_robot1)
            delete(patch_robot2)
            delete(patch_robot3)
            delete(patch_robot4)
            delete(arrow1)
            delete(arrow2)
            delete(arrow3)
            delete(arrow4)
            delete(arrow1_ac)
            delete(arrow2_ac)
            delete(arrow3_ac)
            delete(arrow4_ac)
            delete(ref_traj)
            delete(ref_wpts)
            delete(posiB_plot)
        end
        
        % Draw Robots
        patch_robot1=patch('XData', vert_robot1(1, :), 'YData', vert_robot1(2, :),...
            'FaceColor', [1, 0, 0], 'EdgeColor', 'k', 'LineWidth', 0.5);
        patch_robot2=patch('XData', vert_robot2(1, :), 'YData', vert_robot2(2, :),...
            'FaceColor', [0, 1, 0], 'EdgeColor', 'k', 'LineWidth', 0.5);
        patch_robot3=patch('XData', vert_robot3(1, :), 'YData', vert_robot3(2, :),...
            'FaceColor', [0, 0, 1], 'EdgeColor', 'k', 'LineWidth', 0.5);
        patch_robot4=patch('XData', vert_robot4(1, :), 'YData', vert_robot4(2, :),...
            'FaceColor', [1, 0, 1], 'EdgeColor', 'k', 'LineWidth', 0.5);

         % Draw Cart
        patch_cart=patch('XData', [p1(1), p2(1), p3(1), p4(1)], 'YData', [p1(2), p2(2), p3(2), p4(2)],...
            'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'k', 'LineWidth', 0.5);

        scale=1;

        arrow1=annotation('arrow');
        set(arrow1, 'parent', gca,...
            'position', [robot1_center(1), robot1_center(2), scale*cos(robo.desired_headings(1, i)), scale*sin(robo.desired_headings(1, i))], ...
            'Color', 'red', 'Headlength', 7, 'Headwidth', 7, 'LineWidth', 1.5);
        arrow2=annotation('arrow');
        set(arrow2, 'parent', gca,...
            'position', [robot2_center(1), robot2_center(2), scale*cos(robo.desired_headings(2, i)), scale*sin(robo.desired_headings(2, i))], ...
            'Color', 'red', 'Headlength', 7, 'Headwidth', 7, 'LineWidth', 1.5);
        arrow3=annotation('arrow');
        set(arrow3, 'parent', gca,...
            'position', [robot3_center(1), robot3_center(2), scale*cos(robo.desired_headings(3, i)), scale*sin(robo.desired_headings(3, i))], ...
            'Color', 'red', 'Headlength', 7, 'Headwidth', 7, 'LineWidth', 1.5);
        arrow4=annotation('arrow');
        set(arrow4, 'parent', gca,...
            'position', [robot4_center(1), robot4_center(2), scale*cos(robo.desired_headings(4, i)), scale*sin(robo.desired_headings(4, i))], ...
            'Color', 'red', 'Headlength', 7, 'Headwidth', 7, 'LineWidth', 1.5);

        % Draw Actual Robot Headings
        arrow1_ac=annotation('arrow');
        set(arrow1_ac, 'parent', gca,...
            'position', [robot1_center(1), robot1_center(2), scale*cos(mpc_results.state(4, i)), scale*sin(mpc_results.state(4, i))], ...
            'Color', 'blue', 'Headlength', 7, 'Headwidth', 7, 'LineWidth', 1.5);
        arrow2_ac=annotation('arrow');
        set(arrow2_ac, 'parent', gca,...
            'position', [robot2_center(1), robot2_center(2), scale*cos(mpc_results.state(5, i)), scale*sin(mpc_results.state(5, i))], ...
            'Color', 'blue', 'Headlength', 7, 'Headwidth', 7, 'LineWidth', 1.5);
        arrow3_ac=annotation('arrow');
        set(arrow3_ac, 'parent', gca,...
            'position', [robot3_center(1), robot3_center(2), scale*cos(mpc_results.state(6, i)), scale*sin(mpc_results.state(6, i))], ...
            'Color', 'blue', 'Headlength', 7, 'Headwidth', 7, 'LineWidth', 1.5);
        arrow4_ac=annotation('arrow');
        set(arrow4_ac, 'parent', gca,...
            'position', [robot4_center(1), robot4_center(2), scale*cos(mpc_results.state(7, i)), scale*sin(mpc_results.state(7, i))], ...
            'Color', 'blue', 'Headlength', 7, 'Headwidth', 7, 'LineWidth', 1.5);

        % 3. Plotting reference trajectories and waypoints used for that loop
        temp=cell2mat(mpc_results.waypoints_ref(i));
        ref_wpts=plot(temp(1, :), temp(2, :), 'or-', Linewidth=1.0);
        posiB_plot=plot(posiB(1), posiB(2), 'or', 'MarkerSize', 6, 'MarkerFaceColor','r');
        temp=cell2mat(mpc_results.candidate_ref(i));
        ref_traj=plot(temp(1, :), temp(2, :), '-g', LineWidth=1.5);
        
        if selectMap==4  % Zoom in to the robot for big maps
            xlim([posiB(1)-15, posiB(1)+15])
            ylim([posiB(2)-15, posiB(2)+15])
        end

        title(sprintf('Animation of Motions (Tine: %.2f [s])', i*params.con.t_delta))
        drawnow;
        %ax=gca;
        %exportgraphics(ax, gifFile, 'Append', true);
        pause(0.002);
    end
    hold off
end

%% Figure: The Map and Trajectories
if showfig.MapAndTraj==true
    figure('Name', 'The Map and Trajectories')
    hold on
    show(map.signedDistMap, BoundaryColor=[0, 0, 0],Colorbar='on')
    plot(user.pose_init(1, 1), user.pose_init(2, 1),...
        'c', 'Marker', 'square', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'DisplayName', 'Initial Point')
    plot(user.pose_fnal(1, 1), user.pose_fnal(2, 1),...
        'c', 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'DisplayName', 'Final Point')
    plot(ref.x, ref.y,...
        '-c', 'LineWidth', 3, 'DisplayName', 'Voronoi Ref. Traj.')
    plot(mpc_results.state(1, :), mpc_results.state(2, :),...
        '-b', 'LineWidth', 1.5, 'DisplayName', 'Actual Traj.')
    title("The Map and Trajectories")
    legend()
    hold off
end

% Figure: States Against Time
if showfig.StatesAgainstTime==true
    figure('Name', 'States Against Time')

    subplot(3, 1, 1)
    hold on
    plot(time, mpc_results.used_ref(1, :), '-c', 'DisplayName', 'Reference x_B Posi.')
    plot(time, mpc_results.state(1, 2:end), '-b', 'DisplayName', 'Actual x_B Posi.')
    xlabel('Time [s]')
    ylabel('x_B [m]')
    title('x_B Positions')
    legend()

    subplot(3, 1, 2)
    hold on
    plot(time, mpc_results.used_ref(2, :), '-c', 'DisplayName', 'Reference y_B Posi.')
    plot(time, mpc_results.state(2, 2:end), '-b', 'DisplayName', 'Actual y_B Posi.')
    xlabel('Time [s]')
    ylabel('y_B [m]')
    title('y_B Positions')
    legend()

    subplot(3, 1, 3)
    hold on
    plot(time, rad2deg(mpc_results.used_ref(3, :)), '-c', 'DisplayName', 'Reference \theta_B.')
    plot(time, rem(rad2deg(mpc_results.state(3, 2:end))+180,360)-180 , '-b', 'DisplayName', 'Actual \theta_B')
    xlabel('Time [s]')
    ylabel('\theta_B [deg]')
    title('Orientation')
    legend()

    sgtitle('States Against Time')
end

%% Figure: PseudoInputs Against Time
if showfig.PseudoinputsAgainstTime==true
    figure('Name', 'Pseudoinputs Against Time')

    subplot(3, 1, 1)
    hold on
    plot(time, mpc_results.up(1, :), '-b', 'DisplayName', 'M Point x Posi.')
    xlabel('Time [s]')
    ylabel('x_M [m]')
    title('M Point x Position')
    legend()
    hold off

    subplot(3, 1, 2)
    hold on
    plot(time, mpc_results.up(2, :), '-b', 'DisplayName', 'M Point y Posi.')
    xlabel('Time [s]')
    ylabel('y_M [m]')
    title('M Point y Position')
    legend()
    hold off

    subplot(3, 1, 3)
    hold on
    plot(time, mpc_results.up(3, :), '-r', 'DisplayName', 'M Point Angular Vel.')
    xlabel('Time [s]')
    ylabel('\omega_M [rad/s]')
    title('Angular Velocity About M')
    legend()
    hold off

    sgtitle('Pseudoinputs Against Time')
end

%% Figure: Robot Yaw Rates
if showfig.YawRates==true
    figure('Name', 'Robot Yaw Rates')

    hold on
    for i=1:params.sys.n_rbt
        plot(time, mpc_results.up(3+i, :),...
            'LineStyle', '-', 'Color', [0, 0, 1-0.1*(i-1)], 'DisplayName', sprintf('\theta_R, Robot %d', i))
    end
    xlabel('Time [s]')
    ylabel('\theta_R [rad/s]')
    title('Robot Yaw Rates')
    legend()

    hold off
end

%% Robot Desired and Actual Headings
if figshow.Headings==true
    figure('Name', 'Robot Headings')
    hold on
    for i=1:params.sys.n_rbt
        plot(time, rad2deg(robo.desired_headings(i, :)),...
            'Color', [0, 1-0.2*(i-1), 0], 'DisplayName', sprintf('Desired Heading, Robot %d', i))
        plot(time, rad2deg(mpc_results.state(3+i, 1:end-1)),...
            'Color', [0, 0, 1-0.2*(i-1)], 'DisplayName', sprintf('Actual Heading, Robot %d', i))
    end
    xlabel('Time[sec]')
    ylabel('Headings[deg]')
    ylim([-200, 200])
    title('Robot Headings')
    legend()
    hold off
end

%% Figure: Actual Inputs Against Time
if showfig.InputsAgainstTime==true
    figure('Name', 'Inputs Against Time')
    hold on
    for i=1:params.sys.n_rbt
        plot(time, mpc_results.u(2*(i-1)+1, :),...
            'LineStyle', '-', 'Color', [0, 0, 1-0.1*(i-1)], 'DisplayName', sprintf('Left Motor Input, Robot %d', i))
        plot(time, mpc_results.u(2*(i-1)+2, :),...
            'LineStyle', '-', 'Color', [1-0.1*(i-1), 0, 0], 'DisplayName', sprintf('Right Motor Input, Robot %d', i))
    end
    xlabel('Time [s]')
    ylabel('Motor Speeds [rad/s]')
    title('Motor Inputs')
    legend('Location', 'southeast')
    hold off
end

%% Figure: Computation Time
if showfig.ComputationTime==true
    figure('Name', 'Computational Time')
    hold on

    avg=1000*(mean(timeReport.trajGen_inLoops)+mean(timeReport.contGen_inLoops));

    temp=[timeReport.contGen_inLoops; timeReport.trajGen_inLoops];
    h=bar(time, 1000*temp, 'stacked');
    set(h(2), 'DisplayName', 'Trajectory Generation')
    set(h(1), 'DisplayName', 'Npp Solution')
    yline(1000*params.con.t_delta,...
        '-r', 'LineWidth', 1.5, 'DisplayName', 'MPC Time Intervals')
    yline(1000*options.ipopt.max_cpu_time,...
        '--r', 'LineWidth', 1.5, 'DisplayName', 'Upper limit of CPU Time')
    yline(avg, '--b', 'LineWidth', 1.5,...
        'DisplayName', sprintf('Average Time %.2f [ms] (%.2f [Hz])', avg, 1000/avg))
    xlabel('Run Time [s]')
    ylabel('Computation Time [ms]')
    ylim([0, 1500*params.con.t_delta])
    title('Computational Time')
    legend()
    hold off
end

%% Figure: Time history of minimum signed distances

if showfig.MinSignedDistances==true
    figure("Name", "Time History of Minimum Signed Distance Values")
    hold on
    plot(time, mpc_results.minSignedDist, ...
        '-m', 'DisplayName', 'Min. Signed Distance Values')
    yline(0, 'Color', 'r', 'LineStyle', '--', 'DisplayName', 'Threshold Value')
    
    ylim([-0.1, Inf])
    
    xlabel("Time [s]")
    ylabel("Signed Distances [m]")
    title("Time History of Minimum Signed Distance Values")
    legend("Location", "southeast")
    hold off
end

%% Figure: Verification of motor control inputs
if showfig.Verification==true
    figure('Name', 'Motor Control Verification')

    subplot(3, 1, 1)
    hold on
    for i=1:params.sys.n_rbt
        plot(time, robo.from_up.pose(3*(i-1)+1, :),...
            'Color', [0.2*(i-1), 1-0.2*(i-1), 0], 'LineStyle', '--', 'DisplayName', sprintf('x Posi., from Pseudoinputs, Robot %d', i))
        plot(time, robo.from_u.pose(3*(i-1)+1, :),...
            'Color', [0, 0, 1-0.2*(i-1)], 'LineStyle', '-', 'DisplayName', sprintf('x Posi., from Actual Inputs, Robot %d', i))
        xlabel('Time [s]')
        ylabel('x_R [m]')
        title('Robot x Positions')
        legend('Location', 'northeastoutside')
    end

    subplot(3, 1, 2)
    hold on
    for i=1:params.sys.n_rbt
        plot(time, robo.from_up.pose(3*(i-1)+2, :),...
            'Color', [0.2*(i-1), 1-0.2*(i-1), 0], 'LineStyle', '--', 'DisplayName', sprintf('y Posi., from Pseudoinputs, Robot %d', i))
        plot(time, robo.from_u.pose(3*(i-1)+2, :),...
            'Color', [0, 0, 1-0.2*(i-1)], 'LineStyle', '-', 'DisplayName', sprintf('y Posi., from Actual Inputs, Robot %d', i))
        xlabel('Time [s]')
        ylabel('y_R [m]')
        title('Robot y Positions')
        legend('Location', 'northeastoutside')
    end

    subplot(3, 1, 3)
    hold on
    for i=1:params.sys.n_rbt
        plot(time, rem(rad2deg(robo.from_up.pose(3*(i-1)+3, :))+180, 360)-180,...
            'Color', [0.2*(i-1), 1-0.2*(i-1), 0], 'LineStyle', '--', 'DisplayName', sprintf('Orientation, from Pseudoinputs, Robot %d', i))
        plot(time, rem(rad2deg(robo.from_u.pose(3*(i-1)+3, :))+180, 360)-180,...
            'Color', [0, 0, 1-0.2*(i-1)], 'LineStyle', '-', 'DisplayName', sprintf('Orientation, from Actual Inputs, Robot %d', i))
        plot(time, rem(rad2deg(mpc_results.used_ref(3, :))+180, 360)-180, ':r', 'DisplayName', 'Reference Orientation for B')
        xlabel('Time [s]')
        ylabel('\theta_R [deg]')
        title('Robot Orientations')
        legend('Location', 'northeastoutside')
    end
sgtitle('Verification of Control Inputs')
end


