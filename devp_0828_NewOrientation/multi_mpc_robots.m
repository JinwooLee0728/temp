%% ############ Multiagent Transportation Project: 2025.08.13 #############
%% Computation of Robot Related Quantities

% 1. Robot Headings
robo.desired_headings=zeros(params.sys.n_rbt, length(mpc_results.up(1, :)));
for i=1:length(mpc_results.up(1, :))
    posiB=[mpc_results.state(1:2, i); 0]; thetaB=mpc_results.state(3, i);
    posiM=[mpc_results.up(1:2, i); 0]; omegaM=[0; 0; mpc_results.up(3, i)];

    r_MtoB=posiB-posiM;
    Rot_BtoW=...
        [cos(thetaB), -sin(thetaB), 0;...
        sin(thetaB), cos(thetaB), 0;...
        0, 0, 1];
    for j=1:params.sys.n_rbt
        r_MtoR=r_MtoB+Rot_BtoW*[params.sys.r_BtoR(:, j); 0];
        veloR=cross(omegaM, r_MtoR);
        thetaR_desired=atan2(veloR(2), veloR(1));

        robo.desired_headings(j, i)=thetaR_desired;
    end
end

n_pts=length(mpc_results.up(1, :));

% 2. Robot Poses and Velocities from Pseudoinputs
robo.from_up.pose=zeros(3*params.sys.n_rbt, n_pts);
robo.from_up.velo=zeros(3*params.sys.n_rbt, n_pts);
for i=1:n_pts
    posiB=[mpc_results.state(1:2, i); 0]; thetaB=mpc_results.state(3, i);
    posiM=[mpc_results.up(1:2, i); 0]; omegaM=[0; 0; mpc_results.up(3, i)];

    r_MtoB=posiB-posiM;
    Rot_BtoW=...
        [cos(thetaB), -sin(thetaB), 0;...
        sin(thetaB), cos(thetaB), 0;...
        0, 0, 1];
    for j=1:params.sys.n_rbt
        r_BtoR_inB=[params.sys.r_BtoR(:, j); 0];
        posiR=posiB+Rot_BtoW*r_BtoR_inB;

        r_MtoR=posiR-posiM;
        veloR=cross(omegaM, r_MtoR);
        omegaR=omegaM;
        thetaR=atan2(veloR(2), veloR(1));

        robo.from_up.pose(3*(j-1)+1:3*j, i)=[posiR(1:2); thetaR];
        robo.from_up.velo(3*(j-1)+1:3*j, i)=[veloR(1:2); omegaR(3)];
    end
end

% 3. Robot Poses and Velocities from Control Inputs
robo.from_u.pose=zeros(3*params.sys.n_rbt, n_pts);
robo.from_u.velo=zeros(3*params.sys.n_rbt, n_pts);

% Setting up initial conditions from reference values
posiB_ref=[user.pose_init(1); user.pose_init(2); 0];
thetaB_ref=user.pose_init(3);
thetaR=0;
%thetaR=atan2(ref.y(2)-ref.y(1), ref.x(2)-ref.x(1));

Rot_BtoW=...
    [cos(thetaB_ref), -sin(thetaB_ref), 0;...
    sin(thetaB_ref), cos(thetaB_ref), 0;...
    0, 0, 1];
for j=1:params.sys.n_rbt
    posiR=posiB_ref+Rot_BtoW*[params.sys.r_BtoR(:, j); 0];
    robo.from_u.pose(3*(j-1)+1:3*j, 1) = [posiR(1:2); thetaR];
end

% Numerical Integration
for i=1:n_pts
    for j=1:params.sys.n_rbt
        uL=mpc_results.u(2*(j-1)+1, i);
        uR=mpc_results.u(2*j, i);

        speedR=0.5*params.sys.robo_rdi*(uL+uR);
        omegaR=(params.sys.robo_rdi/params.sys.robo_dst)*(-uL+uR);
        
        thetaR=robo.from_u.pose(3*j, i);
        veloR=speedR*[cos(thetaR); sin(thetaR)];

        robo.from_u.velo(3*(j-1)+1:3*j, i)=[veloR; omegaR];

        if i<n_pts
            posiR=robo.from_u.pose(3*(j-1)+1:3*(j-1)+2, i);
            posiR_next=posiR+veloR*params.con.t_delta;
            thetaR_next=thetaR+omegaR*params.con.t_delta;

            robo.from_u.pose(3*(j-1)+1:3*j, i+1)=[posiR_next; thetaR_next];
        end
    end
end
        
