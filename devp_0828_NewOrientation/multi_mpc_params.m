%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% Paremeters and User Defined Values

%% 1. System Parameters

% 1.1. Cart Parameters
params.sys.cart_hgt=1.03;   % Cart height [m]
params.sys.cart_wdt=2.04;   % Cart width [m]

% 1.2. Robot Parameters
params.sys.robo_sze=0.30;    % Radius of modelled robot [m]
params.sys.robo_rdi=0.10;    % Robot wheel radius [m]
params.sys.robo_dst=0.50;    % Distance between robot wheels [m]

% 1.3. Cart and Robot Connection Points (with respect to cart frame)
params.sys.n_rbt=4;                             % Number of robots
params.sys.r_BtoR=zeros(2, params.sys.n_rbt);   % Connection points
params.sys.r_BtoR(:, 1)=[params.sys.cart_wdt/2; -0.30];
params.sys.r_BtoR(:, 2)=[params.sys.cart_wdt/2; +0.20];
params.sys.r_BtoR(:, 3)=[+0.20; -params.sys.cart_hgt/2];
params.sys.r_BtoR(:, 4)=[-0.60; -params.sys.cart_hgt/2];

% 1.4. Bounds Regarding Motor Feasibility
params.sys.u_lower=-20;   % [rad/s]
params.sys.u_upper=+20;   % [rad/s]


%% 2. Control Parameters

% 2.1. User Defined Initial and Final Pose, Reference Speeds
user.ref_spdd=0.5*params.sys.u_upper*params.sys.robo_rdi;

if selectMap==1       % Matlab simpleMap
    user.pose_init=[3; 1; deg2rad(0)];
    user.pose_fnal=[24; 3; deg2rad(90)];
elseif selectMap==2   % Matlab complexMap
    user.pose_init=[5; 2; deg2rad(0)];
    %user.pose_fnal=[37; 23; deg2rad(0)];
    user.pose_fnal=[20; 15; deg2rad(0)];
elseif selectMap==3   % Custom L-corridor
    user.pose_init=[2; 16; deg2rad(0)];
    user.pose_fnal=[16; 2; deg2rad(90)];
elseif selectMap==4   % IsaacSim hospital map

    % Starting Point 1
    user.pose_init=[-30; 12.5; deg2rad(0)];
    
    %user.pose_fnal=[26; 30; deg2rad(0)];
    %user.pose_fnal=[-37; 3; deg2rad(0)];
    %user.pose_fnal=[10; 0; deg2rad(0)];
    user.pose_fnal=[-20; 20; deg2rad(0)];
    %user.pose_fnal=[-37; 20; deg2rad(0)];

    % Starting Point 2
    %user.pose_init=[20; 34; deg2rad(0)];
    %user.pose_fnal=[16; 16; deg2rad(0)];
else
    disp("Invalid value of variable selectMap")
end

% 2.2. Map-Related Paramters
map.symbolicInterpolationInterval=0.1;   % Intervals for interpolation

% NOTE: Make sure map inflations are zero for maps of resolution 1
if selectMap==1       % Matlab simpleMap
    map.resolution=10;
    map.inflation=0.7;
    map.OriginInWorldCoord=[0, 0];
elseif selectMap==2   % Matlab complexMap
    map.resolution=10;
    map.inflation=0.7;
    map.OriginInWorldCoord=[0, 0];
elseif selectMap==3   % Custom L-corridor
    map.resolution=10;
    map.inflation=0.7;
    map.OriginInWorldCoord=[0, 0];
elseif selectMap==4   % IsaacSim hospital map
    map.resolution=20;
    map.inflation=0.7;
    map.OriginInWorldCoord=[-42.6250, -4.7250];
else
    disp("Invalid value of variable selectMap")
end

% 2.3. MPC Gains and Misc.
params.con.t_delta=0.1;                         % Time stamp intervals for MPC
params.con.n_hor=20;                            % Number of (future) reference points within horizon
params.con.Q_con=1e-6/params.sys.n_rbt;         % Control effort weight
params.con.Q_err_trn=diag([100, 100]);          % Tracking error weight for translational positions
params.con.Q_err_ang=1000;                       % Tracking error weight for orientation
params.con.Q_hdg=2000;                          % Heading angle tracking cost
params.con.Q_chg=1e-1/params.sys.n_rbt;         % Control change cost     
params.con.arg_bnd=1e+3;                        % NPP argument bounds for a "closed" problem
params.con.obstacleBuffer=0.05;                  % Buffer distance, in [m] for obstacles

params.con.prmFactor=0.9;

