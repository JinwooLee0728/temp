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

% 2.1. User Defined Initial Poses
user.pose_init=[2.5; -1.0; deg2rad(+90)];
user.robotOrientation_init=[deg2rad(-90); deg2rad(-90); deg2rad(-90); deg2rad(-90)];

user.pose_fnal=[2.5; 1.5; deg2rad(90)];

% 2.2. Map-Related Paramters
map.symbolicInterpolationInterval=0.1;   % Intervals for interpolation

% NOTE: Make sure map inflations are zero for maps of resolution 1
map.resolution=10;
map.inflation=0.1;
map.OriginInWorldCoord=[-4, -4];

% 2.3. MPC Gains and Misc.
params.con.t_delta=0.1;                         % Time stamp intervals for MPC
params.con.n_hor=10;                            % Number of (future) reference points within horizon
params.con.Q_con=1e-6/params.sys.n_rbt;         % Control effort weight
params.con.Q_err_trn=diag([100, 100]);          % Tracking error weight for translational positions
params.con.Q_err_ang=1000;                       % Tracking error weight for orientation
params.con.Q_hdg=3000;                          % Heading angle tracking cost
params.con.Q_chg=1e-1/params.sys.n_rbt;         % Control change cost     
params.con.arg_bnd=1e+3;                        % NPP argument bounds for a "closed" problem
params.con.obstacleBuffer=0.0;                  % Buffer distance, in [m] for obstacles

params.con.closeEnough=1.5;
