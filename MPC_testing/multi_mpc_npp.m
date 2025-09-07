%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% File for NPP Solver Construction

tic;   % Start timer

fprintf("Building the NPP Solver...\n")

addpath('C:\Users\jinwo\OneDrive\바탕 화면\LARR\Program_Files\casadi-3.7.0-windows64-matlab2018b')
import casadi.*

N=params.con.n_hor;   % Number of points in MPC horizon
M=params.sys.n_rbt;   % Number of robots

% 1. Declaration of Design Variables
X=SX.sym('X', 3+M, N+1);   % Current and future state variables within time horizon
Up=SX.sym('Up', 3+M, N);   % Future pseudoinputs within time horizon
U=SX.sym('U', 2*M, N);     % Future motor control inputs within time horizon

% -------------------------------------------------------------------------
% Note on design variables
% X(:, i)=[xB; yB; thetaB; thetaR1; thetaR2; ... thetaRM]
% Up(:, i)=[xM; yM; omegaM; omegaR1; omegaR2; ... omegaRM]
% U(:, i)=[uL1; uR1; uL2; uR2; ... uLM; uRM]
% -------------------------------------------------------------------------

% 2. Declaration of Parameters
P_X_curr=SX.sym('P_X_curr', length(X(:, 1)), 1);   % Current state, including robot orientations
P_Xb_desr=SX.sym('P_X_desr', 3, N);                % Future reference states, excluding robot orientations
P_U_last=SX.sym('P_U_last', length(U(:, 1)), 1);   % Last control inputs used, used in cost for du/dt

% 3. Construction of the Cost Function
F_Cost=0;
for i=1:N
    % ---------------------------------------------------------------------
    % COST1: Tracking Error Cost

    % For Translational Position
    F_Cost=F_Cost+(X(1:2, i+1)-P_Xb_desr(1:2, i))'*params.con.Q_err_trn*(X(1:2, i+1)-P_Xb_desr(1:2, i));

    % For Orientation (Similar Method to the Heading Cost for Robots)
    temp=([cos(X(3, i+1)); sin(X(3, i+1))]-[cos(P_Xb_desr(3, i)); sin(P_Xb_desr(3, i))]);
    F_Cost=F_Cost+params.con.Q_err_ang*(temp')*(temp);
    % ---------------------------------------------------------------------

    % ---------------------------------------------------------------------
    % COST2: Robot Heading Alignment Cost
    F_Cost=F_Cost+params.con.Q_hdg*HeadingCost(X(:, i), Up(:, i), params);
    
    % Function "HeadingCost" compares the actual robot headings and robot
    % headings determined by the current state and pseudoinput.
    % Cost is minimized if the two coincide.
    % ---------------------------------------------------------------------

    % ---------------------------------------------------------------------
    % COST3: Control Effort Cost
    F_Cost=F_Cost+params.con.Q_con*U(:, 1)'*U(:, i);
    % ---------------------------------------------------------------------

    % ---------------------------------------------------------------------
    % COST4: Control Input Change Rate Cost (Cost for du/dt)
    if i==1
        F_Cost=F_Cost+params.con.Q_chg*((U(:, i)-P_U_last)/params.con.t_delta)'*((U(:, i)-P_U_last)/params.con.t_delta);
    else
        F_Cost=F_Cost+params.con.Q_chg*((U(:, i)-U(:, i-1))/params.con.t_delta)'*((U(:, i)-U(:, i-1))/params.con.t_delta);
    end
    % ---------------------------------------------------------------------
end

% 4. Construction of Constraints

% -------------------------------------------------------------------------
% CONSTRAINT1: Initial Conditions Constraints
G_Init=X(:, 1)-P_X_curr;
% -------------------------------------------------------------------------

G_Dyna=SX.zeros(3+M, N);
G_Cont=SX.zeros(2*M, N);
G_Obst=SX.zeros(1, N);
for i=1:N
    % ---------------------------------------------------------------------
    % CONSTRAINT2: Discretized Dynamics of the System
    G_Dyna(:, i)=Current2Next(X(:, i), Up(:, i), params.con.t_delta)-X(:, i+1);

    % Function "Current2Next" computes the state variables at [k+1] from the
    % state variables and pseudoinputs at [k].
    % ---------------------------------------------------------------------

    % ---------------------------------------------------------------------
    % CONSTRAINT3: Relationship btw Pseudoinputs and Motor Control Inputs
    G_Cont(:, i)=PseudoInputs2ActualInputs(X(:, i), Up(:, i), params)-U(:, i);
    
    % Function "PseudoInputs2ActualInputs" computes motor control inputs
    % from the state variables and pseudoinputs
    % ---------------------------------------------------------------------

    % ---------------------------------------------------------------------
    % CONSTRAINT4: Constraint for Obstacle Evasion
    G_Obst(:, i)=MinSignedDistance(X(:, i), map, params);

    % Function "MinSignedDistance" returns the minimum signed distance
    % values of the pixels that the cart-robot assemply is positioned on.
    % ---------------------------------------------------------------------
end

% 5. Building Structure "npp" for "nlpsol"
npp.x=ReshapeInCols({X, Up, U});                        % Design Variables
npp.p=ReshapeInCols({P_X_curr, P_Xb_desr, P_U_last});   % Parameters
npp.f=F_Cost;                                           % Objective Function
npp.g=ReshapeInCols({G_Init, G_Dyna, G_Cont, G_Obst});  % Constraint Functions

% 6. Setting up "nlpsol"
options.ipopt.max_iter=3000;
options.ipopt.print_level=3;
options.ipopt.print_timing_statistics='yes';
options.ipopt.tol=1e-6;   % Loosened tolerance; default 1e-8

% The NPP solver for the initial cold start
options.ipopt.warm_start_init_point='no';   % No warm start used for initial loop
npp_solver_init=nlpsol('npp_solver_init', 'ipopt', npp, options);

% The NPP solver for later warm starts
options.ipopt.warm_start_init_point='yes';   % Warm start for loops after the initial loop
options.ipopt.warm_start_bound_frac=1e-16;   % Adjustments to speed ipopt
options.ipopt.warm_start_bound_push=1e-16;
options.ipopt.warm_start_mult_bound_push=1e-16;
options.ipopt.warm_start_slack_bound_frac=1e-16;
options.ipopt.warm_start_slack_bound_push=1e-16;
options.ipopt.max_cpu_time=0.5*params.con.t_delta;   % Upper bound on maximum CPU time allowed
npp_solver_warm=nlpsol('npp_solver_warm', 'ipopt', npp, options);

% 7. Lengths of Column Arrays (for convenience)
lgth.X_col=length(X(1, :))*length(X(:, 1));
lgth.Up_col=length(Up(1, :))*length(Up(:, 1));
lgth.U_col=length(U(1, :))*length(U(:, 1));
lgth.P_X_curr_col=length(P_X_curr(1, :))*length(P_X_curr(:, 1));
lgth.P_Xb_desr_col=length(P_Xb_desr(1, :))*length(P_Xb_desr(:, 1));
lgth.P_U_last_col=length(P_U_last(1, :))*length(P_U_last(:, 1));
lgth.G_Init_col=length(G_Init(1, :))*length(G_Init(:, 1));
lgth.G_Dyna_col=length(G_Dyna(1, :))*length(G_Dyna(:, 1));
lgth.G_Cont_col=length(G_Cont(1, :))*length(G_Cont(:, 1));
lgth.G_Obst_col=length(G_Obst(1, :))*length(G_Obst(:, 1));

% 8. Construction of Argument Bounds
bnd=params.con.arg_bnd;   % Bounds imposed on variables for a closed set problem

lbx_arr_x=-bnd*ones(1, lgth.X_col);
ubx_arr_x=+bnd*ones(1, lgth.X_col);
lbx_arr_up=-bnd*ones(1, lgth.Up_col);
ubx_arr_up=+bnd*ones(1, lgth.Up_col);
lbx_arr_u=params.sys.u_lower*ones(1, lgth.U_col);
ubx_arr_u=params.sys.u_upper*ones(1, lgth.U_col);

lbx_arr=[lbx_arr_x, lbx_arr_up, lbx_arr_u];
ubx_arr=[ubx_arr_x, ubx_arr_up, ubx_arr_u];

% Construction of Function Bounds

% -------------------------------------------------------------------------
% Note on Function Bounds
% G_Init: Equality constraint, equal to zero
% G_Dyna: Equality constraint, equal to zero
% G_Cont: Equality constraint, equal to zero
% G_Obst: Inequaltty constraint, lower bound "params.con.obstacleBuffer", upper bound Inf
% -------------------------------------------------------------------------

lbg_arr_init=zeros(1, lgth.G_Init_col);
ubg_arr_init=zeros(1, lgth.G_Init_col);
lbg_arr_dyna=zeros(1, lgth.G_Dyna_col);
ubg_arr_dyna=zeros(1, lgth.G_Dyna_col);
lbg_arr_cont=zeros(1, lgth.G_Cont_col);
ubg_arr_cont=zeros(1, lgth.G_Cont_col);
lbg_arr_obst=+params.con.obstacleBuffer*ones(1, lgth.G_Obst_col);
ubg_arr_obst=+Inf*ones(1, lgth.G_Obst_col);

lbg_arr=[lbg_arr_init, lbg_arr_dyna, lbg_arr_cont, lbg_arr_obst];
ubg_arr=[ubg_arr_init, ubg_arr_dyna, ubg_arr_cont, ubg_arr_obst];

timeReport.nppSolverBuild=toc;   % End timer

fprintf("Building of the NPP Solver completed in %.4f seconds!", timeReport.nppSolverBuild)

clear bnd F_Cost G_Cont G_Dyna G_Init G_Obst i
clear lbg_arr_cont lbg_arr_dyna lbg_arr_init lbg_arr_obst
clear ubg_arr_cont ubg_arr_dyna ubg_arr_init ubg_arr_obst
clear lbx_arr_u lbx_arr_up lbx_arr_x
clear ubx_arr_u ubx_arr_up ubx_arr_x
clear P_U_last P_X_curr P_Xb_desr U Up X

%% Functions

% Discretized Dynamcis
function x_next=Current2Next(x, up, t_delta)
% Current2Next
% : Gives the state variables of the system at the next time instance based
%   on the current state variables and pseudoinputs
%
% Inputs
%   x : (3+M)x1 [xB; yB; thetaB; thetaR1; thetaR2; ... thetaRM] - current
%   system state variables (M = number of robots)
%   up : (3+M)x1 [xM; yM; omegaM; omegaR1; omegaR2; ... omegaRM] -
%   pseudoinputs
%   t_delta : time step between MPC sampling points
%
% Outputs
%   x_next : (3+M)x1 - next system state variables

posiB=[x(1:2); 0]; thetaB=x(3);
thetaR_arr=x(4:end);

posiM=[up(1:2); 0]; omegaM=[0; 0; up(3)];
omegaR_arr=up(4:end);

r_MtoB=posiB-posiM;
veloB=cross(omegaM, r_MtoB);

posiB_next=posiB+veloB*t_delta;
thetaB_next=thetaB+omegaM(3)*t_delta;
thetaR_arr_next=thetaR_arr+omegaR_arr*t_delta;

x_next=[posiB_next(1:2); thetaB_next; thetaR_arr_next];
end

% Relationship Between Pseudoinputs and Actual Inputs
function u=PseudoInputs2ActualInputs(x, up, params)
% PseudoInputs2ActualInputs
% : Computes actual motor control inputs from current state and
%   pseudoinputs
%
% Inputs
%   x : (3+M)x1 [xB; yB; thetaB; thetaR1; thetaR2; ... thetaRM] - current
%   system state variables (M = number of robots)
%   up : (3+M)x1 [xM; yM; omegaM; omegaR1; omegaR2; ... omegaRM] -
%   pseudoinputs
%   params: structure containing parameters
%
% Outputs
%   u : (2*M)x1 [uL1; uR1; uL2; uR2; ... ; uLM; uRM] - motor control inputs

posiB=[x(1:2); 0]; thetaB=x(3);
posiM=[up(1:2); 0]; omegaM=[0; 0; up(3)];

r_MtoB=posiB-posiM;
RotBtoW=...
    [cos(thetaB), -sin(thetaB), 0;...
    sin(thetaB), cos(thetaB), 0;...
    0, 0, 1];

u=[];
for i=1:params.sys.n_rbt
    r_MtoR=r_MtoB+RotBtoW*[params.sys.r_BtoR(:, i); 0];
    veloR=cross(omegaM, r_MtoR);

    omegaR=up(3+i);

    veloR_pseudo=cross(omegaM, r_MtoR);
    veloR_actual=norm(veloR_pseudo)*[cos(x(3+i)); sin(x(3+i)); 0];

    diff1=(veloR_actual(1)-veloR_pseudo(1))^2+(veloR_actual(2)-veloR_pseudo(2))^2;
    diff2=(veloR_actual(1)+veloR_pseudo(1))^2+(veloR_actual(2)+veloR_pseudo(2))^2;

    uL=sign(diff2-diff1)*(norm(veloR)-0.5*omegaR*params.sys.robo_dst)/params.sys.robo_rdi;
    uR=sign(diff2-diff1)*(norm(veloR)+0.5*omegaR*params.sys.robo_dst)/params.sys.robo_rdi;

    temp=u;
    u=[temp; uL; uR];
end

end

% The Heading Cost
function hd_diff=HeadingCost(x, up, params)
% HeadingCost
% : Compares headings from the state vector and the pseudoinput
%
% Inputs
%   x : (3+M)x1 [xB; yB; thetaB; thetaR1; thetaR2; ... thetaRM] - current
%   system state variables (M = number of robots)
%   up : (3+M)x1 [xM; yM; omegaM; omegaR1; omegaR2; ... omegaRM] -
%   pseudoinputs
%   params: structure containing parameters
%
% Outputs
%   hd_diff: 1x1 - Scalar representing the heading difference of all robots

posiB=[x(1:2); 0]; thetaB=x(3);
posiM=[up(1:2); 0]; omegaM=[0; 0; up(3)];

r_MtoB=posiB-posiM;
RotBtoW=...
    [cos(thetaB), -sin(thetaB), 0;...
    sin(thetaB), cos(thetaB), 0;...
    0, 0, 1];

hd_diff=0;
for i=1:params.sys.n_rbt
    r_MtoR=r_MtoB+RotBtoW*[params.sys.r_BtoR(:, i); 0];
    veloR_pseudo=cross(omegaM, r_MtoR);   % Velocity from pseudoinputs
    veloR_actual=norm(veloR_pseudo)*[cos(x(3+i)); sin(x(3+i)); 0];   % Velocity from state

    diff1=(veloR_actual(1)-veloR_pseudo(1))^2+(veloR_actual(2)-veloR_pseudo(2))^2;
    diff2=(veloR_actual(1)+veloR_pseudo(1))^2+(veloR_actual(2)+veloR_pseudo(2))^2;

    hd_diff=hd_diff+min(diff1, diff2);   % Squared distance!
end

end

% The Function for Finding the Minimum Signed Distance along the perimeter
function sd_val=MinSignedDistance(x, map, params)
% MinSignedDistance
% : Return the minimum signed distance value amongst pixels that the
% boundary of the cart-robots assembly is position on
%
% Inputs
%   x : (3+M)x1 [xB; yB; thetaB; thetaR1; thetaR2; ... thetaRM] - current
%   system state variables (M = number of robots)
%   map : structure containing map related informations
%
% Outputs
%   sd_val : 1x1 - minimum signed distance value

addpath('C:\Users\jinwo\OneDrive\바탕 화면\LARR\Program_Files\casadi-3.7.0-windows64-matlab2018b')
import casadi.*

% 1. The minimun Signed Distance Value from the perimeter of the rectangular cart
posiB=x(1:2); thetaB=x(3);
Rot_BtoW=[cos(thetaB), -sin(thetaB); sin(thetaB), cos(thetaB)];

rect_vert=SX.zeros(2, 4);   % Stores the 4 vertices of the rectangle
rect_vert(:, 1)=posiB+Rot_BtoW*[-params.sys.cart_wdt/2; -params.sys.cart_hgt/2];
rect_vert(:, 2)=posiB+Rot_BtoW*[+params.sys.cart_wdt/2; -params.sys.cart_hgt/2];
rect_vert(:, 3)=posiB+Rot_BtoW*[+params.sys.cart_wdt/2; +params.sys.cart_hgt/2];
rect_vert(:, 4)=posiB+Rot_BtoW*[-params.sys.cart_wdt/2; +params.sys.cart_hgt/2];

s=0:0.1:1;   % Sample 10 points from each side
signed_dist_val_cart=[];   % Store smallest signed distance value yet encountered while scanning cart perimeter
for i=1:4
    % Indexing for cyclic structure
    if i<4
        idx_srt=i;
        idx_end=i+1;
    else
        idx_srt=4;
        idx_end=1;
    end

    for j=1:length(s)
        pt=(1-s(j))*rect_vert(:, idx_srt)+s(j)*rect_vert(:, idx_end);   % Sampled Point
        temp2=map.signedDistMap_symbolic(pt);   % Signed distance value at sample point
        temp1=signed_dist_val_cart;
        
        signed_dist_val_cart=[temp1, temp2];   % Add value
    end
end

% 2. The minimum Signed Distance Values from Circular Robots
signed_dist_val_robo=SX.zeros(1, params.sys.n_rbt);
for i=1:params.sys.n_rbt
    posiR=posiB+Rot_BtoW*params.sys.r_BtoR(:, i);   % Robot Center Positions
    signed_dist_val_robo(i)=map.signedDistMap_symbolic(posiR)-params.sys.robo_sze;
    % Subtracted robot radius for minimum signed distance value along
    % circular perimeter
end

% 3. Combining Results and Finding the Minimum Value
signed_dist_val=[signed_dist_val_cart, signed_dist_val_robo];

sd_val=min(signed_dist_val);

end

% Reshaping function for "npp" objects
function results=ReshapeInCols(cell_with_mats)

results=[];
for i=1:length(cell_with_mats)
    mat=cell2mat(cell_with_mats(i));
    temp1=results;
    temp2=reshape(mat, length(mat(1, :))*length(mat(:, 1)), 1);   % reshape into column vectors
    results=[temp1; temp2];
end

end