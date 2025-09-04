#include "nmpc.hpp"
#include <iostream>

NMPC::NMPC(const params::SysParams& sys_params, const params::ControlParams& con_params, const Map& map) {
    setup_nlp(sys_params, con_params, map);
}

void NMPC::setup_nlp(const params::SysParams& sys_params, const params::ControlParams& con_params, const Map& map) {
    using namespace casadi;

    int N = con_params.n_hor;
    int M = sys_params.n_rbt;

    // 1. Declare design variables
    SX X = SX::sym("X", 3 + M, N + 1);
    SX Up = SX::sym("Up", 3 + M, N);
    SX U = SX::sym("U", 2 * M, N);

    // 2. Declare parameters
    SX P_X_curr = SX::sym("P_X_curr", 3 + M, 1);
    SX P_Xb_desr = SX::sym("P_Xb_desr", 3, N);
    SX P_U_last = SX::sym("P_U_last", 2 * M, 1);

    // 3. Construct the cost function
    SX F_Cost = 0;
    for (int i = 0; i < N; ++i) {
        // Tracking Error Cost
        SX err_pos = X(Slice(0, 2), i + 1) - P_Xb_desr(Slice(0, 2), i);
        F_Cost += mtimes(err_pos.T(), mtimes(SX(con_params.Q_err_trn), err_pos));

        SX temp = SX::vertcat({cos(X(2, i + 1)), sin(X(2, i + 1))}) - SX::vertcat({cos(P_Xb_desr(2, i)), sin(P_Xb_desr(2, i))});
        F_Cost += con_params.Q_err_ang * mtimes(temp.T(), temp);

        // Robot Heading Alignment Cost
        F_Cost += con_params.Q_hdg * heading_cost(X(Slice(), i), Up(Slice(), i), sys_params);

        // Control Effort Cost
        F_Cost += con_params.Q_con * mtimes(U(Slice(), i).T(), U(Slice(), i));

        // Control Input Change Rate Cost
        SX dU;
        if (i == 0) {
            dU = (U(Slice(), i) - P_U_last) / con_params.t_delta;
        } else {
            dU = (U(Slice(), i) - U(Slice(), i - 1)) / con_params.t_delta;
        }
        F_Cost += con_params.Q_chg * mtimes(dU.T(), dU);
    }

    // 4. Construct constraints
    std::vector<SX> G;
    G.push_back(X(Slice(), 0) - P_X_curr);

    for (int i = 0; i < N; ++i) {
        G.push_back(current_to_next(X(Slice(), i), Up(Slice(), i), con_params.t_delta) - X(Slice(), i + 1));
        G.push_back(pseudo_to_actual_inputs(X(Slice(), i), Up(Slice(), i), sys_params) - U(Slice(), i));
        G.push_back(min_signed_distance(X(Slice(), i), map.getSymbolicSignedDistanceField(), sys_params));
    }

    // 5. Build NLP
    SX x_nlp = vertcat(SXVector{reshape(X, (3+M)*(N+1), 1), reshape(Up, (3+M)*N, 1), reshape(U, 2*M*N, 1)});
    SX g_nlp = vertcat(G);
    SX p_nlp = vertcat(SXVector{P_X_curr, reshape(P_Xb_desr, 3*N, 1), P_U_last});

    SXDict nlp = {{"x", x_nlp}, {"f", F_Cost}, {"g", g_nlp}, {"p", p_nlp}};

    // 6. Set up solver
    Dict opts;
    opts["ipopt.max_iter"] = 3000;
    opts["ipopt.print_level"] = 3;
    opts["ipopt.tol"] = 1e-6;

    opts["ipopt.warm_start_init_point"] = "no";
    nlp_solver_init_ = nlpsol("nlp_solver_init", "ipopt", nlp, opts);

    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.max_cpu_time"] = 0.5 * con_params.t_delta;
    nlp_solver_warm_ = nlpsol("nlp_solver_warm", "ipopt", nlp, opts);

    // 7. Store lengths
    nlp_lengths_.x_col = (3 + M) * (N + 1);
    nlp_lengths_.up_col = (3 + M) * N;
    nlp_lengths_.u_col = 2 * M * N;
    nlp_lengths_.p_x_curr_col = 3 + M;
    nlp_lengths_.p_xb_desr_col = 3 * N;
    nlp_lengths_.p_u_last_col = 2 * M;

    // 8. Construct argument bounds
    double bnd = con_params.arg_bnd;
    nlp_bounds_.lbx.resize(x_nlp.size1(), -bnd);
    nlp_bounds_.ubx.resize(x_nlp.size1(), bnd);

    for(int i = 0; i < nlp_lengths_.u_col; ++i) {
        nlp_bounds_.lbx[nlp_lengths_.x_col + nlp_lengths_.up_col + i] = sys_params.u_lower;
        nlp_bounds_.ubx[nlp_lengths_.x_col + nlp_lengths_.up_col + i] = sys_params.u_upper;
    }

    nlp_bounds_.lbg.assign(g_nlp.size1(), 0);
    nlp_bounds_.ubg.assign(g_nlp.size1(), 0);

    // Obstacle constraints are inequality constraints
    int g_start_idx = (3 + M) + (3 + M + 2 * M) * N;
    for (int i = 0; i < N; ++i) {
        nlp_bounds_.lbg[g_start_idx + i] = con_params.obstacleBuffer;
        nlp_bounds_.ubg[g_start_idx + i] = casadi::inf;
    }
}

std::map<std::string, casadi::DM> NMPC::solve(
    const std::vector<double>& current_state,
    const std::vector<double>& ref_states,
    const std::vector<double>& last_controls,
    bool is_warm_start,
    const std::map<std::string, casadi::DM>& prev_solution
) {
    args_["p"] = vertcat(DMVector{DM(current_state), DM(ref_states), DM(last_controls)});
    args_["lbx"] = nlp_bounds_.lbx;
    args_["ubx"] = nlp_bounds_.ubx;
    args_["lbg"] = nlp_bounds_.lbg;
    args_["ubg"] = nlp_bounds_.ubg;

    if (is_warm_start) {
        args_["x0"] = prev_solution.at("x");
        args_["lam_x0"] = prev_solution.at("lam_x");
        args_["lam_g0"] = prev_solution.at("lam_g");
        res_ = nlp_solver_warm_(args_);
    } else {
        // Set initial guess for x0 if not warm starting
        // For simplicity, we are not setting a smart initial guess here.
        // In a real application, this should be done.
        args_["x0"] = DM::zeros(nlp_lengths_.x_col + nlp_lengths_.up_col + nlp_lengths_.u_col, 1);
        res_ = nlp_solver_init_(args_);
    }

    return res_;
}


casadi::SX NMPC::current_to_next(const casadi::SX& x, const casadi::SX& up, double t_delta) {
    casadi::SX posiB = casadi::SX::vertcat({x(Slice(0, 2)), 0});
    casadi::SX thetaB = x(2);
    casadi::SX thetaR_arr = x(Slice(3, x.size1()));
    casadi::SX posiM = casadi::SX::vertcat({up(Slice(0,2)), 0});
    casadi::SX omegaM = casadi::SX::vertcat({0, 0, up(2)});
    casadi::SX omegaR_arr = up(Slice(3, up.size1()));

    casadi::SX r_MtoB = posiB - posiM;
    casadi::SX veloB = cross(omegaM, r_MtoB);

    casadi::SX posiB_next = posiB + veloB * t_delta;
    casadi::SX thetaB_next = thetaB + omegaM(2) * t_delta;
    casadi::SX thetaR_arr_next = thetaR_arr + omegaR_arr * t_delta;

    return casadi::SX::vertcat({posiB_next(Slice(0,2)), thetaB_next, thetaR_arr_next});
}

casadi::SX NMPC::pseudo_to_actual_inputs(const casadi::SX& x, const casadi::SX& up, const params::SysParams& params) {
    using namespace casadi;
    SX posiB = SX::vertcat({x(Slice(0, 2)), 0});
    SX thetaB = x(2);
    SX posiM = SX::vertcat({up(Slice(0,2)), 0});
    SX omegaM = SX::vertcat({0, 0, up(2)});

    SX r_MtoB = posiB - posiM;
    SX RotBtoW = SX::vertcat({
        SX::horzcat({cos(thetaB), -sin(thetaB), 0}),
        SX::horzcat({sin(thetaB), cos(thetaB), 0}),
        SX::horzcat({0, 0, 1})
    });

    SX u_vec;
    for (int i = 0; i < params.n_rbt; ++i) {
        SX r_BtoR_i = SX::vertcat({SX(params.r_BtoR[0][i]), SX(params.r_BtoR[1][i]), 0});
        SX r_MtoR = r_MtoB + mtimes(RotBtoW, r_BtoR_i);
        SX veloR = cross(omegaM, r_MtoR);
        SX omegaR = up(3 + i);

        SX uL = (norm_2(veloR) - 0.5 * omegaR * params.robo_dst) / params.robo_rdi;
        SX uR = (norm_2(veloR) + 0.5 * omegaR * params.robo_dst) / params.robo_rdi;
        u_vec = SX::vertcat({u_vec, uL, uR});
    }
    return u_vec;
}

casadi::SX NMPC::heading_cost(const casadi::SX& x, const casadi::SX& up, const params::SysParams& params) {
    using namespace casadi;
    SX posiB = SX::vertcat({x(Slice(0, 2)), 0});
    SX thetaB = x(2);
    SX posiM = SX::vertcat({up(Slice(0,2)), 0});
    SX omegaM = SX::vertcat({0, 0, up(2)});

    SX r_MtoB = posiB - posiM;
    SX RotBtoW = SX::vertcat({
        SX::horzcat({cos(thetaB), -sin(thetaB), 0}),
        SX::horzcat({sin(thetaB), cos(thetaB), 0}),
        SX::horzcat({0, 0, 1})
    });

    SX hd_diff = 0;
    for (int i = 0; i < params.n_rbt; ++i) {
        SX r_BtoR_i = SX::vertcat({SX(params.r_BtoR[0][i]), SX(params.r_BtoR[1][i]), 0});
        SX r_MtoR = r_MtoB + mtimes(RotBtoW, r_BtoR_i);
        SX veloR_pseudo = cross(omegaM, r_MtoR);
        SX veloR_actual = norm_2(veloR_pseudo) * SX::vertcat({cos(x(3+i)), sin(x(3+i)), 0});
        hd_diff += pow(veloR_actual(0) - veloR_pseudo(0), 2) + pow(veloR_actual(1) - veloR_pseudo(1), 2);
    }
    return hd_diff;
}

casadi::SX NMPC::min_signed_distance(const casadi::SX& x, const casadi::Function& sdf, const params::SysParams& params) {
    using namespace casadi;
    SX posiB = x(Slice(0, 2));
    SX thetaB = x(2);
    SX Rot_BtoW = SX::vertcat({
        SX::horzcat({cos(thetaB), -sin(thetaB)}),
        SX::horzcat({sin(thetaB), cos(thetaB)})
    });

    // Cart vertices
    SXVector rect_vert(4);
    rect_vert[0] = posiB + mtimes(Rot_BtoW, SX::vertcat({-params.cart_wdt/2, -params.cart_hgt/2}));
    rect_vert[1] = posiB + mtimes(Rot_BtoW, SX::vertcat({+params.cart_wdt/2, -params.cart_hgt/2}));
    rect_vert[2] = posiB + mtimes(Rot_BtoW, SX::vertcat({+params.cart_wdt/2, +params.cart_hgt/2}));
    rect_vert[3] = posiB + mtimes(Rot_BtoW, SX::vertcat({-params.cart_wdt/2, +params.cart_hgt/2}));

    SXVector signed_dist_val_cart;
    for (int i = 0; i < 4; ++i) {
        SX p1 = rect_vert[i];
        SX p2 = rect_vert[(i + 1) % 4];
        for (double s = 0.0; s <= 1.0; s += 0.1) {
            SX pt = (1 - s) * p1 + s * p2;
            signed_dist_val_cart.push_back(sdf(pt)[0]);
        }
    }

    SXVector signed_dist_val_robo;
    for (int i = 0; i < params.n_rbt; ++i) {
        SX r_BtoR_i = SX::vertcat({params.r_BtoR[0][i], params.r_BtoR[1][i]});
        SX posiR = posiB + mtimes(Rot_BtoW, r_BtoR_i);
        signed_dist_val_robo.push_back(sdf(posiR)[0] - params.robo_sze);
    }

    SX all_dists = SX::vertcat({SX::vertcat(signed_dist_val_cart), SX::vertcat(signed_dist_val_robo)});
    return min(all_dists);
}
