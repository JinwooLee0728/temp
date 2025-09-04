#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <string>

#include "params.hpp"
#include "map.hpp"
#include "planner.hpp"
#include "nmpc.hpp"

// Function to check for MPC termination
bool mpcTermination(const std::vector<double>& state, const params::UserParams& user) {
    double dx = state[0] - user.pose_fnal[0];
    double dy = state[1] - user.pose_fnal[1];
    if (std::sqrt(dx * dx + dy * dy) < 0.5) {
        return true;
    }
    return false;
}

// Function to generate reference states from waypoints (simplified version)
std::vector<std::vector<double>> generateReferenceFromWaypoints(
    const Path& wpts, const params::UserParams& user, const params::ControlParams& con_params) {

    std::vector<double> x_ref, y_ref, theta_ref;

    if (wpts.empty()) {
        return {x_ref, y_ref, theta_ref};
    }

    // Simplified linear interpolation
    double total_length = 0;
    for (size_t i = 0; i < wpts.size() - 1; ++i) {
        double dx = wpts[i+1][0] - wpts[i][0];
        double dy = wpts[i+1][1] - wpts[i][1];
        total_length += std::sqrt(dx*dx + dy*dy);
    }

    double current_dist = 0;
    double target_dist = 0;
    size_t wpts_idx = 0;

    for (int i = 0; i < con_params.n_hor; ++i) {
        while (wpts_idx < wpts.size() - 1) {
            double seg_dx = wpts[wpts_idx+1][0] - wpts[wpts_idx][0];
            double seg_dy = wpts[wpts_idx+1][1] - wpts[wpts_idx][1];
            double seg_len = std::sqrt(seg_dx*seg_dx + seg_dy*seg_dy);

            if (current_dist + seg_len >= target_dist) {
                double ratio = (target_dist - current_dist) / seg_len;
                x_ref.push_back(wpts[wpts_idx][0] + ratio * seg_dx);
                y_ref.push_back(wpts[wpts_idx][1] + ratio * seg_dy);
                break;
            }
            current_dist += seg_len;
            wpts_idx++;
        }
        if (wpts_idx >= wpts.size() - 1) {
            // If we reach the end, just repeat the last point
             x_ref.push_back(wpts.back()[0]);
             y_ref.push_back(wpts.back()[1]);
        }
        target_dist += user.ref_spdd * con_params.t_delta;
    }

    // Use average tangent for orientation
    for (size_t i = 0; i < x_ref.size() - 1; ++i) {
        theta_ref.push_back(std::atan2(y_ref[i+1] - y_ref[i], x_ref[i+1] - x_ref[i]));
    }
    if (!theta_ref.empty()) {
        theta_ref.push_back(theta_ref.back());
    } else if (!wpts.empty()) {
        theta_ref.assign(x_ref.size(), std::atan2(wpts.back()[1] - wpts.front()[1], wpts.back()[0] - wpts.front()[0]));
    } else {
        theta_ref.assign(x_ref.size(), 0.0);
    }


    return {x_ref, y_ref, theta_ref};
}


int main(int argc, char* argv[]) {
    std::cout << "########### MULTIAGENT TRANSPORTATION PORJECT (C++ VERSION) ###########" << std::endl;

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_map_image>" << std::endl;
        return 1;
    }
    std::string map_file_path = argv[1];

    // 1. Select map and planner
    int selectMap = 4; // This is now mainly for selecting initial/final poses
    int selectPlanner = 3; // 1: RRT, 2: RRT*, 3: PRM

    // 2. Initialize parameters
    params::SysParams sys_params;
    params::UserParams user_params;
    params::MapParams map_params;
    params::ControlParams con_params;
    params::initialize_parameters(selectMap, sys_params, user_params, map_params, con_params);

    // 3. Create map object
    std::cout << "Generating map from " << map_file_path << "..." << std::endl;
    Map map(map_file_path, map_params);
    std::cout << "Map generated." << std::endl;

    // 4. Create planner object
    std::cout << "Creating planner..." << std::endl;
    std::unique_ptr<Planner> planner;
    if (selectPlanner == 1) {
        planner = std::make_unique<RRT>();
    } else if (selectPlanner == 2) {
        planner = std::make_unique<RRTStar>();
    } else {
        planner = std::make_unique<PRM>(700, 10.0);
    }
    std::cout << "Planner created." << std::endl;

    // 5. Create NMPC object
    std::cout << "Setting up NMPC..." << std::endl;
    NMPC nmpc(sys_params, con_params, map);
    std::cout << "NMPC setup complete." << std::endl;

    // 6. The MPC Loop
    std::cout << "Starting MPC loop..." << std::endl;

    std::vector<std::vector<double>> mpc_state_history;
    std::vector<double> current_state = {user_params.pose_init[0], user_params.pose_init[1], user_params.pose_init[2]};
    current_state.resize(3 + sys_params.n_rbt, user_params.pose_init[2]); // Add robot orientations
    mpc_state_history.push_back(current_state);

    std::vector<double> last_controls(2 * sys_params.n_rbt, 0.0);
    std::map<std::string, casadi::DM> prev_solution;

    int mpc_iter = 0;
    while (!mpcTermination(current_state, user_params)) {
        std::cout << "\n--- MPC Iteration: " << mpc_iter << " ---" << std::endl;
        std::cout << "Current pose: (" << current_state[0] << ", " << current_state[1] << ", " << current_state[2] << ")" << std::endl;

        // 1. Generate path
        Path wpts = planner->plan({current_state[0], current_state[1]}, {user_params.pose_fnal[0], user_params.pose_fnal[1]}, map);

        if (wpts.empty()) {
            std::cerr << "Planner failed to find a path. Aborting." << std::endl;
            break;
        }

        // 2. Generate reference trajectory
        auto ref_trajectory = generateReferenceFromWaypoints(wpts, user_params, con_params);
        std::vector<double> ref_states_flat;
        for(int i = 0; i < con_params.n_hor; ++i) {
            ref_states_flat.push_back(ref_trajectory[0][i]);
            ref_states_flat.push_back(ref_trajectory[1][i]);
            ref_states_flat.push_back(ref_trajectory[2][i]);
        }

        // 3. Solve NLP
        bool is_warm_start = (mpc_iter > 0);
        auto solution = nmpc.solve(current_state, ref_states_flat, last_controls, is_warm_start, prev_solution);
        prev_solution = solution;

        // 4. Extract results
        casadi::DM x_sol = solution.at("x");
        int x_len = 3 + sys_params.n_rbt;
        int up_len = 3 + sys_params.n_rbt;
        int u_len = 2 * sys_params.n_rbt;

        std::vector<double> next_state_vec = x_sol(casadi::Slice(x_len, 2*x_len)).get_elements();
        current_state = next_state_vec;

        int u_start_idx = x_len * (con_params.n_hor + 1) + up_len * con_params.n_hor;
        std::vector<double> current_controls_vec = x_sol(casadi::Slice(u_start_idx, u_start_idx + u_len)).get_elements();
        last_controls = current_controls_vec;

        mpc_state_history.push_back(current_state);

        mpc_iter++;
        if (mpc_iter >= 200) {
            std::cout << "Max iterations reached." << std::endl;
            break;
        }
    }

    std::cout << "\nMPC loop finished." << std::endl;
    std::cout << "Final pose: (" << current_state[0] << ", " << current_state[1] << ", " << current_state[2] << ")" << std::endl;

    return 0;
}
