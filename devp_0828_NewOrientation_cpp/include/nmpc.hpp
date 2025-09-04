#ifndef NMPC_HPP
#define NMPC_HPP

#include <casadi/casadi.hpp>
#include <vector>
#include "params.hpp"
#include "map.hpp"

class NMPC {
public:
    NMPC(const params::SysParams& sys_params, const params::ControlParams& con_params, const Map& map);

    // Solves the NLP problem
    std::map<std::string, casadi::DM> solve(
        const std::vector<double>& current_state,
        const std::vector<double>& ref_states,
        const std::vector<double>& last_controls,
        bool is_warm_start,
        const std::map<std::string, casadi::DM>& prev_solution = {}
    );

private:
    void setup_nlp(const params::SysParams& sys_params, const params::ControlParams& con_params, const Map& map);

    // Helper functions for NLP formulation
    casadi::SX current_to_next(const casadi::SX& x, const casadi::SX& up, double t_delta);
    casadi::SX pseudo_to_actual_inputs(const casadi::SX& x, const casadi::SX& up, const params::SysParams& params);
    casadi::SX heading_cost(const casadi::SX& x, const casadi::SX& up, const params::SysParams& params);
    casadi::SX min_signed_distance(const casadi::SX& x, const casadi::Function& sdf, const params::SysParams& params);
    casadi::SX reshape_in_cols(const std::vector<casadi::SX>& matrices);

    casadi::Function nlp_solver_init_;
    casadi::Function nlp_solver_warm_;

    std::map<std::string, casadi::DM> args_;
    std::map<std::string, casadi::DM> res_;

    struct NlpBounds {
        std::vector<double> lbx;
        std::vector<double> ubx;
        std::vector<double> lbg;
        std::vector<double> ubg;
    } nlp_bounds_;

    struct NlpLengths {
        int x_col;
        int up_col;
        int u_col;
        int p_x_curr_col;
        int p_xb_desr_col;
        int p_u_last_col;
    } nlp_lengths_;
};

#endif // NMPC_HPP
