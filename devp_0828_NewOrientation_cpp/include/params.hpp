#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <vector>
#include <cmath>
#include <map>
#include <string>

namespace params {

// Helper function to convert degrees to radians
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// System Parameters
struct SysParams {
    double cart_hgt = 1.03;
    double cart_wdt = 2.04;
    double robo_sze = 0.30;
    double robo_rdi = 0.10;
    double robo_dst = 0.50;
    int n_rbt = 4;
    std::vector<std::vector<double>> r_BtoR;
    double u_lower = -20.0;
    double u_upper = 20.0;

    SysParams() {
        r_BtoR = {
            {cart_wdt / 2, cart_wdt / 2, 0.20, -0.60},
            {-0.30, 0.20, -cart_hgt / 2, -cart_hgt / 2}
        };
    }
};

// User-defined parameters
struct UserParams {
    double ref_spdd;
    std::vector<double> pose_init;
    std::vector<double> pose_fnal;
};

// Map-related parameters
struct MapParams {
    double symbolicInterpolationInterval = 0.1;
    int resolution;
    double inflation;
    std::vector<double> OriginInWorldCoord;
};

// MPC Gains and Miscellaneous
struct ControlParams {
    double t_delta = 0.1;
    int n_hor = 20;
    double Q_con;
    std::vector<std::vector<double>> Q_err_trn;
    double Q_err_ang = 1000;
    double Q_hdg = 2000;
    double Q_chg;
    double arg_bnd = 1e+3;
    double obstacleBuffer = 0.05;
    double prmFactor = 0.9;
};

// Function to initialize parameters based on map selection
void initialize_parameters(int selectMap, SysParams& sys, UserParams& user, MapParams& map, ControlParams& control) {
    // Initialize parameters that depend on other parameters
    user.ref_spdd = 0.5 * sys.u_upper * sys.robo_rdi;
    control.Q_con = 1e-6 / sys.n_rbt;
    control.Q_chg = 1e-1 / sys.n_rbt;
    control.Q_err_trn = {{100, 0}, {0, 100}};

    if (selectMap == 1) { // Matlab simpleMap
        user.pose_init = {3, 1, deg2rad(0)};
        user.pose_fnal = {24, 3, deg2rad(90)};
        map.resolution = 10;
        map.inflation = 0.7;
        map.OriginInWorldCoord = {0, 0};
    } else if (selectMap == 2) { // Matlab complexMap
        user.pose_init = {5, 2, deg2rad(0)};
        user.pose_fnal = {20, 15, deg2rad(0)};
        map.resolution = 10;
        map.inflation = 0.7;
        map.OriginInWorldCoord = {0, 0};
    } else if (selectMap == 3) { // Custom L-corridor
        user.pose_init = {2, 16, deg2rad(0)};
        user.pose_fnal = {16, 2, deg2rad(90)};
        map.resolution = 10;
        map.inflation = 0.7;
        map.OriginInWorldCoord = {0, 0};
    } else if (selectMap == 4) { // IsaacSim hospital map
        user.pose_init = {-30, 12.5, deg2rad(0)};
        user.pose_fnal = {-20, 20, deg2rad(0)};
        map.resolution = 20;
        map.inflation = 0.7;
        map.OriginInWorldCoord = {-42.6250, -4.7250};
    }
}

} // namespace params

#endif // PARAMS_HPP
