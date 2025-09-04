#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <vector>
#include "map.hpp"

// Represents a path as a vector of points (x, y)
using Path = std::vector<std::vector<double>>;

// Abstract base class for all planners
class Planner {
public:
    virtual ~Planner() = default;

    // Pure virtual function for planning a path.
    // Takes start and goal poses [x, y, theta] and the map.
    // Returns a path as a sequence of [x, y] waypoints.
    virtual Path plan(const std::vector<double>& start, const std::vector<double>& goal, const Map& map) = 0;
};

// PRM Planner
class PRM : public Planner {
public:
    PRM(int num_nodes, double connection_distance);
    Path plan(const std::vector<double>& start, const std::vector<double>& goal, const Map& map) override;

private:
    int num_nodes_;
    double connection_distance_;
};

// RRT Planner
class RRT : public Planner {
public:
    RRT();
    Path plan(const std::vector<double>& start, const std::vector<double>& goal, const Map& map) override;
};

// RRT* Planner
class RRTStar : public Planner {
public:
    RRTStar();
    Path plan(const std::vector<double>& start, const std::vector<double>& goal, const Map& map) override;
};

#endif // PLANNER_HPP
