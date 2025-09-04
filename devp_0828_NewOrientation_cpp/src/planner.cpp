#include "planner.hpp"
#include <iostream>
#include <random>
#include <queue>
#include <map>
#include <algorithm>

// Helper struct for A* search
struct AStarNode {
    int id;
    double g_cost; // cost from start
    double h_cost; // heuristic cost to goal
    double f_cost() const { return g_cost + h_cost; }
    int parent_id;

    bool operator>(const AStarNode& other) const {
        return f_cost() > other.f_cost();
    }
};

// Helper function for Euclidean distance
double euclidean_distance(const std::vector<double>& p1, const std::vector<double>& p2) {
    return std::sqrt(std::pow(p1[0] - p2[0], 2) + std::pow(p1[1] - p2[1], 2));
}

// --- PRM Implementation ---
PRM::PRM(int num_nodes, double connection_distance)
    : num_nodes_(num_nodes), connection_distance_(connection_distance) {
    std::cout << "PRM planner created with " << num_nodes << " nodes and "
              << connection_distance << " connection distance." << std::endl;
}

Path PRM::plan(const std::vector<double>& start, const std::vector<double>& goal, const Map& map) {
    std::cout << "--- PRM planning ---" << std::endl;

    const cv::Mat& grid = map.getInflatedOccupancyGrid();
    const auto& map_params = map.getMapParams();

    // 1. Node Sampling
    std::vector<std::vector<double>> nodes;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(0.0, grid.cols);
    std::uniform_real_distribution<> dis_y(0.0, grid.rows);

    while (nodes.size() < num_nodes_) {
        int px = static_cast<int>(dis_x(gen));
        int py = static_cast<int>(dis_y(gen));
        if (grid.at<uchar>(py, px) == 0) { // Check for collision (0 is free space)
             // Convert pixel to world coordinates
            double world_x = px / map_params.resolution + map_params.OriginInWorldCoord[0];
            double world_y = (grid.rows - py) / map_params.resolution + map_params.OriginInWorldCoord[1]; // y is flipped
            nodes.push_back({world_x, world_y});
        }
    }

    nodes.push_back(start);
    nodes.push_back(goal);
    int start_id = nodes.size() - 2;
    int goal_id = nodes.size() - 1;

    // 2. Graph Building
    std::vector<std::vector<std::pair<int, double>>> adj(nodes.size());
    for (int i = 0; i < nodes.size(); ++i) {
        for (int j = i + 1; j < nodes.size(); ++j) {
            double dist = euclidean_distance(nodes[i], nodes[j]);
            if (dist < connection_distance_) {
                // Collision check for the edge
                bool collision = false;
                int num_steps = static_cast<int>(dist / (1.0 / map_params.resolution));
                for (int k = 1; k < num_steps; ++k) {
                    double t = static_cast<double>(k) / num_steps;
                    double world_x = nodes[i][0] + t * (nodes[j][0] - nodes[i][0]);
                    double world_y = nodes[i][1] + t * (nodes[j][1] - nodes[i][1]);
                    // Convert world to pixel
                    int px = static_cast<int>((world_x - map_params.OriginInWorldCoord[0]) * map_params.resolution);
                    int py = grid.rows - static_cast<int>((world_y - map_params.OriginInWorldCoord[1]) * map_params.resolution);

                    if (px >= 0 && px < grid.cols && py >= 0 && py < grid.rows && grid.at<uchar>(py, px) != 0) {
                        collision = true;
                        break;
                    }
                }
                if (!collision) {
                    adj[i].push_back({j, dist});
                    adj[j].push_back({i, dist});
                }
            }
        }
    }

    // 3. A* Search
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::map<int, AStarNode> all_nodes;

    open_set.push({start_id, 0.0, euclidean_distance(nodes[start_id], nodes[goal_id]), -1});
    all_nodes[start_id] = {start_id, 0.0, euclidean_distance(nodes[start_id], nodes[goal_id]), -1};

    Path path;
    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        if (current.id == goal_id) {
            // Reconstruct path
            AStarNode node = current;
            while (node.id != -1) {
                path.push_back(nodes[node.id]);
                if (node.parent_id == -1) break;
                node = all_nodes[node.parent_id];
            }
            std::reverse(path.begin(), path.end());
            std::cout << "PRM planner found a path." << std::endl;
            return path;
        }

        for (const auto& neighbor : adj[current.id]) {
            int neighbor_id = neighbor.first;
            double new_g_cost = current.g_cost + neighbor.second;

            if (all_nodes.find(neighbor_id) == all_nodes.end() || new_g_cost < all_nodes[neighbor_id].g_cost) {
                double h_cost = euclidean_distance(nodes[neighbor_id], nodes[goal_id]);
                AStarNode new_node = {neighbor_id, new_g_cost, h_cost, current.id};
                all_nodes[neighbor_id] = new_node;
                open_set.push(new_node);
            }
        }
    }

    std::cout << "PRM planner could not find a path." << std::endl;
    return path; // Return empty path if no solution found
}


// --- RRT Implementation (Placeholder) ---
RRT::RRT() {
    std::cout << "RRT planner created." << std::endl;
}

Path RRT::plan(const std::vector<double>& start, const std::vector<double>& goal, const Map& map) {
    std::cout << "--- RRT planning ---" << std::endl;
    std::cout << "Note: This is a placeholder implementation." << std::endl;
    Path path;
    path.push_back({start[0], start[1]});
    path.push_back({goal[0], goal[1]});
    return path;
}


// --- RRT* Implementation (Placeholder) ---
RRTStar::RRTStar() {
    std::cout << "RRT* planner created." << std::endl;
}

Path RRTStar::plan(const std::vector<double>& start, const std::vector<double>& goal, const Map& map) {
    std::cout << "--- RRT* planning ---" << std::endl;
    std::cout << "Note: This is a placeholder implementation." << std::endl;
    Path path;
    path.push_back({start[0], start[1]});
    path.push_back({goal[0], goal[1]});
    return path;
}
