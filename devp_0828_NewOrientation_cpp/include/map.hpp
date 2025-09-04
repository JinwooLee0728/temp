#ifndef MAP_HPP
#define MAP_HPP

#include <string>
#include <vector>
#include "params.hpp"

// Forward declarations for library types to avoid including heavy headers in the header file.
// This assumes the user will link against OpenCV and CasADi.
namespace cv {
    class Mat;
}

namespace casadi {
    class Function;
}

class Map {
public:
    // Constructor that takes the path to the map image and map parameters.
    // It will perform all the map processing steps.
    Map(const std::string& map_image_path, const params::MapParams& map_params);

    // Public getters to access the processed map data
    const cv::Mat& getOccupancyGrid() const;
    const cv::Mat& getInflatedOccupancyGrid() const;
    const cv::Mat& getSignedDistanceField() const;
    casadi::Function getSymbolicSignedDistanceField() const;

private:
    // Private methods to perform the different map processing steps
    void loadMapFromFile(const std::string& map_image_path);
    void processMap(const params::MapParams& map_params);
    void inflateMap(const params::MapParams& map_params);
    void generateSDF(const params::MapParams& map_params);
    void createSymbolicSDF(const params::MapParams& map_params);

    // Member variables to store the map data
    cv::Mat occupancy_grid_;
    cv::Mat inflated_occupancy_grid_;
    cv::Mat signed_distance_field_;
    casadi::Function symbolic_sdf_;

    params::MapParams map_params_;
};

#endif // MAP_HPP
