#include "map.hpp"
#include <opencv2/opencv.hpp>
#include <casadi/casadi.hpp>
#include <iostream>

// NOTE: This implementation assumes that OpenCV and CasADi are properly installed and linked.

Map::Map(const std::string& map_image_path, const params::MapParams& map_params) : map_params_(map_params) {
    loadMapFromFile(map_image_path);
    processMap(map_params);
    inflateMap(map_params);
    generateSDF(map_params);
    createSymbolicSDF(map_params);
}

const cv::Mat& Map::getOccupancyGrid() const {
    return occupancy_grid_;
}

const cv::Mat& Map::getInflatedOccupancyGrid() const {
    return inflated_occupancy_grid_;
}

const cv::Mat& Map::getSignedDistanceField() const {
    return signed_distance_field_;
}

casadi::Function Map::getSymbolicSignedDistanceField() const {
    return symbolic_sdf_;
}

void Map::loadMapFromFile(const std::string& map_image_path) {
    cv::Mat image = cv::imread(map_image_path, cv::IMREAD_UNCHANGED);

    if (image.empty()) {
        std::cerr << "Error: Could not read image from path: " << map_image_path << std::endl;
        return;
    }

    cv::Mat gray_image;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    } else {
        gray_image = image;
    }

    // Binarize the image
    cv::Mat binary_map;
    cv::threshold(gray_image, binary_map, 127, 255, cv::THRESH_BINARY);

    // Invert the map so that obstacles are 0 and free space is 255 (or 1 after conversion)
    // In MATLAB: map.logicalMat = ~map.logicalMat;
    cv::bitwise_not(binary_map, occupancy_grid_);

    // Convert to CV_8U
    occupancy_grid_.convertTo(occupancy_grid_, CV_8U);
}

void Map::processMap(const params::MapParams& map_params) {
    // The MATLAB code uses a binaryOccupancyMap object which handles coordinate transformations.
    // Here, we are storing the raw occupancy grid. The resolution and origin from map_params
    // will be used when converting between world and map coordinates.
    // For now, we just store the parameters.
}

void Map::inflateMap(const params::MapParams& map_params) {
    if (map_params.inflation > 0) {
        // The inflation in MATLAB is specified in world units. We need to convert it to pixels.
        int inflation_pixels = static_cast<int>(map_params.inflation * map_params.resolution);
        // The kernel for dilation should be odd and its size related to the inflation radius
        int kernel_size = 2 * inflation_pixels + 1;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::dilate(occupancy_grid_, inflated_occupancy_grid_, element);
    } else {
        inflated_occupancy_grid_ = occupancy_grid_.clone();
    }
}

void Map::generateSDF(const params::MapParams& map_params) {
    // The input to distanceTransform should be a binary image where obstacles are 0.
    cv::Mat inverted_map;
    cv::bitwise_not(occupancy_grid_, inverted_map);

    cv::Mat dist_transform;
    cv::distanceTransform(inverted_map, dist_transform, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // The Danielsson algorithm (used by distanceTransform) gives the distance to the nearest zero pixel.
    // We need to subtract the distance to the nearest non-zero pixel to get a signed distance field.
    cv::Mat dist_transform_inv;
    cv::distanceTransform(occupancy_grid_, dist_transform_inv, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    signed_distance_field_ = dist_transform - dist_transform_inv;

    // The MATLAB code subtracts an obstacle thickness. This is to account for the fact that
    // the distance is measured from the center of the pixel.
    double obst_thickness = sqrt(2.0) * 0.5 / map_params.resolution;
    signed_distance_field_ = signed_distance_field_ / map_params.resolution - obst_thickness;
}

void Map::createSymbolicSDF(const params::MapParams& map_params) {
    // Get world coordinates of the map boundaries
    double southwest_x = map_params.OriginInWorldCoord[0];
    double southwest_y = map_params.OriginInWorldCoord[1];
    double northeast_x = southwest_x + occupancy_grid_.cols / map_params.resolution;
    double northeast_y = southwest_y + occupancy_grid_.rows / map_params.resolution;

    // Create grid for interpolation
    std::vector<double> x_grid, y_grid;
    for (double x = southwest_x; x <= northeast_x; x += map_params.symbolicInterpolationInterval) {
        x_grid.push_back(x);
    }
    for (double y = southwest_y; y <= northeast_y; y += map_params.symbolicInterpolationInterval) {
        y_grid.push_back(y);
    }

    // Sample the SDF at the grid points
    std::vector<double> sdf_data;
    for (double y : y_grid) {
        for (double x : x_grid) {
            // Convert world coordinates to map coordinates
            int map_x = static_cast<int>((x - southwest_x) * map_params.resolution);
            int map_y = static_cast<int>((y - southwest_y) * map_params.resolution);

            // Clamp coordinates to be within the map boundaries
            map_x = std::max(0, std::min(occupancy_grid_.cols - 1, map_x));
            map_y = std::max(0, std::min(occupancy_grid_.rows - 1, map_y));

            sdf_data.push_back(signed_distance_field_.at<float>(map_y, map_x));
        }
    }

    // Create the CasADi interpolant
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");

    std::vector<double> x_grid_vec(x_grid.begin(), x_grid.end());
    std::vector<double> y_grid_vec(y_grid.begin(), y_grid.end());

    casadi::Dict opts;
    opts["grid"] = std::vector<std::vector<double>>{x_grid_vec, y_grid_vec};
    opts["values"] = sdf_data;
    opts["method"] = "linear";

    symbolic_sdf_ = casadi::interpolant("signedDistMap_symbolic", "linear", {x_grid, y_grid}, sdf_data);
}
