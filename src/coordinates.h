//
// Created by tsbertalan on 7/3/18.
//

#ifndef PATH_PLANNING_COORDINATES_H
#define PATH_PLANNING_COORDINATES_H

#include "Eigen-3.3/Eigen/Core"
#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <algorithm>

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double>
get_frenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<double>
get_frenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y,
           const std::vector<double> &maps_dx, const std::vector<double> &maps_dy);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> get_xy(
        double s, double d,
        const std::vector<double> &maps_s,
        const std::vector<double> &maps_x,
        const std::vector<double> &maps_y
);

// Alternate
std::vector<double> get_world(
        double s, double d, double yaw,
        const std::vector<double> &maps_s,
        const std::vector<double> &maps_x,
        const std::vector<double> &maps_y
);


int closest_waypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

int
next_waypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);


double distance(double x1, double y1, double x2, double y2);


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x);

double rad2deg(double x);


struct WorldPose {
    double x;
    double y;
    double yaw;
};


struct CarPose {
    double x;
    double y;
    double yaw;
};


struct FrenetPose {
    double s;
    double d;
    double yaw;
};


class CoordinateTransformer {
private:
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
    WorldPose car_reference;

public:
    CoordinateTransformer();

    void set_reference(WorldPose car);

    WorldPose to_world(CarPose from);

    WorldPose to_world(FrenetPose from);

    CarPose to_car(WorldPose from);

    CarPose to_car(FrenetPose from);

    FrenetPose to_frenet(CarPose from);

    FrenetPose to_frenet(WorldPose from);

};

double get_world_dist(WorldPose a, WorldPose b);

#endif //PATH_PLANNING_COORDINATES_H
