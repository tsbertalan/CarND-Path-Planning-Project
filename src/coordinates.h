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
#include "spline.h"
#include "utils.h"

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double>
get_frenet(
    double x, double y, double theta,
    const std::vector<double> &maps_x, const std::vector<double> &maps_y
);

// Transform from Cartesian x,y coordinates to Frenet s,d,yaw coordinates
std::vector<double>
get_frenet(
    double x, double y, double theta,
    const std::vector<double> &maps_x, const std::vector<double> &maps_y,
    const std::vector<double> &maps_dx, const std::vector<double> &maps_dy,
    const std::vector<double> &maps_s
);

// Transform from Frenet s,d coordinates to Cartesian x,y
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

// Define three simple pose types, so we can enforce correct frame usage.
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

// Wrap coordinate transformations in a class that keeps map data.
class CoordinateTransformer {
 private:

  // Lists of the map data.
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // For smoothing the map data, keep spline interpolants.
  spline::tk::spline spline_x, spline_y, spline_dx, spline_dy;

  // The car's reference position for transformations to/from the car frame.
  WorldPose car_reference;

 public:

  // Largest s value in the loop.
  double max_s;

  CoordinateTransformer();

  // Set reference point for transformations to/from car frame.
  void set_reference(WorldPose car);

  // Convert to x,y global frame.
  WorldPose to_world(CarPose from);

  WorldPose to_world(FrenetPose from);

  // Convert to car frame.
  CarPose to_car(WorldPose from);

  CarPose to_car(FrenetPose from);

  // Convert to Frenet global frame.
  FrenetPose to_frenet(CarPose from);

  FrenetPose to_frenet(WorldPose from);

  // Provide access to the map waypoints.
  std::vector<std::vector<double>> get_waypoints();

};

double get_world_dist(WorldPose a, WorldPose b);

#endif //PATH_PLANNING_COORDINATES_H
