//
// Created by tsbertalan on 7/4/18.
//

#ifndef PATH_PLANNING_NEIGHBOR_H
#define PATH_PLANNING_NEIGHBOR_H

#include "coordinates.h"

class Neighbor {
 private:
  CoordinateTransformer &transform;
  double vs, vd;

 public:
  double vx, vy;
  int id;
  WorldPose current_wp;
  FrenetPose current_fp;

  Neighbor(CoordinateTransformer &transform, int id, const WorldPose &current, double vx, double vy);

  Neighbor(CoordinateTransformer &transform, int id, double x, double y, double vx, double vy);

  WorldPose future_position(double dt);

  FrenetPose future_position_frenet(double dt);

  double speed();
};

#endif //PATH_PLANNING_NEIGHBOR_H
