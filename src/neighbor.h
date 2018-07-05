//
// Created by tsbertalan on 7/4/18.
//

#ifndef PATH_PLANNING_NEIGHBOR_H
#define PATH_PLANNING_NEIGHBOR_H

#include "coordinates.h"

class Neighbor {
public:
    int id;
    WorldPose current;
    double vx, vy;

    Neighbor(int id, const WorldPose &current, double vx, double vy);

    Neighbor(int id, double x, double y, double vx, double vy);

    WorldPose future_position(double dt, CoordinateTransformer &transform);
};

#endif //PATH_PLANNING_NEIGHBOR_H
