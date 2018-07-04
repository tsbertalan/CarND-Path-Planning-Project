//
// Created by tsbertalan on 7/3/18.
//

#ifndef PATH_PLANNING_DECISION_MAKING_H
#define PATH_PLANNING_DECISION_MAKING_H

#include <vector>
#include <algorithm>

using namespace std;

#include "jmt.h"
#include "coordinates.h"
#include <math.h>

// https://github.com/jal278/plotpipe
#include "graph.h"

#include "trajectory.h"
#include "neighbor.h"


double measure_cost(Trajectory plan, vector<Neighbor> neighbors);
// Check various things, like the speed of the proposed trajectory,
// whether any collisions are predicted,
// what the acceleration and jerk are,
// and whether we drive off the road.

class Planner {
private:
    plot p1, p2, p3, p4;

public:
    CoordinateTransformer transform;
    double target_max_speed;
    int current_intended_lane;
    int plan_length;
    unsigned long min_reused_points;

    Planner(
            CoordinateTransformer &transform,
            double target_max_speed = 42 * (5280 / 1.) * (1. / 3.2808) * (1 / 3600.), // m/s, NOT mph
            int plan_length = 500,
            int min_reused_points = 4
    );

    Trajectory make_plan(
            WorldPose current,
            double current_speed,
            Trajectory leftover,
            vector<Neighbor> neighbors,
            const double dt = .02
    );

    void show_trajectory(Trajectory plan);

};


#endif //PATH_PLANNING_DECISION_MAKING_H
