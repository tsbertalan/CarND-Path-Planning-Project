//
// Created by tsbertalan on 7/3/18.
//

#ifndef PATH_PLANNING_DECISION_MAKING_H
#define PATH_PLANNING_DECISION_MAKING_H

#include <vector>
#include <algorithm>
#include <math.h>
#include <random>

// https://github.com/jal278/plotpipe
#include "graph.h"

#include "jmt.h"
#include "coordinates.h"
#include "trajectory.h"
#include "neighbor.h"
#include <chrono>

using namespace std;

const double MIPH_TO_MPS = (5280 / 1.) * (1. / 3.2808) * (1 / 3600.);


class Planner {
private:
    plot map_plot;
    std::random_device rd;
    int current_lane = 0;
    int goal_lane = 0;
    long last_lane_change_time_ms;

    int get_lane(double d);

    int get_lane(FrenetPose fp);

    int get_lane(WorldPose wp);

    long construction_time;

    double uniform_random(double low = 0, double high = 1);

    int uniform_random(int low = 0, int high_plus_one = 11);

    long now();

public:
    CoordinateTransformer transform;
    // m/s, NOT mph
    double MAX_SPEED = 48 * MIPH_TO_MPS;
    double MIN_SPEED = 5;
    int PLAN_LENGTH = 1000;
    double EXT_TIME = 1.5;
    unsigned long NUM_REUSED = 8;

    Planner(CoordinateTransformer &transform);

    Trajectory make_plan(
            WorldPose current,
            double current_speed,
            Trajectory leftover,
            vector<Neighbor> neighbors,
            const double dt = .02,
            bool DEBUG = false
    );

    void show_map(vector<Trajectory> plans, vector<Neighbor> neighbors);

    double get_cost(Trajectory plan, vector<Neighbor> neighbors, string label = "", bool heading = false);

};


#endif //PATH_PLANNING_DECISION_MAKING_H
