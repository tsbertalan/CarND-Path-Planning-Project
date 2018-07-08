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

#include "utils.h"
// TODO: Move implementation-specific includes to implementation files.

#include <fstream>

using namespace std;

const double MIPH_TO_MPS = (5280 / 1.) * (1. / 3.2808) * (1 / 3600.);

struct CostDecision {
    double cost;
    const char *reason;
    vector<double> cost_parts;
    vector<const char *> cost_part_names;
};


class Planner {
private:
    plot map_plot;
    std::random_device rd;
    int current_lane = 0;
    int goal_lane = 0;
    ofstream pyfile;

public:
    int getCurrent_lane() const;

    int getGoal_lane() const;

private:
    long last_lane_change_time_ms;

    int get_lane(double d);

    int get_lane(FrenetPose fp);

    int get_lane(WorldPose wp);

    bool cross_lane_plan(Trajectory plan);

    long construction_time;

    double uniform_random(double low = 0, double high = 1);

    int uniform_random(int low = 0, int high_plus_one = 11);

    long now();

public:
    CoordinateTransformer transform;
    // m/s, NOT mph

    Planner(CoordinateTransformer &transform);

    Trajectory make_plan(
            WorldPose current,
            double current_speed,
            Trajectory leftover,
            vector<Neighbor> neighbors,
            const double dt = .02
    );

    void show_map(vector<Trajectory> plans, vector<Neighbor> neighbors);

    CostDecision get_cost(Trajectory plan, vector<Neighbor> neighbors, string label = "", bool heading = false);

    bool plan_changes_goal(Trajectory plan);

    int get_lane(Trajectory plan, bool final = true);

    string describe_plan(Trajectory &plan, double current_speed, double target_speed, double DT);

    vector<const char *> evaluate_reasons(vector<CostDecision> &decisions);

    string declare_reasons(vector<CostDecision> &decisions, CostDecision &decision);

};


#endif //PATH_PLANNING_DECISION_MAKING_H
