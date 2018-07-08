//
// Created by tsbertalan on 7/6/18.
//

#include "decision_making.h"



CostDecision Planner::get_cost(Trajectory plan, vector<Neighbor> neighbors, string label, bool heading) {

    const double FACTOR_DISTANCE = 1;

    const double CRITICAL_DISTANCE_X = 16;
    const double SCALE_DISTANCE_X = .1;

    const double CRITICAL_DISTANCE_Y = 2.5;
    const double SCALE_DISTANCE_Y = 4;

    const double FACTOR_ACCEL = 1. / 48;

    // If goal speed is too close to MAX_SPEED_CONSIDERED,
    // we'll be starved for fast-enough trajectories,
    // and might drop other criteria.
    const double GOAL_SPEED = 45 * MIPH_TO_MPS;
    const double FACTOR_POSITIVE_SPEED_DEVIATION = 1;
    const double FACTOR_NEGATIVE_SPEED_DEVIATION = .2;
    const double FACTOR_VDEV = .1;

    const double FACTOR_LANE_SW = .1;

    const double CRITICAL_SWITCHTIME = 500;
    const double SCALE_SWITCHTIME = .01;
    const double FACTOR_FASTSW = .4;

    vector<const char *> cost_names;
    vector<double> cost_parts;


    // TODO: Reformulate costs as [0,1]-valued functions.


    //// Check whether the plan entails likely collisions.
    // TODO: Use an anisotropic distance.
    // TODO: Analyze and consider side-swipe scenarios.
    double cost_dist = 0;
    int i_neighbor = -1;
    for (auto neighbor : neighbors) {
        i_neighbor++;
        double t0 = plan.times[0];

        for (int i = 0; i < plan.size(); i++) {
            double t = plan.times[i];
            CarPose ego = transform.to_car(plan.poses[i]);
            CarPose other = transform.to_car(neighbor.future_position(t - t0, transform));
            double dx = fabs(ego.x - other.x);
            double dy = fabs(ego.y - other.y);

            double cost;

            cost = expit(dx, CRITICAL_DISTANCE_X, -SCALE_DISTANCE_X)
                   * expit(dy, CRITICAL_DISTANCE_Y, -SCALE_DISTANCE_Y);
            if (cost > cost_dist)
                cost_dist = cost;
        }
    }
    cost_parts.push_back(cost_dist * FACTOR_DISTANCE);
    cost_names.push_back("dist");


    //// Add up the total acceleration.
    double cost_accel = 0;
    for (int i_t = 2; i_t < plan.size(); i_t++) {
        WorldPose pose0 = plan.poses[i_t - 2];
        WorldPose pose1 = plan.poses[i_t - 1];
        WorldPose pose2 = plan.poses[i_t - 0];

        double t0 = plan.times[i_t - 2];
        double t1 = plan.times[i_t - 1];
        double t2 = plan.times[i_t - 0];

        double v0 = get_world_dist(pose0, pose1) / (t1 - t0);
        double v1 = get_world_dist(pose1, pose2) / (t2 - t1);

        double accel = fabs((v1 - v0) / (t1 - t0));
        cost_accel += accel;
    }
    cost_accel /= plan.size() - 2;
    cost_parts.push_back(cost_accel * FACTOR_ACCEL);
    cost_names.push_back("accel");


    //// Find the mean deviation from goal velocity; penalizing larger differences more.
    double cost_vdeviation = 0;
    for (int i_t = 1; i_t < plan.size(); i_t++) {
        double dt = plan.times[i_t] - plan.times[i_t - 1];
        WorldPose p1 = plan.poses[i_t - 1];
        WorldPose p2 = plan.poses[i_t - 0];
        double dxdt = (p2.x - p1.x) / dt;
        double dydt = (p2.y - p1.y) / dt;
        double speed = sqrt(dxdt * dxdt + dydt * dydt);
        //double speed = get_world_dist(p1, p2) / fabs(dt);
        double vdev = speed - GOAL_SPEED;
        if (vdev > 0)
            vdev *= FACTOR_POSITIVE_SPEED_DEVIATION;
        else
            vdev *= -FACTOR_NEGATIVE_SPEED_DEVIATION;
        cost_vdeviation += vdev;
    }
    cost_vdeviation /= plan.size() - 1;
    cost_parts.push_back(cost_vdeviation * FACTOR_VDEV);
    cost_names.push_back("vdev");


    //// Penalize any lane shifts.
    int plan_goal_lane = get_lane(plan);
    bool plan_changes_goal_lane = goal_lane != plan_goal_lane;
    if (plan_changes_goal_lane)
        cost_parts.push_back(FACTOR_LANE_SW);
    else
        cost_parts.push_back(0);
    cost_names.push_back("sw");


    //// Penalise quickly repeated lane shifts.
    double cost_fastsw = 0;
    if (plan_changes_goal_lane) {
        cost_fastsw = expit(now() - last_lane_change_time_ms, CRITICAL_SWITCHTIME, -SCALE_SWITCHTIME);
    }
    cost_parts.push_back(cost_fastsw * FACTOR_FASTSW);
    cost_names.push_back("fastsw");


    //// TODO: Add an out-of-lane cost.


    //// TODO: Add a max-jerk cost.


    ////// Total the cost.
    double cost = 0;
    for (double cp : cost_parts)
        cost += cp;

    // Print the parts.
    int print_width = 12;
    if (label.length() > 0 && heading) {
        cout << "           ";
        for (const char *n : cost_names) {
            printf("%*s", print_width, n);
        }
        cout << endl;
    }
    int largest_component = argmax(cost_parts);
    const char *reason = cost_names[largest_component];
    if (label.length() > 0) {
        cout << "cost = sum(";
        const char *fmt = "%*.3f";
        for (double cp : cost_parts) {
            printf(fmt, print_width - 1, cp);
            if (cp == cost_parts[largest_component])
                printf("*");
            else
                printf(" ");
        }
        cout << ") =";
        printf(fmt, print_width, cost);
        cout << " for " << label << "." << endl;
    }

    CostDecision cd = {.cost=cost, .reason=reason, .cost_parts=cost_parts, .cost_part_names=cost_names};
    return cd;
}
