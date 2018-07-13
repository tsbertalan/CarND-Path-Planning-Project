//
// Created by tsbertalan on 7/6/18.
//

#include "decision_making.h"



CostDecision Planner::get_cost(Trajectory plan, vector<Neighbor> neighbors, string label, bool heading) {

    const double FACTOR_DISTANCE = 1;

    const double CAR_WIDTH = 3;
    const double CAR_LENGTH = 6;
    const double WWARN_RADIUS = CAR_WIDTH - .5;
    const double LWARN_RADIUS = CAR_LENGTH + 4;

    const double CRITICAL_DISTANCE_X = LWARN_RADIUS;
    const double SCALE_DISTANCE_X = .1;

    const double CRITICAL_DISTANCE_Y = WWARN_RADIUS;
    const double SCALE_DISTANCE_Y = 4;

    const double FACTOR_ACCEL = 1. / 24.;
    const double FACTOR_JERK = 1. / 96.;

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

    const double FACTOR_OOL = .001;

    vector<const char *> cost_names;
    vector<double> cost_parts;


    // TODO: Reformulate costs as [0,1]-valued functions.

    // TODO: Interleave all time-dependent costs in one loop.


    //// Check whether the plan entails likely collisions.
    // TODO: Analyze and consider side-swipe scenarios.
    // TODO: Put neighbor loop inside time loop, for interleaving with other time-dependent costs.
    double cost_dist = 0;
    int i_neighbor = -1;
    for (auto neighbor : neighbors) {
        i_neighbor++;
        double t0 = plan.times[0];

        for (int i = 0; i < plan.sdtpath.size(); i++) {
            vector<double> ego_sdt = plan.sdtpath[i];
            FrenetPose other = neighbor.future_position_frenet(ego_sdt[2] - t0);
            double ds = fabs(ego_sdt[0] - other.s);
            double dd = fabs(ego_sdt[1] - other.d);

            double cost;

            cost = expit(ds, CRITICAL_DISTANCE_X, -SCALE_DISTANCE_X)
                   * expit(dd, CRITICAL_DISTANCE_Y, -SCALE_DISTANCE_Y);
            if (cost > cost_dist)
                cost_dist = cost;
        }
    }
    cost_parts.push_back(cost_dist * FACTOR_DISTANCE);
    cost_names.push_back("dist");


    //// Add up the total acceleration.
    // TODO: Penalize latitudinal (d) and longitudinal (s) acceleration separately, and nonlinearly (increase cost sharply past limits).
    double cost_accel = 0;
    int num_accel = 0;
    double cost_jerk = 0;
    int num_jerk = 0;
    for (int i_t = 0; i_t < plan.size(); i_t++) {
        if(i_t >= 0) {
            WorldPose pose3 = plan.poses[i_t - 0];
            double t3 = plan.times[i_t - 0];

            if(i_t >= 1) {
                WorldPose pose2 = plan.poses[i_t - 1];
                double t2 = plan.times[i_t - 1];
                double v2 = get_world_dist(pose3, pose2) / (t3 - t2);

                if(i_t >= 2) {
                    WorldPose pose1 = plan.poses[i_t - 2];
                    double t1 = plan.times[i_t - 2];
                    double v1 = get_world_dist(pose2, pose1) / (t2 - t1);
                    double acceleration_1 = fabs((v2 - v1) / (t2 - t1));
                    if (acceleration_1 > cost_accel)
                        cost_accel = acceleration_1;

                    if(i_t >= 3) {
                        WorldPose pose0 = plan.poses[i_t - 3];
                        double t0 = plan.times[i_t - 3];

                        double v0 = get_world_dist(pose1, pose0) / (t1 - t0);
                        double acceleration_0 = fabs((v1 - v0) / (t1 - t0));
                        double jerk_0 = fabs((acceleration_1 - acceleration_0) / (t1 - t0));
                        if (jerk_0 > cost_jerk)
                            cost_jerk = jerk_0;
                    }
                }
            }
        }
    }

    cost_parts.push_back(cost_accel * FACTOR_ACCEL);
    cost_names.push_back("accel");

    cost_parts.push_back(cost_jerk * FACTOR_JERK);
    cost_names.push_back("jerk");

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


    // Make a lane-centering and out-of-road cost.
    double cost_out_of_lane = 0;
    const double c1 = 1;
    const double cinf = 10;
    const double sl1 = c1 / 2;
    const double sl2 = cinf / 2;
    for(int i_t=1; i_t < plan.sdtpath.size(); i_t++){
        vector<double> sdt = plan.sdtpath[i_t];
        double d = sdt[1];
        if(d < 0)
            cost_out_of_lane += -sl2 * d + c1;
        else if(d < 2)
            cost_out_of_lane += -sl1 * d + c1;
        else if(d < 4)
            cost_out_of_lane += sl1 * d - c1;
        else if(d < 6)
            cost_out_of_lane += -sl1 * d + 3 * c1;
        else if(d < 8)
            cost_out_of_lane += sl1 * d - 3 * c1;
        else if(d < 10)
            cost_out_of_lane += -sl1 * d + 5 * c1;
        else if(d < 12)
            cost_out_of_lane += sl1 * d - 5 * c1;
        else
            cost_out_of_lane += sl2 * d - 12 * sl2 + c1;
    }
    cost_parts.push_back(cost_out_of_lane * FACTOR_OOL);
    cost_names.push_back("OoL");


    //// TODO: Add a max-jerk cost.



    //// TODO: Add a distance-traveled negative cost.



    //// TODO: Add a positive cost for long-time trajectories (dump the vdeviation cost?).




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
