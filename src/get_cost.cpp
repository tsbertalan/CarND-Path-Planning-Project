//
// Created by tsbertalan on 7/6/18.
//

#include "decision_making.h"

#include "parameters.h"


CostDecision Planner::get_cost(Trajectory &plan, vector<Neighbor> neighbors, string label, bool heading) {



    vector<const char *> cost_names;
    vector<double> cost_parts;


    // TODO: Reformulate costs as [0,1]-valued functions.

    // TODO: Interleave all time-dependent costs in one loop.


    //// Check whether the plan entails likely collisions.
    // TODO: Analyze and consider side-swipe scenarios.
    // TODO: Put neighbor loop inside time loop, for interleaving with other time-dependent costs.
    // TODO: Apply a discount factor for cost borne further in the future.
    double cost_dist = 0;
    int i_neighbor = -1;
    for (auto neighbor : neighbors) {
        i_neighbor++;

        for (double t = 0; t <= plan.t_max(); t += .02) {
            double s = plan.s(t);
            double d = plan.d(t);
            FrenetPose other = neighbor.future_position_frenet(t);
            double ds = fabs(s - other.s);
            double dd = fabs(d - other.d);

            double cost;

            // Treat collisions as fatal.
            if (
                // Not /2 because it's our radius plus their radius (distance between centers).
                    ds <= CAR_LENGTH
                    && ds >= -CAR_LENGTH
                    && dd <= CAR_WIDTH
                    && dd >= -CAR_WIDTH
                    ) {
                cost = PENALTY_FATAL;

                // Passing is fine.
            } else if (dd < -CAR_WIDTH || dd > CAR_WIDTH) {
                cost = 0;

                // Following or being followed is linearly costly with s-distance.
            } else {
                if (ds > 0)
                    cost = max(1 - (ds - CAR_LENGTH) * SCALE_DISTANCE_X, 0.);
                else
                    cost = max(1 + (ds - CAR_LENGTH) * SCALE_DISTANCE_X, 0.);
            }

            // Take the largest such cost of the whole trajectory.
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
    for (double t = 0; t <= plan.t_max(); t += .02) {
        double accel = plan.accel(t);
        if (accel > cost_accel)
            cost_accel = accel;
        double jerk = plan.jerk(t);
        if (jerk > cost_jerk)
            cost_jerk = jerk;
    }

    cost_parts.push_back(cost_accel * FACTOR_ACCEL);
    cost_names.push_back("accel");

    cost_parts.push_back(cost_jerk * FACTOR_JERK);
    cost_names.push_back("jerk");


    // Put a pseudo-boolean cost on trajectories whose average accel exceeds


    //// Find the mean deviation from goal velocity; penalizing larger differences more.
    double cost_vdeviation = plan.speed(plan.t_max()) - GOAL_SPEED;
    if (cost_vdeviation > 0)
        cost_vdeviation *= FACTOR_POSITIVE_SPEED_DEVIATION;
    else
        cost_vdeviation *= -FACTOR_NEGATIVE_SPEED_DEVIATION;
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
    for (double t = 0; t <= plan.t_max(); t += .02) {
        double d = plan.d(t);
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
