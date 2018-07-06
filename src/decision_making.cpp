//
// Created by tsbertalan on 7/3/18.
//
#include <map>
#include "decision_making.h"
#include "utils.h"

using namespace std;
using namespace std::chrono;

int Planner::get_lane(double d) {
    if (d < 4)
        return 0;
    else if (d < 8)
        return 1;
    else
        return 2;
}

int Planner::get_lane(FrenetPose fp) {
    return get_lane(fp.d);
}

int Planner::get_lane(WorldPose wp) {
    return get_lane(transform.to_frenet(wp));
}

int Planner::get_lane(Trajectory plan, bool final) {
    if (final)
        return get_lane(plan.poses[plan.size() - 1]);
    else
        return get_lane(plan.poses[0]);
}



Trajectory
Planner::make_plan(
        WorldPose current,
        double current_speed,
        Trajectory leftover,
        vector<Neighbor> neighbors,
        const double dt,
        bool DEBUG
) {
    long start_planner_ms = now();

    // Either extend the leftover trajectory in the intended lane, or bust a move.
    vector<Trajectory> plans;
    vector<string> plan_names;

    transform.set_reference(current);

    // Get the curent lane index.
    current_lane = get_lane(current);


    const int NUM_PLANS = 128;
    const bool SHOW_ALL_PLANS = true;

    // Generate multiple plans.
    for (int iplan = 0; iplan < NUM_PLANS; iplan++) {

        int otherlane = uniform_random(0, 3);
        double target_speed = uniform_random(MIN_SPEED_CONSIDERED, MAX_SPEED_CONSIDERED);
        double DT = uniform_random(.5, 1.75);
        double d_offset = 0;
        double DS = -1;
//        double DS=uniform_random(8, 32);

        Trajectory lane_switch = leftover.subtrajectory(NUM_REUSED + 1, 0, dt);
        lane_switch.JMT_extend(
                transform,
                target_speed,
                PLAN_LENGTH,
                current,
                current_speed,
                EXT_TIME,
                otherlane * 4 + 2 + d_offset,
                DS,
                DT
        );
        ostringstream oss;
        if (otherlane == goal_lane)
            oss << "cruise";
        else
            oss << "lane_switch" << current_lane << "to" << otherlane;
        oss << " to " << target_speed << "[m/s]";
        oss << " in " << DT << "[s]";
        plan_names.push_back(oss.str());
        plans.push_back(lane_switch);
    }

    // Evaluate costs.
    vector<CostDecision> decisions;
    vector<double> costs;
    for (int i = 0; i < plans.size(); i++) {
        Trajectory plan = plans[i];
        string planName = plan_names[i];
        ostringstream oss;
        string label;
        if (SHOW_ALL_PLANS) {
            oss << "#" << i << ": " << plan_names[i];
            label = oss.str();
        }
        CostDecision decision = get_cost(plan, neighbors, label, i == 0);
        costs.push_back(decision.cost);
        decisions.push_back(decision);
    }


    // What was the most common reason for a bad cost?
    map<const char *, int> histogram;
    for (auto d: decisions) {
        if (histogram.find(d.reason) == histogram.end())
            histogram[d.reason] = 0;
        histogram[d.reason] += 1;
    }
    vector<const char *> reasons;
    vector<int> counts;
    for (auto const &x : histogram) {
        reasons.push_back(x.first);
        counts.push_back(x.second);
    }
    vector<unsigned long> reason_prevalences = argsort(counts);
    reverse(reason_prevalences.begin(), reason_prevalences.end());
    const char *primary_reason = reasons[reason_prevalences[0]];
    const char *secondary_reason = primary_reason;
    if (counts.size() > 1)
        secondary_reason = reasons[reason_prevalences[1]];


    // Choose a plan.
    int best_plan = argmin(costs);
    double lowest_cost = costs[best_plan];
    Trajectory plan = plans[best_plan];
    cout << "Chose plan " << best_plan << " (" << plan_names[best_plan] << ")";
    CostDecision best_dec = decisions[best_plan];
    double pri_reason_val = best_dec.cost_parts[
            find(best_dec.cost_part_names.begin(), best_dec.cost_part_names.end(), primary_reason)
            - best_dec.cost_part_names.begin()
    ];
    double sec_reason_val = best_dec.cost_parts[
            find(best_dec.cost_part_names.begin(), best_dec.cost_part_names.end(), secondary_reason)
            - best_dec.cost_part_names.begin()
    ];
    if (best_dec.reason == primary_reason) {
        cout << " because of " << secondary_reason << "=" << sec_reason_val;
        if (primary_reason != secondary_reason)
            cout << " and despite " << primary_reason << "=" << pri_reason_val;
    } else {
        cout << " because of " << primary_reason << "=" << pri_reason_val;
    }
    cout << "." << endl;

    // Record the time that a lane switch was planned.
    int plan_goal_lane = get_lane(plan);
    if (plan_goal_lane != current_lane) {
        last_lane_change_time_ms = now();
    }
    if (plan_goal_lane != goal_lane)
        cout << " ======== LANE CHANGE ======= " << endl;
    goal_lane = plan_goal_lane;


    // Describe the neighbors.
    if (neighbors.size() > 0) {
        vector<double> dists;
        for (auto n : neighbors) {
            dists.push_back(get_world_dist(n.current, current));
        }
        int imin = min_element(dists.begin(), dists.end()) - dists.begin();
        cout << "Closest neighbor is " << neighbors[imin].id << " at " << dists[imin] << "[m]." << endl;
    }

    if (DEBUG) show_map(plans, neighbors);

    long end_planner_ms = now();
    cout << " == planner took " << (end_planner_ms - start_planner_ms) << " [ms] == " << endl;// << endl;


    return plan;
}

long Planner::now() {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - construction_time;
}

Planner::Planner(CoordinateTransformer &transform) : transform(transform) {
    construction_time = 0;
    construction_time = now();
    last_lane_change_time_ms = 0;
}

void Planner::show_map(vector<Trajectory> plans, vector<Neighbor> neighbors) {

    WorldPose ego_now = plans[0].poses[0];

    transform.set_reference(ego_now);

    const double DIST_VIZ_CAP = 30.;

    int iplan = -1;
    vector<vector<double>> X, Y, T, C;
    vector<string> styles;
    for (auto plan: plans) {
        iplan++;
        vector<double> x, y, t, c;
        for (WorldPose pose : plan.poses) {
            CarPose cp = transform.to_car(pose);
            x.push_back(cp.x);
            y.push_back(cp.y);
        }
        for (double tv : plan.times) {
            t.push_back(tv);
            c.push_back(0);
        }
        X.push_back(x);
        Y.push_back(y);
        T.push_back(t);
        C.push_back(c);
        styles.push_back("lines");
    }

    double s_now = transform.to_frenet(ego_now).s;

    for (auto n: neighbors) {
        vector<double> x, y, c;
        for (int i = 0; i < plans[0].size(); i++) {
            double t0 = plans[0].times[0];
            double t = plans[0].times[i];
            WorldPose other = n.future_position(t - t0, transform);
            CarPose cp = transform.to_car(other);
            x.push_back(cp.x);
            y.push_back(cp.y);
            double min_dist = 9999;
            for (auto p : plans) {
                min_dist = min(min_dist, get_world_dist(other, p.poses[i % p.size()]));
            }
            c.push_back(min(min_dist, DIST_VIZ_CAP));
        }
        styles.push_back("lines");
        X.push_back(x);
        Y.push_back(y);
        C.push_back(c);
    }

    map_plot.plot_data(
            X, Y,
            "x [m] (car)", "y [m] (car)", styles, "plans and neighbor projections", "capped dist to nearest plan",
            C
    );

}


double Planner::uniform_random(double low, double high) {
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(low, high);
    return dis(gen);
}

int Planner::uniform_random(int low, int high_plus_one) {
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(low, high_plus_one - 1);
    return dis(gen);
}

double expit(double x, double x_critical, double scale_factor = 1) {
    return 1. / (1. + (double) exp((x - x_critical) * scale_factor));
}

CostDecision Planner::get_cost(Trajectory plan, vector<Neighbor> neighbors, string label, bool heading) {

    const double FACTOR_DISTANCE_X = 1;
    const double FACTOR_DISTANCE_Y = 1;
    const double CRITICAL_DISTANCE_X = 10;
    const double SCALE_DISTANCE_X = .5;
    const double CRITICAL_DISTANCE_Y = 1.5;
    const double SCALE_DISTANCE_Y = 3.5;
    const double CHECK_SCALE = 1.5;

    const double FACTOR_ACCEL = 1. / 20;

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
    double cost_dist_x = 0;
    double cost_dist_y = 0;
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

            // Track the MAXIMUM costs obtained.
            cost = expit(dx, CRITICAL_DISTANCE_X, SCALE_DISTANCE_X);
            if (
                // Track the maximum.
                    cost > cost_dist_x
                    && dy < CRITICAL_DISTANCE_Y * CHECK_SCALE
                    )
                cost_dist_x = cost;
            cost = expit(dy, CRITICAL_DISTANCE_Y, SCALE_DISTANCE_Y);
            if ( // Track the maximum.
                    cost > cost_dist_y
                    // Don't reacto to close-in-y cars unless we're basically abreast of them.
                    // Thes kind of nonlinear effects make it clear
                    // why a NEURAL NETWORK would be good for this kind of stuff.
                    && dx < CRITICAL_DISTANCE_X * CHECK_SCALE
                    )
                cost_dist_y = cost;
        }
    }
    cost_parts.push_back(cost_dist_x * FACTOR_DISTANCE_X);
    cost_names.push_back("dist_x");
    cost_parts.push_back(cost_dist_y * FACTOR_DISTANCE_Y);
    cost_names.push_back("dist_y");


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
        cost_fastsw = expit(now() - last_lane_change_time_ms, CRITICAL_SWITCHTIME, SCALE_SWITCHTIME);
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
