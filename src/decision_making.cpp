//
// Created by tsbertalan on 7/3/18.
//
#include "decision_making.h"
#include "utils.h"

using namespace std;
using namespace std::chrono;


Trajectory
Planner::make_plan(
        WorldPose current,
        double current_speed,
        Trajectory leftover,
        vector<Neighbor> neighbors,
        const double dt,
        bool DEBUG
) {
    auto start_planner_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    // Either extend the leftover trajectory in the intended lane, or bust a move.
    vector<Trajectory> plans;
    vector<string> plan_names;

    transform.set_reference(current);

    // Get the curent lane index.
    FrenetPose fp = transform.to_frenet(current);
    int current_lane;
    if (fp.d < 4)
        current_lane = 0;
    else if (fp.d < 8)
        current_lane = 1;
    else
        current_lane = 2;

    // Generate multiple plans.
    for (int otherlane = 0; otherlane < 3; otherlane++) {
        for (double target_speed = MIN_SPEED; target_speed < MAX_SPEED; target_speed += 5) {
            for (double DT = .5; DT < 1.75; DT += .25) {
                Trajectory lane_switch = leftover.subtrajectory(NUM_REUSED + 1, 0, dt);
                lane_switch.JMT_extend(
                        transform,
                        target_speed,
                        PLAN_LENGTH,
                        current,
                        current_speed,
                        EXT_TIME,
                        otherlane * 4 + 2,
                        -1,
                        DT
                );
                ostringstream oss;
                if (otherlane == current_lane)
                    oss << "cruise";
                else
                    oss << "lane_switch" << current_lane << "to" << otherlane;
                oss << " to " << target_speed << "[m/s]";
                oss << " in " << DT << "[s]";
                plan_names.push_back(oss.str());
                plans.push_back(lane_switch);
            }
        }
    }

    // Evaluate costs.
    vector<double> costs;
    for (int i = 0; i < plans.size(); i++) {
        Trajectory plan = plans[i];
        string planName = plan_names[i];
        costs.push_back(get_cost(plan, neighbors, plan_names[i], i == 0));
    }

    // Choose a plan.
    int best_plan = argmin(costs);
    double lowest_cost = costs[best_plan];
    cout << "Chose plan " << best_plan << " (" << plan_names[best_plan] << ")." << endl;

    // Describe the neighbors.
    if (neighbors.size() > 0) {
        vector<double> dists;
        for (auto n : neighbors) {
            dists.push_back(get_world_dist(n.current, current));
        }
        int imin = min_element(dists.begin(), dists.end()) - dists.begin();
        cout << "Closest neighbor is " << neighbors[imin].id << " at " << dists[imin] << "[m]." << endl;
    }


    Trajectory plan = plans[best_plan];

    show_map(plans, neighbors);

    auto end_planner_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    cout << " == planner took " << (end_planner_ms - start_planner_ms) << " [ms] == " << endl << endl;


    return plan;
}

Planner::Planner(CoordinateTransformer &transform) : transform(transform) {}

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


double Planner::randAB(double low, double high) {
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(low, high);
    return dis(gen);
}

double Planner::get_cost(Trajectory plan, vector<Neighbor> neighbors, string label, bool heading) {


    const double FACTOR_DISTANCE = 8;
    const double FACTOR_ACCEL = 1. / 30;
    const double CRITICAL_DISTANCE = 4.5;
    const double GOAL_SPEED = 47 * MIPH_TO_MPS;
    const double FACTOR_POSITIVE_SPEED_DEVIATION = 1;
    const double FACTOR_NEGATIVE_SPEED_DEVIATION = .1;
    const double FACTOR_VDEV = .05;

    int print_width = 12;
    vector<const char *> cost_names = {"dist", "accel", "vdev"};
    vector<double> cost_parts;
    if (label.length() > 0 && heading) {
        cout << "           ";
        for (const char *n : cost_names) {
            printf("%*s", print_width, n);
        }
        cout << endl;
    }

    //// Check whether the plan entails likely collisions.
    double cost_dist = 0;
    int ineighbor = -1;

    vector<double> X, Y, T;
    for (auto xy: plan.poses) {
        X.push_back(xy.x);
        Y.push_back(xy.y);
    }
    for (auto t: plan.times) T.push_back(t);

    for (auto neighbor : neighbors) {
        ineighbor++;
        double t0 = plan.times[0];

        vector<double> Xn, Yn;

        for (int i = 0; i < plan.size(); i++) {
            double t = plan.times[i];
            WorldPose ego = plan.poses[i];
            WorldPose other = neighbor.future_position(t - t0, transform);
            double d = get_world_dist(ego, other);
            double dfactor;
            if (d > 0)
                dfactor = 1 / (1 + exp(d - CRITICAL_DISTANCE));
            else
                dfactor = 999999;
            cost_dist += dfactor;

            Xn.push_back(other.x);
            Yn.push_back(other.y);
        }
    }
    cost_dist /= neighbors.size() * plan.size();
    cost_parts.push_back(cost_dist * FACTOR_DISTANCE);


    //// Add up the total acceleration.
    double cost_accel = 0;
    for (int i = 2; i < plan.size(); i++) {
        WorldPose pose0 = plan.poses[i - 2];
        WorldPose pose1 = plan.poses[i - 1];
        WorldPose pose2 = plan.poses[i - 0];

        double t0 = plan.times[i - 2];
        double t1 = plan.times[i - 1];
        double t2 = plan.times[i - 0];

        double v0 = get_world_dist(pose0, pose1) / (t1 - t0);
        double v1 = get_world_dist(pose1, pose2) / (t2 - t1);

        double accel = fabs((v1 - v0) / (t1 - t0));
        cost_accel += accel;
    }
    cost_accel /= plan.size() - 2;
    cost_parts.push_back(cost_accel * FACTOR_ACCEL);


    //// Find the mean deviation from goal velocity; penalizing larger differences more.
    double cost_vdeviation = 0;
    for (int i = 1; i < plan.size(); i++) {
        double dt = plan.times[i] - plan.times[i - 1];
        WorldPose p1 = plan.poses[i - 1];
        WorldPose p2 = plan.poses[i - 0];
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


    ////// Total the cost.
    double cost = 0;
    for (double cp : cost_parts)
        cost += cp;

    // Print the parts.
    if (label.length() > 0) {
        cout << "cost = sum(";
        const char *fmt = "%*.3f";
        for (double cp : cost_parts) {
            printf(fmt, print_width, cp);
        }
        cout << ") =";
        printf(fmt, print_width, cost);
        cout << " for " << label << "." << endl;
    }

    return cost;
}
