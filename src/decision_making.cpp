//
// Created by tsbertalan on 7/3/18.
//
#include "decision_making.h"
#include "utils.h"

using namespace std;


Trajectory
Planner::make_plan(
        WorldPose current,
        double current_speed,
        Trajectory leftover,
        vector<Neighbor> neighbors,
        const double dt,
        bool DEBUG
) {
    // Either extend the leftover trajectory in the intended lane, or bust a move.
    vector<Trajectory> plans;
    vector<string> planNames;

    transform.set_reference(current);

    const double TEXT = 2;
    const double DT = 0.75;

    // Get the curent lane index.
    FrenetPose fp = transform.toFrenet(current);
    int current_lane;
    if (fp.d < 4)
        current_lane = 0;
    else if (fp.d < 8)
        current_lane = 1;
    else
        current_lane = 2;

    // Consider cruising on the current path.
    Trajectory cruise = leftover.subtrajectory(min_reused_points + 1, 0, dt);
    cruise.JMT_extend(
            transform,
            target_max_speed,
            plan_length,
            current,
            current_speed,
            TEXT,
            current_lane * 4 + 2
    );
    planNames.push_back("cruise");
    plans.push_back(cruise);

    // Switch to a lane.
    for (int otherlane = 0; otherlane < 3; otherlane++) {
        if (otherlane != current_lane) {
            Trajectory lane_switch = leftover.subtrajectory(min_reused_points + 1, 0, dt);
            lane_switch.JMT_extend(
                    transform,
                    target_max_speed,
                    plan_length,
                    current,
                    current_speed,
                    TEXT,
                    otherlane * 4 + 2
            );
            ostringstream oss;
            oss << "lane_switch" << current_lane << "to" << otherlane;
            planNames.push_back(oss.str());
            plans.push_back(lane_switch);
        }
    }

    // Evaluate costs.
    vector<double> costs;
    for (int i = 0; i < plans.size(); i++) {
        Trajectory plan = plans[i];
        string planName = planNames[i];
        costs.push_back(get_cost(plan, neighbors, planNames[i], i == 0));
    }

    // Choose a plan.
    int best_plan = -1;
    double lowest_cost = 9999999999;
    for (int i = 0; i < plans.size(); i++) {
        double cost = costs[i];
        if (cost < lowest_cost) {
            best_plan = i;
            lowest_cost = cost;
        }
    }

    // Describe the neighbors.
    if (neighbors.size() > 0) {
        vector<double> dists;
        for (auto n : neighbors) {
            dists.push_back(worldDist(n.current, current));
        }
        int imin = min_element(dists.begin(), dists.end()) - dists.begin();
        cout << "Closest neighbor is " << neighbors[imin].id << " at " << dists[imin] << "[m]." << endl;
    }
    cout << " ==== " << endl;

    Trajectory plan = plans[best_plan];

    show_map(plans, neighbors);

    if (DEBUG) show_trajectory(plan);

    return plan;
}

Planner::Planner(
        CoordinateTransformer &transform,
        double target_max_speed,
        int plan_length,
        int min_reused_points
)
        : transform(transform), target_max_speed(target_max_speed), plan_length(plan_length),
          min_reused_points(min_reused_points) {
    ;
}

void Planner::show_map(vector<Trajectory> plans, vector<Neighbor> neighbors) {

    WorldPose ego_now = plans[0].poses[0];

    transform.set_reference(ego_now);

    int iplan = -1;
    vector<vector<double>> X, Y, T;
    vector<string> styles;
    for (auto plan: plans) {
        iplan++;
        vector<double> x, y, t;
        for (WorldPose pose : plan.poses) {
            CarPose cp = transform.toCar(pose);
            x.push_back(cp.x);
            y.push_back(cp.y);
        }
        for (double tv : plan.times) {
            t.push_back(tv);
        }
        X.push_back(x);
        Y.push_back(y);
        T.push_back(t);
        styles.push_back("lines");
    }

    double s_now = transform.toFrenet(ego_now).s;

    for (auto n: neighbors) {
        vector<double> x, y;
        for (int i = 0; i < plans[0].size(); i++) {
            double t0 = plans[0].times[0];
            double t = plans[0].times[i];
            WorldPose other = n.future_position(t - t0, transform);
            CarPose cp = transform.toCar(other);
            x.push_back(cp.x);
            y.push_back(cp.y);
        }
        styles.push_back("lines");
        X.push_back(x);
        Y.push_back(y);
    }
    cout << endl;

    pmap.plot_data(X, Y, "x [m] (car)", "y [m] (car)", styles, "plans and neighbor projections", "t [s]",
                   plans[0].times);

}

void Planner::show_trajectory(Trajectory plan) {
    vector<double> s, d;
    for (WorldPose pose : plan.poses) {
        FrenetPose fp = transform.toFrenet(pose);
        s.push_back(fp.s);
        d.push_back(fp.d);
    }
    cout << endl;
    p1.plot_data(s, d, "points", "d vs s [m]");

    vector<double> X, Y;
    for (WorldPose pose : plan.poses) {
        X.push_back(pose.x);
        Y.push_back(pose.y);
    }
    p2.plot_data(X, Y, "points", "y vs x (world) [m]");

    vector<double> T, V;
    for (int i = 1; i < plan.size(); i++) {
        T.push_back(plan.times[i]);
        double dx = plan.poses[i].x - plan.poses[i - 1].x;
        double dy = plan.poses[i].y - plan.poses[i - 1].y;
        double dt = plan.times[i] - plan.times[i - 1];
        double mps = sqrt(pow(dx, 2) + pow(dy, 2)) / dt;
        double mph = mps * (1. / 5280.) * (3.2808 / 1.) * (3600. / 1);
        V.push_back(mph);
    }
    p3.plot_data(T, V, "points", "speed [mph] vs t [s]");

    vector<double> Xc, Yc;
    for (WorldPose pose : plan.poses) {
        CarPose cp = transform.toCar(pose);
        Xc.push_back(cp.x);
        Yc.push_back(cp.y);
    }
    p4.plot_data(Xc, Yc, "points", "y vs x (car) [m]");

    cout << "Plots updated." << endl;
}

double Planner::randAB(double low, double high) {
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(low, high);
    return dis(gen);
}

double Planner::get_cost(Trajectory plan, vector<Neighbor> neighbors, string label, bool heading) {


    const double COLLISION_COST = 4;
    const double ACCEL_COST = 1. / 28;

    int print_width = 12;
    vector<const char *> cost_names = {"dist", "accel"};
    vector<double> cost_parts;
    if (label.length() > 0 && heading) {
        cout << "    ";
        for (const char *n : cost_names) {
            printf("%*s", print_width, n);
        }
        cout << endl;
    }

    // Check whether the plan entails likely collisions.
    double dist_cost = 0;
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
            double d = worldDist(ego, other);
            double dfactor;
            if (d > 0)
                dfactor = 1. / d;
            else
                dfactor = 999999;
            dist_cost += dfactor;

            Xn.push_back(other.x);
            Yn.push_back(other.y);
        }

    }
    cost_parts.push_back(dist_cost * COLLISION_COST);

    // Add up the total acceleration.
    double accel_tot = 0;
    for (int i = 2; i < plan.size(); i++) {
        WorldPose pose0 = plan.poses[i - 2];
        WorldPose pose1 = plan.poses[i - 1];
        WorldPose pose2 = plan.poses[i - 0];

        double t0 = plan.times[i - 2];
        double t1 = plan.times[i - 1];
        double t2 = plan.times[i - 0];

        double v0 = worldDist(pose0, pose1) / (t1 - t0);
        double v1 = worldDist(pose1, pose2) / (t2 - t1);

        double accel = fabs((v1 - v0) / (t1 - t0));
        accel_tot += accel;
    }
    cost_parts.push_back(accel_tot * ACCEL_COST);

    // Total the cost.
    double cost = 0;
    for (double cp : cost_parts)
        cost += cp;

    // Print the parts.
    if (label.length() > 0) {
        cout << "sum(";
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
