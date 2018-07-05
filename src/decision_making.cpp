//
// Created by tsbertalan on 7/3/18.
//
#include "decision_making.h"
#include "utils.h"


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

    transform.set_reference(current);

    const double TEXT = .8;

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
            plans.push_back(lane_switch);
        }
    }

    // Evaluate costs.
    vector<double> costs;
    for (auto plan : plans) {
        costs.push_back(get_cost(plan, neighbors));
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

    Trajectory plan = plans[best_plan];

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

void Planner::show_trajectory(Trajectory plan) {
    vector<float> s, d;
    for (WorldPose pose : plan.poses) {
        FrenetPose fp = transform.toFrenet(pose);
        s.push_back(fp.s);
        d.push_back(fp.d);
    }
    cout << endl;
    p1.plot_data(s, d, "points", "d vs s [m]");

    vector<float> X, Y;
    for (WorldPose pose : plan.poses) {
        X.push_back(pose.x);
        Y.push_back(pose.y);
    }
    p2.plot_data(X, Y, "points", "y vs x (world) [m]");

    vector<float> T, V;
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

    vector<float> Xc, Yc;
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

double Planner::get_cost(Trajectory plan, vector<Neighbor> neighbors) {
    double cost = 0;

    const double COLLISION_COST = 2;

    // Check whether the plan entains likely collisions.
    for (auto neighbor : neighbors) {
        double t0 = plan.times[0];
        for (int i = 0; i < plan.size(); i++) {
            double t = plan.times[i];
            WorldPose ego = plan.poses[i];
            WorldPose other = neighbor.future_position(t - t0, transform);
            double d = distance(ego.x, ego.y, other.x, other.y);
            double dfactor;
            if (d > 0)
                dfactor = 1. / d;
            else
                dfactor = 999999;
            cost += COLLISION_COST * dfactor;
        }
    }
    return cost;
}
