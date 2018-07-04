//
// Created by tsbertalan on 7/3/18.
//
#include "decision_making.h"


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

    // Consider cruising on the current path.
    Trajectory plan = leftover.subtrajectory(min_reused_points + 1, 0, dt);
    if (plan.size() > 0)
        plan.JMT_extend(transform, target_max_speed, plan_length);
    else
        plan.JMT_extend(transform, target_max_speed, plan_length, current, current_speed);

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
}
