//
// Created by tsbertalan on 7/3/18.
//
#include "decision_making.h"

Trajectory
Planner::make_plan(
        WorldPose current,
        double current_speed,
        Trajectory leftover,
        vector<Neighbor> neighbors
) {
    // Either extend the leftover trajectory in the intended lane, or bust a move.
    vector<Trajectory> plans;

    const double dt = .02;

    // Consider cruising on the current path.
    unsigned long reuse_most_points = min_reused_points;
    // TODO: Need more abstraction here.

    cout << "leftover.size()=" << leftover.size() << endl;
    Trajectory plan = leftover.subtrajectory(reuse_most_points + 1, 0, dt);
    double t0 = dt * plan.size();

    if (plan.size() > 1)
        transform.set_reference(plan.poses[plan.size() - 1]);
    else
        transform.set_reference(current);

    cout << "Kept " << plan.size() << " points for a t0 of " << t0 << "." << endl;

    CarPose root;
    double root_sspeed = current_speed;
    double root_dspeed = 0;
    double root_saccel = 0;
    double root_daccel = 0;

    if (plan.size() > 0) {
        root = transform.toCar(plan.poses[plan.size() - 1]);
        if (plan.size() > 1) {
            CarPose rm1 = transform.toCar(plan.poses[plan.size() - 2]);
            root_sspeed = (root.x - rm1.x) / dt;
            root_dspeed = (root.y - rm1.y) / dt;

            if (plan.size() > 2) {
                CarPose rm2 = transform.toCar(plan.poses[plan.size() - 3]);

                double sspeedm1 = (rm1.x - rm2.x) / dt;
                double dspeedm1 = (rm1.y - rm2.y) / dt;

                root_saccel = (root_sspeed - sspeedm1) / dt;
                root_daccel = (root_dspeed - dspeedm1) / dt;
            }
        }


    } else {
        root = transform.toCar(current);
    }

    double DT, Ds;
//    DT = dt * (plan_length - plan.size());
    DT = .5;
    Ds = (root_sspeed + target_max_speed) / 2 * DT;

    FrenetPose frenetRoot = transform.toFrenet(root);
    FrenetPose frenetLeaf = {.s=frenetRoot.s + Ds, .d=2 + 4 * 2, .yaw=0};
    CarPose leaf = transform.toCar(frenetLeaf);

    double xci, xcdi, xcddi, xcf, xcdf, xcddf, yci, ycdi, ycddi, ycf, ycdf, ycddf;
    xci = root.x;
    xcdi = root_sspeed;
    xcddi = root_saccel;

    xcf = leaf.x;
    xcdf = target_max_speed;
    xcddf = 0;

    yci = root.y;
    ycdi = root_dspeed;
    ycddi = root_daccel;

    ycf = leaf.y;
    ycdf = 0;
    ycddf = 0;

    PolyTrajectory pt = JMT(
            xci, xcdi, xcddi,
            xcf, xcdf, xcddf,
            yci, ycdi, ycddi,
            ycf, ycdf, ycddf,
            DT
    );

    cout << "JMT = ";
    cout << "(";
    cout << xci << "," << xcdi << "," << xcddi << ",";
    cout << xcf << "," << xcdf << "," << xcddf;
    cout << "), ";

    cout << "(";
    cout << yci << "," << ycdi << "," << ycddi << ",";
    cout << ycf << "," << ycdf << "," << ycddf;
    cout << ")" << endl;

    cout << "Planning JMT from (xc,yc)=";
    cout << "(" << xci << "," << yci << ")";
    cout << " at t0=" << t0;
    cout << " to ";
    cout << "(" << xcf << "," << ycf << ")";
    cout << " at t=" << t0 + DT;
    cout << " (DT=" << DT << ")." << endl;

    plan.extend(pt, plan_length, DT, transform);

    cout << "Final t achieved is actually " << plan.times[plan.size() - 1] << "." << endl;
    cout << "final plan has " << plan.size() << " points." << endl;


    // Maintain plots.
    vector<float> s, d;
    for (WorldPose pose : plan.poses) {
        FrenetPose fp = transform.toFrenet(pose);
        s.push_back(fp.s);
        d.push_back(fp.d);
    }
    cout << endl;
    p1.plot_data(s, d, "points", "d vs s");

    vector<float> X, Y;
    for (WorldPose pose : plan.poses) {
        X.push_back(pose.x);
        Y.push_back(pose.y);
    }
    p2.plot_data(X, Y, "points", "y vs x (world)");

    vector<float> T, V;
    for (int i = 1; i < plan.size(); i++) {
        T.push_back(plan.times[i]);
        double dx = plan.poses[i].x - plan.poses[i - 1].x;
        double dy = plan.poses[i].y - plan.poses[i - 1].y;
        V.push_back(sqrt(pow(dx, 2) + pow(dy, 2)) / dt);
    }
    p3.plot_data(T, V, "points", "speed vs t");

    vector<float> Xc, Yc;
    for (WorldPose pose : plan.poses) {
        CarPose cp = transform.toCar(pose);
        Xc.push_back(cp.x);
        Yc.push_back(cp.y);
    }
    p4.plot_data(Xc, Yc, "points", "y vs x (car)");

    cout << "current_speed=" << current_speed << "; Ds=" << Ds << endl;

    if (xcdi > 1) {
        cout << "xcdi=" << xcdi << endl;
    }

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
