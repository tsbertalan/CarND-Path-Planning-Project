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
    Trajectory cruise = leftover.subtrajectory(reuse_most_points + 1, 0, dt);
    double t0 = dt * cruise.size();

    if (cruise.size() > 1)
        transform.set_reference(cruise.poses[cruise.size() - 1]);
    else
        transform.set_reference(current);

    cout << "Kept " << cruise.size() << " points for a t0 of " << t0 << "." << endl;

    CarPose root;
    double root_sspeed = current_speed;
    double root_dspeed = 0;
    double root_saccel = 0;
    double root_daccel = 0;

    if (cruise.size() > 0) {
        root = transform.toCar(cruise.poses[cruise.size() - 1]);
        if (cruise.size() > 1) {
            CarPose rm1 = transform.toCar(cruise.poses[cruise.size() - 2]);
            root_sspeed = (root.x - rm1.x) / dt;
            root_dspeed = (root.y - rm1.y) / dt;

            if (cruise.size() > 2) {
                CarPose rm2 = transform.toCar(cruise.poses[cruise.size() - 3]);

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
//    DT = dt * (plan_length - cruise.size());
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

    double t = t0;
    while (t < t0 + DT) {
        t += dt;

        vector<double> xy = pt(t - t0);
        CarPose pose = {.x=xy[0], .y=xy[1], .yaw=0};
        WorldPose wp = transform.toWorld(pose);
        cruise.poses.push_back(wp);
        cruise.times.push_back(t);

        if (cruise.size() == plan_length) {
            break;
        }
    }

    cout << "Final t achieved is actually " << t << "." << endl;
    cout << "final cruise has " << cruise.size() << " points." << endl;


    // Maintain plots.
    vector<float> s, d;
    for (WorldPose pose : cruise.poses) {
        FrenetPose fp = transform.toFrenet(pose);
        s.push_back(fp.s);
        d.push_back(fp.d);
    }
    cout << endl;
    p1.plot_data(s, d, "points", "d vs s");

    vector<float> X, Y;
    for (WorldPose pose : cruise.poses) {
        X.push_back(pose.x);
        Y.push_back(pose.y);
    }
    p2.plot_data(X, Y, "points", "y vs x (world)");

    vector<float> T, V;
    for (int i = 1; i < cruise.size(); i++) {
        T.push_back(cruise.times[i]);
        double dx = cruise.poses[i].x - cruise.poses[i - 1].x;
        double dy = cruise.poses[i].y - cruise.poses[i - 1].y;
        V.push_back(sqrt(pow(dx, 2) + pow(dy, 2)) / dt);
    }
    p3.plot_data(T, V, "points", "speed vs t");

//    cruise.times[1] - cruise.times[0]

    vector<float> Xc, Yc;
    for (WorldPose pose : cruise.poses) {
        CarPose cp = transform.toCar(pose);
        Xc.push_back(cp.x);
        Yc.push_back(cp.y);
    }
    p4.plot_data(Xc, Yc, "points", "y vs x (car)");

    cout << "current_speed=" << current_speed << "; Ds=" << Ds << endl;

    if (xcdi > 1) {
        cout << "xcdi=" << xcdi << endl;
    }

    return cruise;
}

Planner::Planner(
        const CoordinateTransformer &transform,
        double target_max_speed,
        int plan_length,
        int min_reused_points
)
        : transform(transform), target_max_speed(target_max_speed), plan_length(plan_length),
          min_reused_points(min_reused_points) {
    ;
}
