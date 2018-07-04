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
    // TODO: Need more abstraction here.

    cout << "leftover.size()=" << leftover.size() << endl;
    Trajectory cruise = leftover.subtrajectory(min_reused_points + 1);
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
    DT = .02 * (plan_length - cruise.size());
    Ds = (root_sspeed + target_max_speed) / 2 * DT;

    FrenetPose frenetRoot = transform.toFrenet(root);
    FrenetPose frenetLeaf = {.s=frenetRoot.s + Ds, .d=2, .yaw=0};
    CarPose leaf = transform.toCar(frenetLeaf);

    double si, sdi, sddi, sf, sdf, sddf, di, ddi, dddi, df, ddf, dddf;
    si = root.x;
    sdi = root_sspeed;
    sddi = root_saccel;

    sf = leaf.x;
    sdf = target_max_speed;
    sddf = 0;

    di = root.y;
    ddi = root_dspeed;
    dddi = root_daccel;

    df = leaf.y;
    ddf = 0;
    dddf = 0;

    PolyTrajectory pt = JMT(
            si, sdi, sddi,
            sf, sdf, sddf,
            di, ddi, dddi,
            df, ddf, dddf,
            DT
    );

    cout << "JMT = ";
    cout << "(";
    cout << si << "," << sdi << "," << sddi << ",";
    cout << sf << "," << sdf << "," << sddf;
    cout << "), ";

    cout << "(";
    cout << di << "," << ddi << "," << dddi << ",";
    cout << df << "," << ddf << "," << dddf;
    cout << ")" << endl;

    cout << "Planning JMT from (s,d)=";
    cout << "(" << si << "," << di << ")";
    cout << " at t0=" << t0;
    cout << " to ";
    cout << "(" << sf << "," << df << ")";
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
    for (auto pose : cruise.poses) {
        FrenetPose fp = transform.toFrenet(pose);
        s.push_back(fp.s);
        d.push_back(fp.d);
    }
    cout << endl;
    p1.plot_data(s, d, "points", "d vs s");

    vector<float> X, Y;
    for (auto pose : cruise.poses) {
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

    vector<float> Xc, Yc;
    for (auto pose : cruise.poses) {
        CarPose cp = transform.toCar(pose);
        Xc.push_back(cp.x);
        Yc.push_back(cp.y);
    }
    p4.plot_data(Xc, Yc, "points", "y vs x (car)");

    cout << "current_speed=" << current_speed << "; Ds=" << Ds << endl;

    if (sdi > 1) {
        cout << "sdi=" << sdi << endl;
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
