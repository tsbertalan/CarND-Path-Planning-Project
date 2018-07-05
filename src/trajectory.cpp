//
// Created by tsbertalan on 7/4/18.
//

#include "trajectory.h"
#include <assert.h>

using namespace std;

Trajectory::Trajectory(vector<double> X, vector<double> Y, double dt) {
    init(X, Y, dt);
}

Trajectory::Trajectory(double dt) {
    init({}, {}, dt);
}

vector<vector<double>> Trajectory::decompose() {

    vector<double> X, Y;
    for (auto pose : poses) {
        X.push_back(pose.x);
        Y.push_back(pose.y);
    }

    return {X, Y};
}

Trajectory Trajectory::subtrajectory(int end, int start, double dt) {
    Trajectory out({}, {}, dt);

    if (end < 0) {
        end += poses.size() + 1;
    }
    unsigned long send = end;

    if (start < 0) {
        start += poses.size();
    }
    unsigned long sstart = start;

    for (unsigned long i = sstart; i < send; i++) {
        if (i >= poses.size()) {
            break;
        }
        out.poses.push_back(poses[i]);
        out.times.push_back(times[i]);
    }

    return out;
}

void Trajectory::init(vector<double> X, vector<double> Y, double dt) {
    this->dt = dt;
    double last_yaw = 0;
    double t = -dt;
    for (int i = 0; i < X.size(); i++) {
        t += dt;
        WorldPose wp = {.x=X[i], .y=Y[i]};
        if (X.size() > 2 && i < X.size() - 1) {
            wp.yaw = atan2(Y[i + 1] - Y[i], X[i + 1] - X[i]);
            last_yaw = wp.yaw;
        } else {
            wp.yaw = last_yaw;
        }

        poses.push_back(wp);
        times.push_back(t);
    }
}

unsigned long Trajectory::size() {
    return poses.size();
}

void Trajectory::extend(
        PolyTrajectory path,
        unsigned long max_length,
        double text,
        CoordinateTransformer &transform
) {
    double t0;
    if (size() > 0) {
        t0 = times[size() - 1];
    } else {
        t0 = 0;
    }
    double t = t0;
    while (t < t0 + text) {
        t += dt;

        vector<double> xy = path(t - t0);
        CarPose pose = {.x=xy[0], .y=xy[1], .yaw=0};
        WorldPose wp = transform.toWorld(pose);
        poses.push_back(wp);
        times.push_back(t);

        if (size() == max_length) {
            break;
        }
    }

}

void Trajectory::JMT_extend(
        CoordinateTransformer transform,
        double final_speed,
        unsigned long plan_length,
        WorldPose current,
        double current_speed,
        double text,
        double final_d,
        double Ds,
        double DT
) {
    if (size() > 1)
        transform.set_reference(poses[size() - 1]);
    else
        transform.set_reference(current);

    CarPose root = transform.toCar(current);

    double root_sspeed = current_speed;
    double root_saccel = 0;
    double root_dspeed = 0;
    double root_daccel = 0;

    double t0 = dt * size();

    if (size() > 0) {
        root = transform.toCar(poses[size() - 1]);
        if (size() > 1) {
            CarPose rm1 = transform.toCar(poses[size() - 2]);
            root_sspeed = (root.x - rm1.x) / dt;
            root_dspeed = (root.y - rm1.y) / dt;

            if (size() > 2) {
                CarPose rm2 = transform.toCar(poses[size() - 3]);

                double sspeedm1 = (rm1.x - rm2.x) / dt;
                double dspeedm1 = (rm1.y - rm2.y) / dt;

                root_saccel = (root_sspeed - sspeedm1) / dt;
                root_daccel = (root_dspeed - dspeedm1) / dt;
            }
        }
    }

    if (Ds == -1) {
        Ds = (root_sspeed + final_speed) / 2 * DT;
    }

    FrenetPose frenetRoot = transform.toFrenet(root);
    FrenetPose frenetLeaf = {.s=frenetRoot.s + Ds, .d=final_d, .yaw=0};
    CarPose leaf = transform.toCar(frenetLeaf);

    double xci, xcdi, xcddi, xcf, xcdf, xcddf, yci, ycdi, ycddi, ycf, ycdf, ycddf;
    xci = root.x;
    xcdi = root_sspeed;
    xcddi = root_saccel;

    xcf = leaf.x;
    xcdf = final_speed;
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

    extend(pt, plan_length, text, transform);
}
