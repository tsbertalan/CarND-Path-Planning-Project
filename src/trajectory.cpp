//
// Created by tsbertalan on 7/4/18.
//

#include "trajectory.h"

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
        double DT,
        CoordinateTransformer &transform
) {
    double t0;
    if (size() > 0) {
        t0 = times[size() - 1];
    } else {
        t0 = 0;
    }
    double t = t0;
    while (t < t0 + DT) {
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
