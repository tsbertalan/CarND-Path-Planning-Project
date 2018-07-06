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
        WorldPose wp = transform.to_world(pose);
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
        unsigned int plan_length,
        WorldPose current,
        double current_speed,
        double text,
        double final_d,
        double Ds,
        double DT
) {
    if (size() > 1)
        transform.set_reference(ultimate());
    else
        transform.set_reference(current);

    CarPose initial_pose = transform.to_car(current);

    double vs0 = current_speed;
    double as0 = 0;
    double vd0 = 0;
    double ad0 = 0;

    double t0 = dt * size();

    if (size() > 0) {
        initial_pose = transform.to_car(ultimate());
        if (size() > 1) {
            CarPose rm1 = transform.to_car(penultimate());
            vs0 = (initial_pose.x - rm1.x) / dt;
            vd0 = (initial_pose.y - rm1.y) / dt;

            if (size() > 2) {
                CarPose rm2 = transform.to_car(antepenultimate());

                double vsm1 = (rm1.x - rm2.x) / dt;
                double vdm1 = (rm1.y - rm2.y) / dt;

                as0 = (vs0 - vsm1) / dt;
                ad0 = (vd0 - vdm1) / dt;
            }
        }
    }

    if (Ds == -1) {
        Ds = (vs0 + final_speed) / 2 * DT;
    }

    FrenetPose frenetRoot = transform.to_frenet(initial_pose);
    FrenetPose frenetLeaf = {.s=frenetRoot.s + Ds, .d=final_d, .yaw=0};
    CarPose leaf = transform.to_car(frenetLeaf);

    double xci, xcdi, xcddi, xcf, xcdf, xcddf, yci, ycdi, ycddi, ycf, ycdf, ycddf;
    xci = initial_pose.x;
    xcdi = vs0;
    xcddi = as0;

    xcf = leaf.x;
    xcdf = final_speed;
    xcddf = 0;

    yci = initial_pose.y;
    ycdi = vd0;
    ycddi = ad0;

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

WorldPose Trajectory::initial() {
    return poses[0];
}

WorldPose Trajectory::ultimate() {
    return poses[size() - 1];
}

WorldPose Trajectory::penultimate() {
    return poses[size() - 2];
}

WorldPose Trajectory::antepenultimate() {
    return poses[size() - 3];
}

WorldPose Trajectory::preantepenultimate() {
    return poses[size() - 4];
}

WorldPose Trajectory::suprapreantepenultimate() {
    return poses[size() - 5];
}

bool Trajectory::empty() {
    return poses.empty();
}
