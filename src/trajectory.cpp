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
        PolyTrajectory sdpath,
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


    // Evaluate the Frenet JMT.
    vector<double> coarse_x, coarse_y, coarse_t;
    for (double t = t0 + dt; t < t0 + text; t += 0.1) {

        vector<double> sd = sdpath(t - t0);
        FrenetPose pose = {.s=sd[0], .d=sd[1], .yaw=0};
        WorldPose wp = transform.to_world(pose);

//        poses.push_back(wp);
//        times.push_back(t);

        coarse_x.push_back(wp.x);
        coarse_y.push_back(wp.y);
        coarse_t.push_back(t);
    }

    // Make a fine interpolant with a spline.
    tk::spline sp_x, sp_y;
    sp_x.set_points(coarse_t, coarse_x);
    sp_y.set_points(coarse_t, coarse_y);

    // Use the interpolant.
    for (double t = t0 + dt; t < t0 + text; t += dt) {

        WorldPose wp = {.x=sp_x(t), .y=sp_y(t), .yaw=0};

        poses.push_back(wp);
        times.push_back(t);
//
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

    FrenetPose initial_pose = transform.to_frenet(current);

    double vs0 = current_speed;
    double as0 = 0;
    double vd0 = 0;
    double ad0 = 0;

    if (size() > 0) {
        initial_pose = transform.to_frenet(ultimate());

        // If we have two points, we can estimate the initial velocity.
        if (size() > 1) {
            vector<double> S, D;
            for (WorldPose p : poses) {
                FrenetPose fp = transform.to_frenet(p);
                S.push_back(fp.s);
                D.push_back(fp.d);
            }
            tk::spline s_spline, d_spline;
            s_spline.set_points(times, S);
            d_spline.set_points(times, D);

            double tf = times[size() - 1];
            vs0 = (s_spline(tf) - s_spline(tf - dt)) / dt;
            vd0 = (d_spline(tf) - d_spline(tf - dt)) / dt;

            // If we have three points, we can estimate the initial acceleration.
            if (size() > 2) {

                double vsm1 = (s_spline(tf - dt) - s_spline(tf - dt * 2)) / dt;
//                double vdm1 = (d_spline(tf - dt) - d_spline(tf - dt * 2)) / dt;

                as0 = (vs0 - vsm1) / dt;
//                ad0 = (vd0 - vdm1) / dt;
            }
        }
    }

    // Threshold the vd0 velocity--don't bother reproducing small initial sideways movement,
    // which might be just noise (from the imprecise transforms),
    // but still can have a macroscopic effect on the generated trajectories.
//    vd0 = expit(fabs(vd0), .5, 50) * vd0;
//    ad0 = expit(fabs(ad0), 2, 25) * ad0;


    if (Ds == -1) {
        // If distance to drive isn't given,
        // approximate it by pretending constant mean speed.
        Ds = (vs0 + final_speed) / 2 * DT;
    }

    FrenetPose leaf = {.s=initial_pose.s + Ds, .d=final_d, .yaw=0};

    double si, sdi, sddi, sf, sdf, sddf, di, ddi, dddi, df, ddf, dddf;
    si = initial_pose.s;
    sdi = vs0;
    sddi = as0;

    sf = leaf.s;
    sdf = final_speed;
    sddf = 0;

    di = initial_pose.d;
    ddi = vd0;
    dddi = ad0;

    df = leaf.d;
    ddf = 0;
    dddf = 0;

    PolyTrajectory pt = JMT(
            si, sdi, sddi,
            sf, sdf, sddf,
            di, ddi, dddi,
            df, ddf, dddf,
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
