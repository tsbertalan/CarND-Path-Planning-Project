//
// Created by tsbertalan on 7/4/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <string>
#include <sstream>
#include <map>
#include "coordinates.h"
#include "jmt.h"
#include "spline.h"

struct State {
    double y, yp, ypp;
};

struct FullState {
    State s, d;
};


class TrajectorySegment {
    // A TrajectorySegment is a map from [0, 1] to RxR.
    // However, they first map time from t_pin_0 to t_pin_1
    // to this [0, 1] range.

    PolyTrajectory pt;

public:

    double remap(double t);

    double t_pin_0, t_pin_1;

    TrajectorySegment(double t_pin_0, double t_pin_1, FullState begin, FullState end);

    FrenetPose operator()(double t);

    double s_derivative(double t, int order = 1);

    double d_derivative(double t, int order = 1);

};

struct SegmentRemit {
    TrajectorySegment f;
    double t_responsible_0, t_responsible_1;
};

class Trajectory {
private:
    std::map<double, double>
            s_cache, d_cache,
            sp_cache, dp_cache,
            spp_cache, dpp_cache,
            sppp_cache, dppp_cache;

    std::vector<SegmentRemit> segments;

    FullState state(double t);

    CoordinateTransformer *transform;

public:

    double t_max();

    Trajectory(CoordinateTransformer *transform);

//    Trajectory(const Trajectory &parent);

    FrenetPose operator()(double t);

    double s(double t, bool ignore_tmax = false);

    double d(double t, bool ignore_tmax = false);

    double sp(double t, bool ignore_tmax = false);

    double dp(double t, bool ignore_tmax = false);

    double spp(double t);

    double dpp(double t);

    double sppp(double t);

    double dppp(double t);

    double speed(double t);

    double accel(double t);

    double jerk(double t);

    FrenetPose frenet(double t);

    WorldPose world(double t);

    Trajectory generate_extension(
            FrenetPose current, double t_reuse, double t_replan, double DT, double DS, double sp,
            double d, double spp = 0, double dp = 0, double dpp = 0
    );

    void cut_start(double t_reuse, double t_replan);

    SegmentRemit &get_remit(double t);

    std::string dumps();

    void plot(double t_reuse = -1, double t_replan = -1);

    std::vector<std::vector<double>> decompose();

};


#endif //PATH_PLANNING_TRAJECTORY_H