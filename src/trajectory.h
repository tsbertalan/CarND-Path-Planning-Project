//
// Created by tsbertalan on 7/4/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include "coordinates.h"
#include "jmt.h"
#include <string>
#include <sstream>
#include "spline.h"

class Trajectory {
private:
    double dt, Ds_used, DT_used;

public:
    double get_Ds_used() const;

    double getDT_used() const;

private:

    void init(std::vector<double> X, std::vector<double> Y, double dt = .02);

    void extend(PolyTrajectory sdpath_callable, unsigned long max_length, double DT, CoordinateTransformer &transform);

public:

    std::vector<WorldPose> poses;
    std::vector<std::vector<double>> sdtpath;
    std::vector<double> times;

    Trajectory(std::vector<double> X, std::vector<double> Y, double dt = .02);

    explicit Trajectory(double dt = .02);

    unsigned long size();

    bool empty();

    std::vector<std::vector<double>> decompose();

    Trajectory subtrajectory(int end, int start = 0, double dt = .02);

    void JMT_extend(
            CoordinateTransformer transform,
            double final_speed,
            unsigned int plan_length,
            WorldPose current,
            double current_speed,
            double text,
            double final_d = 2 + 4 * 2,
            double Ds = -1,
            double DT = .75
    );

    WorldPose initial();

    WorldPose ultimate();

    WorldPose penultimate();

    WorldPose antepenultimate();

    WorldPose preantepenultimate();

    WorldPose suprapreantepenultimate();

};


#endif //PATH_PLANNING_TRAJECTORY_H