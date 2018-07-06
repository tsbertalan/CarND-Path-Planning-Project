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

class Trajectory {
private:
    double dt;

    void init(std::vector<double> X, std::vector<double> Y, double dt = .02);

    void extend(PolyTrajectory path, unsigned long max_length, double DT, CoordinateTransformer &transform);

public:
    std::vector<WorldPose> poses;
    std::vector<double> times;

    Trajectory(std::vector<double> X, std::vector<double> Y, double dt = .02);

    explicit Trajectory(double dt = .02);

    unsigned long size();

    std::vector<std::vector<double>> decompose();

    Trajectory subtrajectory(int end, int start = 0, double dt = .02);

    void JMT_extend(
            CoordinateTransformer transform,
            double final_speed,
            unsigned long plan_length,
            WorldPose current,
            double current_speed,
            double text,
            double final_d = 2 + 4 * 2,
            double Ds = -1,
            double DT = .75
    );

};


#endif //PATH_PLANNING_TRAJECTORY_H