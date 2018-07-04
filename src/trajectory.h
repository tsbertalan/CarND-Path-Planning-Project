//
// Created by tsbertalan on 7/4/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include "coordinates.h"
#include "jmt.h"


class Trajectory {
private:
    void init(std::vector<double> X, std::vector<double> Y, double dt = .02);

    double dt;

public:
    Trajectory(std::vector<double> X, std::vector<double> Y, double dt = .02);

    Trajectory(double dt = .02);

    std::vector<std::vector<double>> decompose();

    Trajectory subtrajectory(int end, int start = 0, double dt = .02);

    void extend(PolyTrajectory path, unsigned long max_length, double DT, CoordinateTransformer &transform);

    std::vector<WorldPose> poses;
    std::vector<double> times;

    unsigned long size();
};


#endif //PATH_PLANNING_TRAJECTORY_H