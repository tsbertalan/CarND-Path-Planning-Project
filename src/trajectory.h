//
// Created by tsbertalan on 7/4/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include "coordinates.h"


class Trajectory {
private:
    void init(std::vector<double> X, std::vector<double> Y, double dt = .02);

public:
    Trajectory(std::vector<double> X, std::vector<double> Y, double dt = .02);

    Trajectory(double dt = .02);

    std::vector<std::vector<double>> decompose();

    Trajectory subtrajectory(int end, int start = 0);

    std::vector<WorldPose> poses;
    std::vector<double> times;

    unsigned long size();
};


#endif //PATH_PLANNING_TRAJECTORY_H