//
// Created by tsbertalan on 7/2/2018.
//

#ifndef PATH_PLANNING_JMT_H
#define PATH_PLANNING_JMT_H


#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include <vector>


class PolyPath1D {
private:
    Eigen::VectorXd coefficients;

public:
    PolyPath1D(Eigen::VectorXd coefficients);

    double operator()(double x);

    double derivative(double x, int order = 1);
};


class PolyTrajectory {
public:

    std::vector<PolyPath1D> paths;

    PolyTrajectory(std::vector<PolyPath1D> paths);

    std::vector<double> operator()(double t);
};


PolyPath1D JMT_single_channel(double yi, double ydi, double yddi, double yf, double ydf, double yddf, double T);


PolyTrajectory JMT(
        double si, double spi, double sddi, double sf, double sdf, double sddf,
        double di, double dpi, double dppi, double df, double dpf, double dppf,
        double T);

#endif //PATH_PLANNING_JMT_H
