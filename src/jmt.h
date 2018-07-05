//
// Created by tsbertalan on 7/2/2018.
//

#ifndef PATH_PLANNING_JMT_H
#define PATH_PLANNING_JMT_H


#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include <vector>


class PolyPath {
private:
    Eigen::VectorXd coefficients;
    double upper_bound_slope;
    double upper_bound_value;

    double call(double x);

public:
    double upper_bound;

    PolyPath(Eigen::VectorXd coefficients, double upper_bound);

    double operator()(double x, bool threshold = true);

    std::vector<double> operator()(std::vector<double> X);
};


class PolyTrajectory {
private:
    std::vector<PolyPath> paths;

public:
    PolyTrajectory(std::vector<PolyPath> paths);

    std::vector<double> operator()(double t);

//    std::vector<std::vector<double>> operator()(std::vector<double> T) {
//        std::vector<std::vector<double>> states;
//        for(double t : T) {
//            states.push_back(
//                    (*this)(t)
//            );
//        }
//        return states;
//    }

    std::vector<std::vector<double>> operator()(std::vector<double> T);

};


PolyPath JMT_single_channel(double yi, double ydi, double yddi, double yf, double ydf, double yddf, double T);


PolyTrajectory JMT(
        double si, double sdi, double sddi, double sf, double sdf, double sddf,
        double di, double ddi, double dddi, double df, double ddf, double dddf,
        double T);

#endif //PATH_PLANNING_JMT_H
