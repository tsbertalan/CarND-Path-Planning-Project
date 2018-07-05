//
// Created by tsbertalan on 7/2/2018.
//

#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


std::vector<double> PolyPath::operator()(std::vector<double> X) {
    std::vector<double> Y;
    for (double x : X) {
        Y.push_back((*this)(x));
    }
    return Y;
}

double PolyPath::operator()(double x, bool threshold) {
    if (threshold && x >= upper_bound) {
        return upper_bound_slope * (x - upper_bound) + upper_bound_value;
    } else {
        double y = 0;
        for (int p = 0; p < coefficients.size(); p++) {
            y += pow(x, p) * coefficients[p];
        }
        return y;
    }
}

PolyPath::PolyPath(VectorXd coefficients, double upper_bound)
        : coefficients(coefficients), upper_bound(upper_bound) {
    double ymax = (*this)(upper_bound, false);
    double dt = .01;
    double ymaxm1 = (*this)(upper_bound - dt, false);
    upper_bound_slope = (ymax - ymaxm1) / dt;
    upper_bound_value = ymax;
}


std::vector<double> PolyTrajectory::operator()(double t) {
    std::vector<double> state;
    for (PolyPath path : paths) {
        state.push_back(path(t));
    }
    return state;
}

PolyTrajectory::PolyTrajectory(std::vector<PolyPath> paths) {
    this->paths = paths;
}

std::vector<std::vector<double>> PolyTrajectory::operator()(std::vector<double> T) {
    std::vector<std::vector<double>> evaluated_paths;
    for (PolyPath path : paths) {
        evaluated_paths.push_back(
                path(T)
        );
    }
    return evaluated_paths;
}

PolyPath JMT_single_channel(double yi, double ydi, double yddi, double yf, double ydf, double yddf, double T) {
    double a0, a1, a2, a3, a4, a5;

    a0 = yi;
    a1 = ydi;
    a2 = yddi * .5;

    MatrixXd rhs(3, 1);
    rhs <<
        yf - (yi + ydi * T + yddi * .5 * T * T),
            ydf - (ydi + yddi * T),
            yddf - yddi;

    double T2, T3, T4, T5;
    T2 = pow(T, 2);
    T3 = pow(T, 3);
    T4 = pow(T, 4);
    T5 = pow(T, 5);

    MatrixXd A(3, 3);
    A <<
      T3, T4, T5,
            3 * T2, 4 * T3, 5 * T4,
            6 * T, 12 * T2, 20 * T3;

    MatrixXd a345 = A.inverse() * rhs;
    a3 = a345(0);
    a4 = a345(1);
    a5 = a345(2);

    VectorXd coefficients(6);
    coefficients << a0, a1, a2, a3, a4, a5;

    return PolyPath(coefficients, T);
}


PolyTrajectory
JMT(double si, double sdi, double sddi, double sf, double sdf, double sddf, double di, double ddi, double dddi,
    double df, double ddf, double dddf, double T) {
    PolyPath spath = JMT_single_channel(si, sdi, sddi, sf, sdf, sddf, T);
    PolyPath dpath = JMT_single_channel(di, ddi, dddi, df, ddf, dddf, T);
    return PolyTrajectory({spath, dpath});
}
