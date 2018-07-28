//
// Created by tsbertalan on 7/2/2018.
//

#ifndef PATH_PLANNING_JMT_H
#define PATH_PLANNING_JMT_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include <vector>

// Polynomial evaluation object.
class PolyPath1D {
 private:
  std::vector<double> coefficients;

 public:

  // Coefficients are in ascending order, starting with the constant term.
  PolyPath1D(std::vector<double> coefficients);

  // Evaluate the polynomial.
  double operator()(double x);

  // Evaluate the derivatives of the polynomial.
  double derivative(double x, int order = 1);
};

// Multiple-dependent variable (e.g., s and d) polynomial in one independent variable.
class PolyTrajectory {
 public:

  std::vector<PolyPath1D> paths;

  PolyTrajectory(std::vector<PolyPath1D> paths);

  // Evaluate all dependent/response variables at one point in the independent.
  std::vector<double> operator()(double t);
};

// Given boundary conditions, compute a jerk-minimizing trajectory in one dependent variable.
PolyPath1D JMT_single_channel(
    double yi, double ydi, double yddi,
    double yf, double ydf, double yddf,
    double T
);

// Given boundary conditions, compute two-dependent-variable JMT.
PolyTrajectory JMT(
    double si, double spi, double sddi, double sf, double sdf, double sddf,
    double di, double dpi, double dppi, double df, double dpf, double dppf,
    double T);

#endif //PATH_PLANNING_JMT_H
