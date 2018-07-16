//
// Created by tsbertalan on 7/2/2018.
//

#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

PolyPath1D::PolyPath1D(VectorXd coefficients)
    : coefficients(coefficients) {
  double dt = .02;
}

double PolyPath1D::operator()(double x) {
  return derivative(x, 0);
}

double PolyPath1D::derivative(double x, int order) {
  double y = 0;
  for (int p = 0; p < coefficients.size(); p++) {
    double derivative_factor = 1;
    for (int k = 0; k < order; k++) {
      derivative_factor *= p - k;
      if (derivative_factor==0)
        break;
    }
    if (derivative_factor==0)
      continue;
    else {
      // Need to use double power; otherwise iext version of pow is used, which gives wrong results.
      y += derivative_factor*pow((double) x, (double) (p - order))*coefficients[p];
    }
  }
  return y;
}

std::vector<double> PolyTrajectory::operator()(double t) {
  std::vector<double> state;
  for (PolyPath1D &path : paths) {
    state.push_back(path(t));
  }
  return state;
}

PolyTrajectory::PolyTrajectory(std::vector<PolyPath1D> paths) {
  this->paths = paths;
}

PolyPath1D JMT_single_channel(double yi, double ydi, double yddi, double yf, double ydf, double yddf, double T) {
  double a0, a1, a2, a3, a4, a5;

  a0 = yi;
  a1 = ydi;
  a2 = yddi*.5;

  MatrixXd rhs(3, 1);
  rhs <<
      yf - (yi + ydi*T + yddi*.5*T*T),
      ydf - (ydi + yddi*T),
      yddf - yddi;

  double T2, T3, T4, T5;
  T2 = pow(T, 2);
  T3 = pow(T, 3);
  T4 = pow(T, 4);
  T5 = pow(T, 5);

  MatrixXd A(3, 3);
  A <<
    T3, T4, T5,
      3*T2, 4*T3, 5*T4,
      6*T, 12*T2, 20*T3;

  MatrixXd a345 = A.inverse()*rhs;
  a3 = a345(0);
  a4 = a345(1);
  a5 = a345(2);

  VectorXd coefficients(6);
  coefficients << a0, a1, a2, a3, a4, a5;

  return PolyPath1D(coefficients);
}

PolyTrajectory
JMT(
    double si, double spi, double sppi,
    double sf, double spf, double sppf,
    double di, double dpi, double dppi,
    double df, double dpf, double dppf,
    double T) {
  PolyPath1D spath = JMT_single_channel(si, spi, sppi, sf, spf, sppf, T);
  PolyPath1D dpath = JMT_single_channel(di, dpi, dppi, df, dpf, dppf, T);
  return PolyTrajectory({spath, dpath});
}
