//
// Created by tsbertalan on 7/2/2018.
//

#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

PolyPath1D::PolyPath1D(std::vector<double> coefficients)
    : coefficients(coefficients) {
  double dt = .02;
}


double PolyPath1D::operator()(double x) {
  return derivative(x, 0);
}


double PolyPath1D::derivative(double x, int order) {
  // Evaluate the polynomial the naive way, as a sum of powers.
  // Further, evaluate the derivative an even more naive way,
  // with an explicit "factorial" expression.
  // TODO: Surely this could be improved per Teukolsky & Press.

  double y = 0;

  for (int p = 0; p < coefficients.size(); p++) {
    // If we're doing a derivative, there is a premultiplier.
    double derivative_factor = 1;
    for (int k = 0; k < order; k++) {
      derivative_factor *= p - k;
      if (derivative_factor==0)
        break;
    }

    if (derivative_factor==0)
      continue;

    else {
      // Need to cast to double before power; otherwise iext version
      // of pow is used, which gives wrong results.
      y += derivative_factor
          *pow((double) x, (double) (p - order))
          *coefficients[p];
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

  // JMT is a quintic polynomial; it will have 6 coefficients.
  double a0, a1, a2, a3, a4, a5;

  // First three coefficients are known in closed form.
  a0 = yi;
  a1 = ydi;
  a2 = yddi*.5;

  // Remaining coefficients are result of a linear problem
  //     A * a345 = rhs
  // Construct right-hand-side per lecture.
  MatrixXd rhs(3, 1);
  rhs <<
      yf - (yi + ydi*T + yddi*.5*T*T),
      ydf - (ydi + yddi*T),
      yddf - yddi;

  // Precompute powers of time interval.
  double T2, T3, T4, T5;
  T2 = pow(T, 2);
  T3 = pow(T, 3);
  T4 = pow(T, 4);
  T5 = pow(T, 5);

  // Assemble matrix for inversion.
  MatrixXd A(3, 3);
  A <<
    T3, T4, T5,
      3*T2, 4*T3, 5*T4,
      6*T, 12*T2, 20*T3;

  // Solve for remaining coefficients.
  MatrixXd a345 = A.inverse()*rhs;
  a3 = a345(0);
  a4 = a345(1);
  a5 = a345(2);

  return PolyPath1D({a0, a1, a2, a3, a4, a5});
}

PolyTrajectory
JMT(
    double si, double spi, double sppi,
    double sf, double spf, double sppf,
    double di, double dpi, double dppi,
    double df, double dpf, double dppf,
    double T) {

  // This is basically just a dumb bucket for two single-dependent-variable polynomials.
  PolyPath1D spath = JMT_single_channel(si, spi, sppi, sf, spf, sppf, T);
  PolyPath1D dpath = JMT_single_channel(di, dpi, dppi, df, dpf, dppf, T);

  return PolyTrajectory({spath, dpath});
}
