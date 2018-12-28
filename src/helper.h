#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <iostream>

using CppAD::AD;

constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
inline Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

inline Eigen::VectorXd derivative(Eigen::VectorXd coeffs) {
  // Assume we have a polynomial of the form:
  //    c0 + c1 * x + c2 * x^2 + c3 * x^3 + ... + cN * x^N
  // The derivative is in the form:
  //    c1 + 2 * c2 * x + 3 * c3 * x^2 + ... + N * cN * x^N-1
  // If we re-write this as:
  //    p0 + p1 * x + p2 * x^2 +  ... + pN-1 * x^N-1
  // we can see that the new coefficients are:
  //    p0 = c1
  //    p1 = c2 * 2
  //    p2 = c3 * 3
  //    ...
  //    pN-1 = cN * N
  Eigen::VectorXd coeffs_p(coeffs.size() - 1);
  for (int i = 0; i < coeffs_p.size(); ++i)
    coeffs_p[i] = coeffs[i+1] * (i+1);

  return coeffs_p;
}

// Evaluate a polynomial.
inline double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
inline AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

#endif /* HELPER_H */
