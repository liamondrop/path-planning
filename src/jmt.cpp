#include "./jmt.h"
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"

std::vector<double> JMT::get_jmt(const VehicleState& start,
                                 const VehicleState& end, const double t) {
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd(3);
  Eigen::VectorXd x = Eigen::VectorXd(3);

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

  A << t3, t4, t5,
       3 * t2, 4 * t3, 5 * t4,
       6 * t, 12 * t2, 20 * t3;

  b << end.p - (start.p + start.v * t + 0.5 * start.a * t2),
       end.v - (start.v + start.a * t),
       end.a - start.a;

  x = A.inverse() * b;

  return {start.p, start.v, start.a / 2.0, x[0], x[1], x[2]};
}

double JMT::eval_at_time(const std::vector<double>& jmt_coeffs,
                         const double t) {
  double total = 0.0;

  for (int i = 0; i < jmt_coeffs.size(); ++i) {
    total += jmt_coeffs[i] * pow(t, i);
  }

  return total;
}
