#ifndef SRC_JMT_H_
#define SRC_JMT_H_

#include <iostream>
#include <vector>
#include "./vehicle.h"

class JMT {
 public:
  static std::vector<double> get_jmt(const VehicleState& start,
                                     const VehicleState& end, const double t);

  static double eval_at_time(const std::vector<double>& jmt_coeffs,
                             const double t);
};

#endif  // SRC_JMT_H_
