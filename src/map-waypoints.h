#ifndef SRC_PATH_CONVERTER_H_
#define SRC_PATH_CONVERTER_H_

#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../include/spline.h"
#include "./constants.h"
#include "./jmt.h"

struct MapPath {
  std::vector<double> X;
  std::vector<double> Y;
};

class MapWaypoints {
 private:
  double distance;
  tk::spline x_spline;
  tk::spline y_spline;
  tk::spline dx_spline;
  tk::spline dy_spline;

 public:
  MapWaypoints(std::string file_path);

  std::vector<double> convert_frenet_to_cartesian(const double s,
                                                  const double d) const;

  MapPath make_path(std::vector<double> jmt_s,
                    std::vector<double> jmt_d,
                    const double t,
                    const int start_index,
                    const int end_index) const;
};

#endif  // SRC_PATH_CONVERTER_H_
