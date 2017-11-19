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

struct Point {
  double x;
  double y;
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

  Point convert_frenet_to_cartesian(const double s, const double d) const;

  MapPath generate_path(std::vector<double> s_trajectory,
                        std::vector<double> d_trajectory,
                        const double time_increment, const int start_index,
                        const int end_index) const;
};

#endif  // SRC_PATH_CONVERTER_H_
