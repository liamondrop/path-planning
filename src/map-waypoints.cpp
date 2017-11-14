#include "./map-waypoints.h"
#include <string>
#include <vector>
#include "./constants.h"
#include "./jmt.h"

MapWaypoints::MapWaypoints(std::string file_path) {
  std::ifstream in_file(file_path.c_str(), std::ifstream::in);

  if (!in_file.is_open()) {
    std::cerr << "Error: Cannot open input file: " << file_path << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string line;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> ss;
  std::vector<double> dxs;
  std::vector<double> dys;

  double x, y, s, dx, dy;

  // Load information from file to memory
  while (getline(in_file, line)) {
    std::istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    xs.push_back(x);
    ys.push_back(y);
    ss.push_back(s);
    dxs.push_back(dx);
    dys.push_back(dy);
  }

  if (in_file.is_open()) {
    in_file.close();
  }

  xs.push_back(xs[0]);
  ys.push_back(ys[0]);
  ss.push_back(TRACK_DISTANCE);
  dxs.push_back(dxs[0]);
  dys.push_back(dys[0]);

  // Uses the loaded information to fit cubic polynomial curves
  x_spline.set_points(ss, xs);
  y_spline.set_points(ss, ys);
  dx_spline.set_points(ss, dxs);
  dy_spline.set_points(ss, dys);
}

std::vector<double> MapWaypoints::convert_sd_to_xy(const double s,
                                                   const double d) const {
  const double mod_s = fmod(s, TRACK_DISTANCE);
  const double x_edge = x_spline(mod_s);
  const double y_edge = y_spline(mod_s);
  const double dx = dx_spline(mod_s);
  const double dy = dy_spline(mod_s);

  const double x = x_edge + dx * d;
  const double y = y_edge + dy * d;

  return {x, y};
}

MapPath MapWaypoints::make_path(std::vector<double> jmt_s,
                                std::vector<double> jmt_d, const double dt,
                                const int n) const {
  MapPath map_path;

  for (int i = 0; i < n; ++i) {
    const double s = JMT::eval_at_time(jmt_s, i * dt);
    const double d = JMT::eval_at_time(jmt_d, i * dt);
    const auto point = convert_sd_to_xy(s, d);

    map_path.X.push_back(point[0]);
    map_path.Y.push_back(point[1]);
  }

  return map_path;
}
