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
  std::vector<double> X;
  std::vector<double> Y;
  std::vector<double> S;
  std::vector<double> dX;
  std::vector<double> dY;

  double x, y, s, dx, dy;

  // Load information from file to memory
  while (getline(in_file, line)) {
    std::istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    X.push_back(x);
    Y.push_back(y);
    S.push_back(s);
    dX.push_back(dx);
    dY.push_back(dy);
  }

  if (in_file.is_open()) {
    in_file.close();
  }

  X.push_back(X[0]);
  Y.push_back(X[0]);
  S.push_back(TRACK_DISTANCE);
  dX.push_back(dX[0]);
  dY.push_back(dY[0]);

  // Uses the loaded information to fit cubic polynomial curves
  x_spline.set_points(S, X);
  y_spline.set_points(S, Y);
  dx_spline.set_points(S, dX);
  dy_spline.set_points(S, dY);
}

std::vector<double> MapWaypoints::convert_frenet_to_cartesian(
    const double s, const double d) const {
  const double mod_s = fmod(s, TRACK_DISTANCE);
  const double dx = dx_spline(mod_s);
  const double dy = dy_spline(mod_s);

  const double x = x_spline(mod_s) + dx * d;
  const double y = y_spline(mod_s) + dy * d;

  return {x, y};
}

MapPath MapWaypoints::make_path(std::vector<double> jmt_s,
                                std::vector<double> jmt_d, const double dt,
                                const int start_index,
                                const int end_index) const {
  MapPath map_path;

  for (int i = start_index; i < end_index; ++i) {
    const double s = JMT::eval_at_time(jmt_s, i * dt);
    const double d = JMT::eval_at_time(jmt_d, i * dt);
    const auto point = convert_frenet_to_cartesian(s, d);

    map_path.X.push_back(point[0]);
    map_path.Y.push_back(point[1]);
  }

  return map_path;
}
