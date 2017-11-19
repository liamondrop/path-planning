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
  std::vector<double> DX;
  std::vector<double> DY;

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
    DX.push_back(dx);
    DY.push_back(dy);
  }

  if (in_file.is_open()) {
    in_file.close();
  }

  // push the first waypoints to the end of the coordinate vectors to ensure a
  // smooth transition when completing the loop
  X.push_back(X[0]);
  Y.push_back(Y[0]);
  S.push_back(TRACK_LENGTH);
  DX.push_back(DX[0]);
  DY.push_back(DY[0]);

  // calculate the mappings from s (distance along the length of the track) to
  // other x, y, and d values of the waypoints
  x_spline.set_points(S, X);
  y_spline.set_points(S, Y);
  dx_spline.set_points(S, DX);
  dy_spline.set_points(S, DY);
}

Point MapWaypoints::convert_frenet_to_cartesian(const double s,
                                                const double d) const {
  // handle when the s value exceeds the length of the track
  const double mod_s = fmod(s, TRACK_LENGTH);

  // Evaluate x, y, dx, and dy at position s.
  // Then multiply the (x,y) components of the unit d vector by the desired
  // offset (d) and add to the (x,y) position of the center line to get the
  // mapping from frenet (s,d) to cartesian (x,y) coordinates
  const double dx = dx_spline(mod_s);
  const double dy = dy_spline(mod_s);
  const double x = x_spline(mod_s) + dx * d;
  const double y = y_spline(mod_s) + dy * d;

  return {x, y};
}

MapPath MapWaypoints::generate_path(std::vector<double> s_trajectory,
                                    std::vector<double> d_trajectory,
                                    const double time_increment,
                                    const int start_index,
                                    const int end_index) const {
  MapPath map_path;

  for (int i = start_index; i < end_index; ++i) {
    const double s = JMT::eval_at_time(s_trajectory, i * time_increment);
    const double d = JMT::eval_at_time(d_trajectory, i * time_increment);
    const Point point = convert_frenet_to_cartesian(s, d);

    map_path.X.push_back(point.x);
    map_path.Y.push_back(point.y);
  }

  return map_path;
}
