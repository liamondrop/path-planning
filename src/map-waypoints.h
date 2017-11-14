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

/**
PathConverter contains the map of the highway. Use it to
convert Frenet Coordinates (s, d) to map coordinates (x, y) or to
convert Jerk Minimized Trajectories (JMT) to a path plan (XYPath) of (x, y)
points on the map
**/

/**
map file convention stored at file path:
Each row of the file contains x y s dx dy values which are the waypoints.
The x and y are the waypoint's map coordinate position.
The s value is the distance along the road to get to that waypoint.
The dx and dy values define the unit normal vector pointing outward of the
highway loop.
 **/

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
  /**
   * file_path - a file path containing map information
   * distance - the total road distance of the map
   **/
  MapWaypoints(std::string file_path);

  /**
   * Takes in (s, d) coordinates in the frenet frame (which is along the loop of
   * the road) Returns a vector (x, y) which are cartesian coordinates in the
   * fixed map frame
   **/
  std::vector<double> convert_sd_to_xy(const double s, const double d) const;

  MapPath make_path(std::vector<double> jmt_s, std::vector<double> jmt_d,
                    const double t, const int n) const;
};

#endif  // SRC_PATH_CONVERTER_H_
