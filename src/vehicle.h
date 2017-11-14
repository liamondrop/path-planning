#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_

#include <math.h>
#include <iostream>
#include <vector>
#include "./constants.h"

/**
 * store position, velocity, and acceleration in the s or d axis
 **/
struct VehicleState {
  double p;
  double v;
  double a;
};

class Vehicle {
 private:
  void update_adjacent_lanes();
  Lane convert_d_to_lane(const double d);
  double convert_lane_to_d(const Lane lane_to_convert);
  double get_target_d(const Behavior behavior);
  std::vector<double> s_trajectory;
  std::vector<double> d_trajectory;

 public:
  double s;
  double d;
  double v;
  double front_gap;
  double front_v;
  double front_s;

  Vehicle(const double s, const double d, const double v);
  void update_states(const VehicleState& state_s, const VehicleState& state_d);
  void set_next_state(const Behavior bevaior);
  void realize_behavior(const Behavior behavior);
  std::vector<double> get_s_trajectory();
  std::vector<double> get_d_trajectory();

  VehicleState saved_state_s;
  VehicleState saved_state_d;
  Lane current_lane;
  Lane lane_at_right;
  Lane lane_at_left;
};

#endif  // SRC_VEHICLE_H_
