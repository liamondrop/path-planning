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
  VehicleState state_s;
  VehicleState state_d;

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
  Lane current_lane;
  Lane lane_at_right;
  Lane lane_at_left;

  Vehicle(const double s, const double d, const double v);
  virtual ~Vehicle();

  void update_states(const VehicleState& new_state_s,
                     const VehicleState& new_state_d);

  void realize_behavior(const Behavior& behavior,
                        const Vehicle& forward_vehicle,
                        const double time_horizon);

  std::vector<double> get_s_trajectory() { return s_trajectory; }
  std::vector<double> get_d_trajectory() { return d_trajectory; }
};

#endif  // SRC_VEHICLE_H_
