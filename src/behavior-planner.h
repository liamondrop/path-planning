#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <iostream>
#include <vector>

#include "./constants.h"
#include "./vehicle.h"

class BehaviorPlanner {
 private:
  static double get_gap(const Vehicle& my_vehicle,
                        const std::vector<Vehicle>& other_vehicles,
                        const Lane lane, const double direction);

  static double get_forward_cost(const double gap);

  static double get_lane_change_cost(const double forward_gap,
                                     const double rear_gap,
                                     const Lane lane);

 public:
  static Behavior update(const Vehicle& my_vehicle,
                         const Vehicle& forward_vehicle,
                         const std::vector<Vehicle>& other_vehicles);
};

#endif  // BEHAVIOR_PLANNER_H_
