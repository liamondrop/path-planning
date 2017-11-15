#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <iostream>
#include <vector>

#include "./constants.h"
#include "./vehicle.h"

class BehaviorPlanner {
 public:
  BehaviorPlanner();
  Behavior update(Vehicle& my_vehicle,
                  const std::vector<Vehicle>& other_vehicles);

  double get_gap(const Vehicle& my_vehicle,
                 const std::vector<Vehicle>& other_vehicles, const Lane lane,
                 const double direction);

 private:
  double current_front_v;
  double current_front_s;

  double get_forward_cost(const double gap);

  double get_lane_change_cost(const double forward_gap,
                              const double rear_gap,
                              const Lane lane);
};

#endif  // BEHAVIOR_PLANNER_H_
