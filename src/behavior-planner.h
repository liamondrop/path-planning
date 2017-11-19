#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <iostream>
#include <vector>

#include "./constants.h"
#include "./vehicle.h"

class BehaviorPlanner {
 private:
  Vehicle my_vehicle;
  std::vector<Vehicle> other_vehicles;

  double get_keep_lane_cost(const double gap);
  double get_lane_change_cost(const Lane lane);
  double get_gap(const Lane lane, const double direction);

 public:
  BehaviorPlanner(const Vehicle& my_vehicle,
                  const std::vector<Vehicle>& other_vehicles);
  virtual ~BehaviorPlanner() {}

  Behavior update(const double forward_gap);
};

#endif  // BEHAVIOR_PLANNER_H_
