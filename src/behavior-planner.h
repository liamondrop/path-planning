#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <iostream>
#include <vector>

#include "./constants.h"
#include "./vehicle.h"

class BehaviorPlanner {
 public:
  BehaviorPlanner();
  Behavior update(Vehicle& my_vehicle, std::vector<Vehicle>& other_vehicles);
  double get_gap(const Vehicle& my_vehicle,
                 const std::vector<Vehicle>& other_vehicles,
                 const Lane lane_type, const double where);

 private:
  double current_front_v;
  double current_front_s;

  double get_cost(const double front_gap, const double back_gap,
                  const Lane lane) const;
  double get_cost(const double gap) const;
};

#endif  // BEHAVIOR_PLANNER_H_
