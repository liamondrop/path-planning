#include "./behavior-planner.h"
#include <vector>

BehaviorPlanner::BehaviorPlanner() {}

Behavior BehaviorPlanner::update(Vehicle& my_vehicle, const std::vector<Vehicle>& other_vehicles) {
  my_vehicle.front_gap =
      get_gap(my_vehicle, other_vehicles, my_vehicle.current_lane, FROM_FRONT);
  my_vehicle.front_v = current_front_v;
  my_vehicle.front_s = current_front_s;

  const double straight_cost = get_cost(my_vehicle.front_gap);

  const double frontleft =
      get_gap(my_vehicle, other_vehicles, my_vehicle.lane_at_left, FROM_FRONT);
  const double backleft =
      get_gap(my_vehicle, other_vehicles, my_vehicle.lane_at_left, FROM_BACK);

  const double frontright =
      get_gap(my_vehicle, other_vehicles, my_vehicle.lane_at_right, FROM_FRONT);
  const double backright =
      get_gap(my_vehicle, other_vehicles, my_vehicle.lane_at_right, FROM_BACK);

  const double left_cost =
      get_cost(frontleft, backleft, my_vehicle.lane_at_left);
  const double right_cost =
      get_cost(frontright, backright, my_vehicle.lane_at_right);

  std::cout << "---------------------------------" << std::endl;
  std::cout << "FRONT LEFT GAP:  " << frontleft << std::endl;
  std::cout << "FRONT GAP:       " << my_vehicle.front_gap << std::endl;
  std::cout << "FRONT RIGHT GAP: " << frontright << std::endl;
  std::cout << "---------------------------------" << std::endl;
  std::cout << "REAR LEFT GAP:   " << backleft << std::endl;
  std::cout << "REAR RIGHT GAP:  " << backright << std::endl;
  std::cout << "---------------------------------" << std::endl;
  std::cout << "LEFT COST:       " << left_cost << std::endl;
  std::cout << "STRAIGHT COST:   " << straight_cost << std::endl;
  std::cout << "RIGHT COST:      " << straight_cost << std::endl;
  std::cout << "---------------------------------" << std::endl;

  if (left_cost < straight_cost && left_cost < right_cost) {
    std::cout << "==> DECISION: TURN LEFT." << std::endl;

    return Behavior::TURN_LEFT;
  }

  if (right_cost < straight_cost && right_cost < left_cost) {
    std::cout << "==> DECISION: TURN RIGHT." << std::endl;

    return Behavior::TURN_RIGHT;
  }

  std::cout << "==> DECISION: KEEP LANE." << std::endl;

  return Behavior::KEEP_LANE;
}

double BehaviorPlanner::get_cost(const double front_gap, const double back_gap,
                                 const Lane lane) const {
  double cost = (FRONT_GAP_FACTOR / front_gap + BACK_GAP_FACTOR / back_gap);

  if (lane == Lane::NONE || lane == Lane::UNSPECIFIED) {
    return REALLY_BIG_NUMBER;
  }

  if (front_gap < FRONT_GAP_THRESH || back_gap < BACK_GAP_THRESH) {
    std::cout << "... Insufficient space to turn!" << std::endl;

    return REALLY_BIG_NUMBER;
  }

  // We don't want to turn at every opportunity back and forth
  cost = cost * TURN_PENALTY_FACTOR;

  if (lane == Lane::MID) {
    cost = cost * MIDLANE_REWARD_FACTOR;
  }

  return cost;
}

double BehaviorPlanner::get_cost(const double gap) const {
  if (gap < FRONT_GAP_THRESH) {
    std::cout << "... WARNING: Too near the front vehicle!" << std::endl;
    std::cout << "We must turn! gap: " << gap << std::endl;

    return REALLY_BIG_NUMBER;
  }

  return FRONT_GAP_FACTOR / gap;
}

double BehaviorPlanner::get_gap(const Vehicle& my_vehicle,
                                const std::vector<Vehicle>& other_vehicles,
                                const Lane lane, const double where) {
  if (lane == Lane::NONE || lane == Lane::UNSPECIFIED) {
    return 0.0001;
  }

  double smallest_gap = REALLY_BIG_NUMBER;

  for (auto& other_vehicle : other_vehicles) {
    double gap = (other_vehicle.s - my_vehicle.s) * where;

    if (other_vehicle.current_lane == lane && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
      current_front_v = other_vehicle.v;
      current_front_s = other_vehicle.s;
    }
  }

  return smallest_gap;
}
