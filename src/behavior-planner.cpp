#include "./behavior-planner.h"
#include <vector>

BehaviorPlanner::BehaviorPlanner() {}

Behavior BehaviorPlanner::update(Vehicle& my_vehicle,
                                 const std::vector<Vehicle>& other_vehicles) {
  my_vehicle.front_gap = get_gap(my_vehicle, other_vehicles,
                                 my_vehicle.current_lane, DIRECTION_FORWARD);
  my_vehicle.front_v = current_front_v;
  my_vehicle.front_s = current_front_s;

  const double straight_cost = get_forward_cost(my_vehicle.front_gap);

  const double forward_left_space = get_gap(
      my_vehicle, other_vehicles, my_vehicle.lane_at_left, DIRECTION_FORWARD);
  const double rear_left_space = get_gap(
      my_vehicle, other_vehicles, my_vehicle.lane_at_left, DIRECTION_REAR);

  const double forward_right_space = get_gap(
      my_vehicle, other_vehicles, my_vehicle.lane_at_right, DIRECTION_FORWARD);
  const double rear_right_space = get_gap(
      my_vehicle, other_vehicles, my_vehicle.lane_at_right, DIRECTION_REAR);

  const double left_turn_cost = get_lane_change_cost(
      forward_left_space, rear_left_space, my_vehicle.lane_at_left);
  const double right_turn_cost = get_lane_change_cost(
      forward_right_space, rear_right_space, my_vehicle.lane_at_right);

  std::cout << "---------------------------------" << std::endl;
  std::cout << "FORWARD LEFT SPACE:  " << forward_left_space << std::endl;
  std::cout << "FORWARD SPACE:       " << my_vehicle.front_gap << std::endl;
  std::cout << "FORWARD RIGHT SPACE: " << forward_right_space << std::endl;
  std::cout << "---------------------------------" << std::endl;
  std::cout << "REAR LEFT SPACE:     " << rear_left_space << std::endl;
  std::cout << "REAR RIGHT SPACE:    " << rear_right_space << std::endl;
  std::cout << "---------------------------------" << std::endl;
  std::cout << "LEFT TURN COST:      " << left_turn_cost << std::endl;
  std::cout << "STRAIGHT COST:       " << straight_cost << std::endl;
  std::cout << "RIGHT TURN COST:     " << right_turn_cost << std::endl;
  std::cout << "---------------------------------" << std::endl;

  if (left_turn_cost < straight_cost && left_turn_cost < right_turn_cost) {
    std::cout << "==> DECISION: TURN LEFT." << std::endl;
    return Behavior::TURN_LEFT;
  }

  if (right_turn_cost < straight_cost && right_turn_cost < left_turn_cost) {
    std::cout << "==> DECISION: TURN RIGHT." << std::endl;
    return Behavior::TURN_RIGHT;
  }

  std::cout << "==> DECISION: KEEP LANE." << std::endl;
  return Behavior::KEEP_LANE;
}

double BehaviorPlanner::get_lane_change_cost(const double forward_gap,
                                             const double rear_gap,
                                             const Lane lane) {
  if (lane == Lane::OUT_OF_BOUNDS) {
    return LARGE_VALUE;
  }

  if (forward_gap < FORWARD_GAP_THRESHOLD || rear_gap < REAR_GAP_THRESHOLD) {
    std::cout << "... Insufficient space to turn!" << std::endl;
    return LARGE_VALUE;
  }

  // A lane change should come with some inherent cost
  double cost = CHANGE_LANE_WEIGHT;

  // Apply weights to the amount of space in the adjacent lane
  cost *= (FORWARD_GAP_WEIGHT / forward_gap + REAR_GAP_WEIGHT / rear_gap);

  // All else being equal, prefer the middle lane
  if (lane == Lane::MID) {
    cost *= PREFER_MID_LANE_WEIGHT;
  }

  return cost;
}

double BehaviorPlanner::get_forward_cost(const double gap) {
  if (gap < FORWARD_GAP_THRESHOLD) {
    std::cout << "... WARNING: Too near the front vehicle!" << std::endl;

    return LARGE_VALUE;
  }

  return FORWARD_GAP_WEIGHT / gap;
}

double BehaviorPlanner::get_gap(const Vehicle& my_vehicle,
                                const std::vector<Vehicle>& other_vehicles,
                                const Lane lane, const double direction) {
  if (lane == Lane::OUT_OF_BOUNDS) {
    return 0.0;
  }

  double smallest_gap = LARGE_VALUE;

  for (auto& other_vehicle : other_vehicles) {
    double gap = direction * (other_vehicle.s - my_vehicle.s);

    if (other_vehicle.current_lane == lane && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
      current_front_v = other_vehicle.v;
      current_front_s = other_vehicle.s;
    }
  }

  return smallest_gap;
}
