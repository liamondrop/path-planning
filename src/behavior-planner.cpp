#include "./behavior-planner.h"
#include <map>
#include <string>
#include <vector>

// Used to determine whether to look for vehicles ahead or vehicles behind
const double DIRECTION_FORWARD = 1.0;
const double DIRECTION_REAR = -1.0;

std::map<Lane, const std::string> LaneNames = {
    {Lane::LEFT, "Left"}, {Lane::MID, "Middle"}, {Lane::RIGHT, "Right"},
};

BehaviorPlanner::BehaviorPlanner(const Vehicle& my_vehicle,
                                 const std::vector<Vehicle>& other_vehicles)
    : my_vehicle(my_vehicle), other_vehicles(other_vehicles) {}

Behavior BehaviorPlanner::get_behavior(const double forward_gap) {
  const double straight_cost = get_keep_lane_cost(forward_gap);
  const double right_turn_cost = get_lane_change_cost(my_vehicle.lane_at_right);
  const double left_turn_cost = get_lane_change_cost(my_vehicle.lane_at_left);

  std::cout << "---------------------------------" << std::endl;
  std::cout << "LEFT TURN COST:  " << left_turn_cost << std::endl;
  std::cout << "STRAIGHT COST:   " << straight_cost << std::endl;
  std::cout << "RIGHT TURN COST: " << right_turn_cost << std::endl;
  std::cout << "=================================" << std::endl;

  if (left_turn_cost < straight_cost && left_turn_cost < right_turn_cost) {
    std::cout << "==> DECISION: TURN LEFT" << std::endl;
    return Behavior::TURN_LEFT;
  }

  if (right_turn_cost < straight_cost && right_turn_cost < left_turn_cost) {
    std::cout << "==> DECISION: TURN RIGHT" << std::endl;
    return Behavior::TURN_RIGHT;
  }

  std::cout << "==> DECISION: KEEP LANE" << std::endl;
  return Behavior::KEEP_LANE;
}

double BehaviorPlanner::get_lane_change_cost(const Lane lane) {
  if (lane == Lane::OUT_OF_BOUNDS) return LARGE_VALUE;

  const double forward_gap = get_gap(lane, DIRECTION_FORWARD);
  const double rear_gap = get_gap(lane, DIRECTION_REAR);

  if (forward_gap < FORWARD_GAP_THRESHOLD || rear_gap < REAR_GAP_THRESHOLD) {
    std::cout << "... Nearby vehicle in the " << LaneNames[lane] << " lane!"
              << std::endl;
    return LARGE_VALUE;
  }

  // Apply weights to the amount of space in the adjacent lane
  double cost = (FORWARD_GAP_WEIGHT / forward_gap + REAR_GAP_WEIGHT / rear_gap);

  // All else being equal, prefer the middle lane
  if (lane == Lane::MID) {
    cost *= PREFER_MID_LANE_WEIGHT;
  }

  // Any lane change should come with some intrinsic cost
  return cost * LANE_CHANGE_WEIGHT;
}

double BehaviorPlanner::get_keep_lane_cost(const double gap) {
  if (gap < FORWARD_GAP_THRESHOLD) {
    std::cout << "... Too near the forward vehicle!" << std::endl;
    return LARGE_VALUE;
  }

  return FORWARD_GAP_WEIGHT / gap;
}

double BehaviorPlanner::get_gap(const Lane lane, const double direction) {
  if (lane == Lane::OUT_OF_BOUNDS) return 0.0;

  double smallest_gap = LARGE_VALUE;
  for (Vehicle other_vehicle : other_vehicles) {
    double gap = direction * (other_vehicle.s - my_vehicle.s);

    if (other_vehicle.current_lane == lane && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
    }
  }

  return smallest_gap;
}
