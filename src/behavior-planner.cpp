#include "./behavior-planner.h"

using namespace std;

BehaviorPlanner::BehaviorPlanner() {}

Behavior BehaviorPlanner::update(Vehicle& myCar,
                                     std::vector<Vehicle>& otherCars) {
  myCar.front_gap = this->get_gap(myCar, otherCars, myCar.current_lane, FROM_FRONT);
  myCar.front_v = this->current_front_v;
  myCar.front_s = this->current_front_s;

  const double straight_cost = this->get_cost(myCar.front_gap);

  const double frontleft =
      this->get_gap(myCar, otherCars, myCar.lane_at_left, FROM_FRONT);
  const double backleft =
      this->get_gap(myCar, otherCars, myCar.lane_at_left, FROM_BACK);

  const double frontright =
      this->get_gap(myCar, otherCars, myCar.lane_at_right, FROM_FRONT);
  const double backright =
      this->get_gap(myCar, otherCars, myCar.lane_at_right, FROM_BACK);

  cout << "|" << endl;
  cout << "| LOOK LEFT..." << endl;
  const double left_cost = get_cost(frontleft, backleft, myCar.lane_at_left);

  cout << "|" << endl;
  cout << "| LOOK RIGHT..." << endl;
  const double right_cost =
      get_cost(frontright, backright, myCar.lane_at_right);

  cout << "|" << endl;
  cout << "| GAP FRONT - left | straight | right:" << endl;
  cout << "|           " << frontleft << " | " << myCar.front_gap << " | "
       << frontright << endl;

  cout << "|" << endl;
  cout << "| GAP BACK - left | right:" << endl;
  cout << "|          " << backleft << " | " << backright << endl;
  cout << "|" << endl;

  if (left_cost < straight_cost && left_cost < right_cost) {
    cout << "| DECISION: TURN LEFT. " << left_cost << " < " << straight_cost
         << endl;
    cout << "| \n"
         << "|" << endl;

    return Behavior::TURN_LEFT;
  }

  if (right_cost < straight_cost && right_cost < left_cost) {
    cout << "| DECISION: TURN RIGHT. " << right_cost << " < " << straight_cost
         << endl;
    cout << "| \n"
         << "|" << endl;

    return Behavior::TURN_RIGHT;
  }

  cout << "| DECISION: KEEP LANE." << endl;
  cout << "| COSTS - left | straight | right:" << endl;
  cout << "|        " << left_cost << " | " << straight_cost << " | "
       << right_cost << endl;

  return Behavior::KEEP_LANE;
}

double BehaviorPlanner::get_cost(const double front_gap, const double back_gap,
                                 const Lane lane) const {
  double cost = (FRONT_GAP_FACTOR / front_gap + BACK_GAP_FACTOR / back_gap);

  if (lane == Lane::NONE || lane == Lane::UNSPECIFIED) {
    cout << "|... No lane. \n"
         << "|" << endl;
    return REALLY_BIG_NUMBER;
  }

  if (front_gap < FRONT_GAP_THRESH || back_gap < BACK_GAP_THRESH) {
    cout << "|... Insufficient space to turn!" << endl;
    cout << "|... GAP - front: " << front_gap << " back: " << back_gap << endl;

    return REALLY_BIG_NUMBER;

  } else {
    cout << "| \n"
         << "|" << endl;
  }

  // We don't want to turn at every opportunity back and forth
  cost = cost * TURN_PENALTY_FACTOR;

  if (lane == Lane::MID) {
    // we want to reward going in the middle
    cost = cost * MIDLANE_REWARD_FACTOR;
  }

  return cost;
}

double BehaviorPlanner::get_cost(const double gap) const {
  if (gap < FRONT_GAP_THRESH) {
    cout << "|... WARNING: Too near the front vehicle!" << endl;
    cout << "| We must turn! gap: " << gap << endl;

    return REALLY_BIG_NUMBER;

  } else {
    cout << "| \n"
         << "|" << endl;
  }

  return FRONT_GAP_FACTOR / gap;
}

double BehaviorPlanner::get_gap(const Vehicle& myCar,
                                const std::vector<Vehicle>& otherCars,
                                const Lane lane, const double where) {
  if (lane == Lane::NONE || lane == Lane::UNSPECIFIED) {
    return 0.0001;
  }

  double smallest_gap = REALLY_BIG_NUMBER;

  for (auto& otherCar : otherCars) {
    double gap = (otherCar.s - myCar.s) * where;

    if (otherCar.current_lane == lane && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
      this->current_front_v = otherCar.v;
      this->current_front_s = otherCar.s;
    }
  }

  return smallest_gap;
}
