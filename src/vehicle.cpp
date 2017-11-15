#include "./vehicle.h"
#include <vector>

#include "./jmt.h"

const double LEFT_d = 2.1;
const double MID_d = 6.0;
const double RIGHT_d = 9.9;

Vehicle::Vehicle(const double s, const double d, const double v)
    : s(s), d(d), v(v) {
  current_lane = convert_d_to_lane(d);
  update_adjacent_lanes();
}

void Vehicle::update_adjacent_lanes() {
  switch (current_lane) {
    case Lane::LEFT:
      lane_at_left = Lane::OUT_OF_BOUNDS;
      lane_at_right = Lane::MID;
      break;
    case Lane::MID:
      lane_at_left = Lane::LEFT;
      lane_at_right = Lane::RIGHT;
      break;
    case Lane::RIGHT:
      lane_at_left = Lane::MID;
      lane_at_right = Lane::OUT_OF_BOUNDS;
      break;
    default:
      lane_at_left = Lane::OUT_OF_BOUNDS;
      lane_at_right = Lane::OUT_OF_BOUNDS;
  }
}

void Vehicle::update_states(const VehicleState& state_s,
                            const VehicleState& state_d) {
  saved_state_s = state_s;
  saved_state_s.p = fmod(saved_state_s.p, TRACK_DISTANCE);
  saved_state_d = state_d;
}

void Vehicle::realize_behavior(const Behavior behavior) {
  // get target states based on behavior s component
  double target_s = saved_state_s.p + TIME_HORIZON * saved_state_s.v;
  double target_v = saved_state_s.v;

  if (behavior == Behavior::KEEP_LANE) {
    if (front_gap > FRONT_BUFFER) {
      target_v = SPEED_LIMIT;
    } else {
      target_v = fmax(MINIMUM_SPEED, fmin(SPEED_LIMIT, front_v - SPEED_BUFFER));
    }

    // Estimate a safe target distance based on our selected speed
    target_s =
        saved_state_s.p + TIME_HORIZON * 0.5 * (saved_state_s.v + target_v);
  }

  VehicleState target_state_s = {target_s, target_v, 0.0};
  VehicleState target_state_d = {get_target_d(behavior), 0.0, 0.0};

  // generate JMTs
  s_trajectory = JMT::get_jmt(saved_state_s, target_state_s, TIME_HORIZON);
  d_trajectory = JMT::get_jmt(saved_state_d, target_state_d, TIME_HORIZON);

  // save target states
  update_states(target_state_s, target_state_d);
}

std::vector<double> Vehicle::get_s_trajectory() { return s_trajectory; }

std::vector<double> Vehicle::get_d_trajectory() { return d_trajectory; }

Lane Vehicle::convert_d_to_lane(const double d) {
  if (d >= 0.0 && d < 4.0) return Lane::LEFT;
  if (d >= 4.0 && d < 8.0) return Lane::MID;
  if (d >= 8.0 && d < 12.0) return Lane::RIGHT;
  return Lane::OUT_OF_BOUNDS;
}

double Vehicle::convert_lane_to_d(const Lane lane_to_convert) {
  switch (lane_to_convert) {
    case Lane::LEFT:
      return LEFT_d;
    case Lane::RIGHT:
      return RIGHT_d;
    default:
      return MID_d;
  }
}

double Vehicle::get_target_d(const Behavior behavior) {
  switch (behavior) {
    case Behavior::TURN_RIGHT:
      return convert_lane_to_d(lane_at_right);
    case Behavior::TURN_LEFT:
      return convert_lane_to_d(lane_at_left);
    default:
      return convert_lane_to_d(current_lane);
  }
}
