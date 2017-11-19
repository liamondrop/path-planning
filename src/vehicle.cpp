#include "./vehicle.h"
#include <vector>

#include "./jmt.h"

const double LEFT_d = 2.2;
const double MID_d = 6.0;
const double RIGHT_d = 9.8;

Vehicle::Vehicle(const double s, const double d, const double v)
    : s(s), d(d), v(v) {
  current_lane = convert_d_to_lane(d);
  update_adjacent_lanes();
}

void Vehicle::set_goal_states(const VehicleState& new_state_s,
                              const VehicleState& new_state_d) {
  state_s = new_state_s;
  state_s.p = fmod(state_s.p, TRACK_LENGTH);
  state_d = new_state_d;
}

void Vehicle::realize_behavior(const Behavior& behavior,
                               const Vehicle& forward_vehicle,
                               const double time_horizon) {
  // get target states based on behavior s component
  double target_s = state_s.p + time_horizon * state_s.v;
  double target_v = state_s.v;
  const double forward_gap = (forward_vehicle.s - s);

  if (behavior == Behavior::KEEP_LANE) {
    if (0 > forward_gap || forward_gap > FRONT_BUFFER) {
      target_v = SPEED_LIMIT;
    } else {
      target_v = fmin(SPEED_LIMIT, forward_vehicle.v - SPEED_BUFFER);
      target_v = fmax(MINIMUM_SPEED, target_v);
    }

    // Estimate a safe target distance based on our selected speed
    target_s = state_s.p + time_horizon * 0.5 * (state_s.v + target_v);
  }

  VehicleState target_state_s = {target_s, target_v, 0.0};
  VehicleState target_state_d = {get_target_d(behavior), 0.0, 0.0};

  // generate JMTs
  s_trajectory = JMT::get_jmt(state_s, target_state_s, time_horizon);
  d_trajectory = JMT::get_jmt(state_d, target_state_d, time_horizon);

  // save target states
  set_goal_states(target_state_s, target_state_d);
}

Lane Vehicle::convert_d_to_lane(const double d) {
  if (d >= 0.0 && d < 4.0) return Lane::LEFT;
  if (d >= 4.0 && d < 8.0) return Lane::MID;
  if (d >= 8.0 && d < 12.0) return Lane::RIGHT;
  return Lane::OUT_OF_BOUNDS;
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
