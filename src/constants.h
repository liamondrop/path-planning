#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// Here's the duration period for each path plan we send to the controller
const double TIME_INCREMENT = 0.02;
const double TIME_HORIZON = 2.0;
const int TIME_STEPS = static_cast<int>(TIME_HORIZON / TIME_INCREMENT);

const double REALLY_BIG_NUMBER = 2e32;

// how much points left for the controller to perform
// before we start planning again
const int PATH_SIZE_CUTOFF = 10;

// used as parameter in BehaviorPlanner::get_gap()
const double FROM_FRONT = 1.0;
const double FROM_BACK = -1.0;

// Total road distance of the the highway loop
const double TRACK_DISTANCE = 6945.554;

// boundaries of acceptable speed of our vehicle
const double HARD_SPEED_LIMIT = 22.352;  // 50mph in m/s
const double SPEED_LIMIT = 20.75;
const double MIN_SPEED = 10.0;

// if the gap is less than this we consider it unsafe to turn
const double FRONT_GAP_THRESH = 20.0;
const double BACK_GAP_THRESH = 15.0;

// This is the buffers we want against the leading front vehicle
// for safety so we don't collide with the vehicle right in front of us
const double FRONT_BUFFER = FRONT_GAP_THRESH + 5.0;
const double SPEED_BUFFER = 2.0;

// Parameters than can be tweaked which affects the cost of each behavior
const double MIDLANE_REWARD_FACTOR = 0.35;  // must be 0 < x < 1
const double BACK_GAP_FACTOR = 0.4;  // must be less than FRONT_GAP_FACTOR
const double FRONT_GAP_FACTOR = 1.0;
const double TURN_PENALTY_FACTOR = 1.4;  // must be x > 1

typedef enum { LEFT, MID, RIGHT, NONE, UNSPECIFIED } Lane;
typedef enum { KEEP_LANE, TURN_RIGHT, TURN_LEFT } Behavior;

#endif  // CONSTANTS_H_
