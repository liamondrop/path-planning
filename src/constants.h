#ifndef CONSTANTS_H_
#define CONSTANTS_H_

const double LARGE_VALUE = 2e+32;

// The length of the path to send to the controller
const double TIME_INCREMENT = 0.02;
const double TIME_HORIZON = 2.0;
const int TIME_STEPS = static_cast<int>(TIME_HORIZON / TIME_INCREMENT);

const int PATH_SIZE_CUTOFF = 10;
const int PREVIOUS_PATH_POINTS_TO_KEEP = 25;

// used as parameter in BehaviorPlanner::get_gap()
const double DIRECTION_FORWARD = 1.0;
const double DIRECTION_REAR = -1.0;

// Total road distance of the the highway loop
const double TRACK_DISTANCE = 6945.554;

// boundaries of acceptable speed of our vehicle
const double SPEED_LIMIT = 20.75;  // speed in meters/s
const double MINIMUM_SPEED = 10.0;

// if the gap is less than this we consider it unsafe to turn
const double FORWARD_GAP_THRESHOLD = 20.0;
const double REAR_GAP_THRESHOLD = 15.0;

// Guard against colliding with the vehicle in front
const double FRONT_BUFFER = FORWARD_GAP_THRESHOLD + 5.0;
const double SPEED_BUFFER = 1.0;

// Weights for the behavior planner
const double PREFER_MID_LANE_WEIGHT = 0.25;
const double REAR_GAP_WEIGHT = 0.5;
const double FORWARD_GAP_WEIGHT = 1.0;
const double LANE_CHANGE_WEIGHT = 1.4;

typedef enum { LEFT, MID, RIGHT, OUT_OF_BOUNDS } Lane;
typedef enum { KEEP_LANE, TURN_RIGHT, TURN_LEFT } Behavior;

#endif  // CONSTANTS_H_
