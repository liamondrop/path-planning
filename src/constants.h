#ifndef CONSTANTS_H_
#define CONSTANTS_H_

const double LARGE_VALUE = 2e+32;

// The length of the path to send to the controller
const double TIME_INCREMENT = 0.02;
const double PLANNING_HORIZON = 2.0;
const int TIME_STEPS = static_cast<int>(PLANNING_HORIZON / TIME_INCREMENT);

// The point at which to update and extend the current planned trajectory
const int PATH_SIZE_THRESHOLD = 10;

// Used to determine whether to look for vehicles ahead or vehicles behind
const double DIRECTION_FORWARD = 1.0;
const double DIRECTION_REAR = -1.0;

// Total road distance of the the highway loop
const double TRACK_DISTANCE = 6945.554;

// boundaries of acceptable speed of our vehicle
const double SPEED_LIMIT = 20.75;  // speed in meters/s
const double MINIMUM_SPEED = 10.0;

// if the gap is less than this, consider it unsafe to turn
const double FORWARD_GAP_THRESHOLD = 25.0;
const double REAR_GAP_THRESHOLD = 15.0;

// Guard against colliding with the vehicle in front
const double FRONT_BUFFER = FORWARD_GAP_THRESHOLD + 5.0;
const double SPEED_BUFFER = 1.0;

// Weights for the behavior planner
const double PREFER_MID_LANE_WEIGHT = 0.5;
const double REAR_GAP_WEIGHT = 0.5;
const double FORWARD_GAP_WEIGHT = 1.0;
const double LANE_CHANGE_WEIGHT = 1.6;

typedef enum { LEFT, MID, RIGHT, OUT_OF_BOUNDS } Lane;
typedef enum { KEEP_LANE, TURN_RIGHT, TURN_LEFT } Behavior;

#endif  // CONSTANTS_H_
