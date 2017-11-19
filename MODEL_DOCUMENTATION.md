# Model Documentation

## Summary

The path planner defined in this repository undertakes the following steps to safely navigate around a virtual highway alongside other traffic:

  1. It processes an initial list of map waypoints to be used to localize the vehicle with respect to its position on the map.
  1. It produces an initial startup trajectory, which brings the vehicle safely and comfortably up to the desired speed limit.
  1. It observes its surroundings and determines an appropriate behavior to navigate the course safely and efficiently
  1. It generates a trajectory to fulfill the desired behavior within the desired safety and comfort parameters
  1. It converts the generated trajectory into a list of X and Y coordinates to be fed back to the vehicle controller

## Classes

The project is composed of a few helper classes, described below.

### Map Waypoints

When the path planning module is first started, the [Map Waypoints](src/map-waypoints.h) class is initiated by reading in a [csv file](data/highway_map.csv) consisting of 181 waypoints. These waypoints coarsely define the full path of the track, which is a loop.

Each waypoint is composed of an `(x,y)` map position, a Frenet `s` value (the distance along the direction of the road), and a Frenet `d` unit normal vector (split up into the x component, and the y component). The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. By fitting a cubic spline to the relationship between the distance traveled (`s`) and each other waypoint coordinate, we can precisely determine the `(x,y)` coordinates of the planned trajectories. The library used to fit the splines can be found at [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/).

### JMT

The [JMT](src/jmt.h) class provides a couple of static methods for calculating a ["Jerk Minimized Trajectory"](http://www.shadmehrlab.org/book/minimum_jerk/minimumjerk.htm), and evaluating the `s` and `d` coordinates of a trajectory at any point in time. We calculate the JMT by defining the starting and ending `VehicleState` and a time horizon. `VehicleState` in this case refers to a vehicle's position, its velocity, and its acceleration in some dimension. When JMTs have been calculated for both the `s` and `d` dimensions, we can find an optimal trajectory that allows us to smoothly travel from on point to the next.

### Behavior Planner

The [Behavior Planner](src/behavior-planner.h) class stores a reference to our vehicle and all of the other vehicles currently in range of our vehicle's sensors. It provides a method to decide whether to turn left, turn right or continue driving straight, based on the location of the vehicles and which decision will allow our vehicle to drive safely as close to the speed limit as possible.

### Vehicle

The [Vehicle](src/vehicle.h) class provides methods for storing a vehicle's state, implementing state changes prescribed by the Behavior Planner, as well as utility methods for translating the current lateral offset to a lane and vice versa and storing the lanes to the left and right of the vehicle's current lane.

## Procedure

### 1. Initiate Map Waypoints

The first step of the path planning procedure is to read in all of the map waypoints and save cubic spline functions for evaluating the vehicle at distance `s` and calculating each of the corresponding `x`, `y`, `dx`, and `dy` values. This will enable us to precisely determine the position of the vehicle at any time and determine its `(x,y)` coordinates to feed back to the controller.

### 2. Generate Startup Trajectory

Now that the waypoints have been saved, we generate an initial trajectory to transition the car from a standstill up to the desired speed limit. This mini routine is defined as `generate_initial_path` [here](src/main.cpp#L36). We simply define a start state, which is the vehicle's initial position, velocity and acceleration, and an end state, which is the desired position (40 meters ahead in this case), velocity (20.75 meters per second or slightly under 50mph), and acceleration (0, since acceleration should be flat once we reach the top of the curve).

### 3. Determine Subsequent Behavior

Now that the vehicle is moving, we need to determine subsequent behavior as it navigates around the track. Once the vehicle has some margin of path points left in its previous planned trajectory, defined in the [constants](src/constants.h) file as `PATH_SIZE_THRESHOLD`, it's time to determine the next move. We do this by storing our vehicle's current state as well as the states of all the vehicles currently detected on the road and initiating an instance of `BehaviorPlanner`. With this data, the Behavior Planner will calculate the cost of going straight, changing to the left lane or changing to the right and choose the option with the lowest cost. The costs of each of these behaviors are determined by the proximity of the vehicles in the various lanes, as well as which option allows our vehicle to travel as closely as possible to the speed limit.

In its current implementation, the Behavior Planner is a simple finite state machine with only three choices. The planner only looks ahead and at the immediately adjacent lanes, so it sometimes happens that it greedily chooses a behavior that appears locally optimal, even though it might make more sense at times to slow down and get behind a vehicle that is slightly slower in order to transition to a lane that is ultimately better. However, the behavior is stable and the vehicle generally does a good job of getting itself out of traffic jams when the opportunity arises.

Another interesting behavior that arose is that when no other cars are nearby, the planner will have difficulty deciding which lane to choose and will begin to change back and forth erratically, which is obviously not desirable. This problem was solved by applying a cost to changing lanes, such that the car will always prefer to remain in its current lane, all else being equal, as well as adding a slight preference for remaining in the center lane. With these additional weights, the behavior became very stable.

### 4. Generate Required Trajectory to Fulfill Behavior

Once the behavior is determined, it is passed to the vehicle to implement. This is handled in the `Vehicle.realize_behavior` method defined [here](src/vehicle.cpp#L23). This method determines a target distance and velocity as well as the lateral offset if a lane change is called for. If another vehicle is ahead, the vehicle will slow its velocity to maintain a safe distance. With the target position and velocity determined, a new trajectory is created. The target states are also saved, as these will be the start states for the subsequent trajectory.

### 5. Pass Generated Path X & Y coordinates to Controller

Once the trajectory is calculated, it only remains to map the Frenet coordinates to the Cartesian space of the map and send these back to the controller.

This describes the complete procedure, and at this point, we return back to step 3 and determine the next behavior.
