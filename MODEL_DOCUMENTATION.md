# Model Documentation

## Summary

The path planner defined in this repository undertakes the following steps to safely navigate around a virtual highway alongside other traffic:

  - It processes an initial list of map waypoints to be used to localize the vehicle with respect to its position on the map.
  - It produces an initial startup trajectory, which brings the vehicle safely and comfortably up to the desired speed limit.
  - It observes its surroundings and determines an appropriate behavior to navigate the course safely and efficiently
  - It generates a trajectory to fulfill the desired behavior within the desired safety and comfort parameters
  - It converts the generated trajectory into a list of X and Y coordinates to be fed back to the vehicle controller

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

### Initiate Map Waypoints

### Generate Startup Trajectory

### Determine Subsequent Behavior

### Generate Required Trajectory to Fulfill Behavior

### Pass Generated Path X & Y coordinates to Controller
