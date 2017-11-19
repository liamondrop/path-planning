# Model Documentation

## Summary

The path planner defined in this repository undertakes the following steps to safely navigate around a virtual highway alongside other traffic:

  - It processes an initial list of map waypoints to be used to localize the vehicle with respect to its position on the map.
  - It produces an initial startup trajectory, which brings the vehicle safely and comfortably up to the desired speed limit.
  - It observes its surroundings and determines an appropriate behavior to navigate the course safely and efficiently
  - It generates a trajectory to fulfill the desired behavior within the desired safety and comfort parameters
  - It converts the generated trajectory into a list of X and Y coordinates to be fed back to the vehicle controller

## Classes

### Map Waypoints

When the path planning module is first started, the [Map Waypoints](src/map-waypoints.h) class is initiated by reading in a [csv file](data/highway_map.csv) consisting of 181 waypoints. These waypoints coarsely define the full path of the track, which is a loop.

Each waypoint is composed of an x, y map position, a Frenet s value (the distance along the direction of the road), and a Frenet d unit normal vector (split up into the x component, and the y component). The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. By fitting a cubic spline to the relationship between the distance traveled (s) and each other waypoint coordinate, we can precisely determine the (x, y) coordinates of the planned trajectories. The library used to fit the splines can be found at [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/).

### JMT

### Vehicle

### Behavior Planner

## Initiate Map Waypoints

## Generate Startup Trajectory

## Determine Subsequent Behavior

## Generate Required Trajectory to Fulfill Behavior

## Pass Generated Path X & Y coordinates to Controller
