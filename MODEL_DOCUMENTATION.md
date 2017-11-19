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

### JMT

### Vehicle

### Behavior Planner

## Initiate Map Waypoints

## Generate Startup Trajectory

## Determine Subsequent Behavior

## Generate Required Trajectory to Fulfill Behavior

## Pass Generated Path X & Y coordinates to Controller
