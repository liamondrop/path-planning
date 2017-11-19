#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "json.hpp"

#include "./behavior-planner.h"
#include "./jmt.h"
#include "./map-waypoints.h"
#include "./vehicle.h"

// For convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned, else the
// empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");

  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }

  return "";
}

MapPath generate_initial_path(Vehicle &my_vehicle,
                              const MapWaypoints &map_waypoints) {
  const int steps = 2 * TIME_STEPS;
  const double time_horizon = steps * TIME_INCREMENT;
  const double target_s = my_vehicle.s + 40.0;

  const VehicleState start_state_s = {my_vehicle.s, my_vehicle.v, 0.0};
  const VehicleState start_state_d = {my_vehicle.d, 0.0, 0.0};

  const VehicleState end_state_s = {target_s, SPEED_LIMIT, 0.0};
  const VehicleState end_state_d = {my_vehicle.d, 0.0, 0.0};

  std::vector<double> jmt_s = JMT::get_jmt(start_state_s, end_state_s, time_horizon);
  std::vector<double> jmt_d = JMT::get_jmt(start_state_d, end_state_d, time_horizon);

  my_vehicle.set_goal_states(end_state_s, end_state_d);

  return map_waypoints.generate_path(jmt_s, jmt_d, TIME_INCREMENT, 0, steps);
}

int main() {
  uWS::Hub h;

  bool is_initial_frame = true;
  std::cout << "Loading map waypoints..." << std::endl;
  MapWaypoints map_waypoints("../data/highway_map.csv");
  std::cout << "Map waypoints loaded..." << std::endl;

  h.onMessage([&map_waypoints, &is_initial_frame](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          auto json_data = j[1];

          // Main car's localization Data
          const double car_x = json_data["x"];
          const double car_y = json_data["y"];
          const double car_s = json_data["s"];
          const double car_d = json_data["d"];
          const double car_speed = json_data["speed"];

          // Previous path data given to the Planner
          const std::vector<double> previous_path_x = json_data["previous_path_x"];
          const std::vector<double> previous_path_y = json_data["previous_path_y"];
          const double end_path_s = json_data["end_path_s"];
          const double end_path_d = json_data["end_path_d"];

          auto sensor_fusion = json_data["sensor_fusion"];

          // Update our vehicle instance
          Vehicle my_vehicle(car_s, car_d, car_speed);

          // Return the previous planned path until the path size is less than
          // the PATH_SIZE_THRESHOLD
          int path_size = previous_path_x.size();
          MapPath planned_path = {previous_path_x, previous_path_y};

          if (is_initial_frame) {
            planned_path = generate_initial_path(my_vehicle, map_waypoints);
            is_initial_frame = false;
          } else if (path_size < PATH_SIZE_THRESHOLD) {
            std::cout << "=================================" << std::endl;
            std::cout << "CAR_S: " << car_s << std::endl;
            std::cout << "CAR_D: " << car_d << std::endl;
            std::cout << "CAR_X: " << car_x << std::endl;
            std::cout << "CAR_Y: " << car_y << std::endl;

            // inititate a placeholder for the nearest
            // forward vehicle.
            Vehicle forward_vehicle(0, -1, 0);
            double smallest_gap = LARGE_VALUE;

            std::vector<Vehicle> other_vehicles;
            for (auto detected_vehicle : sensor_fusion) {
              const int id = detected_vehicle[0];
              const double vx = detected_vehicle[3];
              const double vy = detected_vehicle[4];
              const double s = detected_vehicle[5];
              const double d = detected_vehicle[6];
              const double velocity = sqrt(vx * vx + vy * vy);

              Vehicle other_vehicle(s, d, velocity);

              // find the nearest vehicle in the current lane
              const double gap = s - car_s;
              if (other_vehicle.current_lane == my_vehicle.current_lane &&
                  gap > 0.0 && gap < smallest_gap) {
                smallest_gap = gap;
                forward_vehicle = other_vehicle;
              }

              other_vehicles.push_back(other_vehicle);
            }

            // determine the space between our vehicle and the nearest vehicle
            // in the current lane, if any
            double forward_gap = forward_vehicle.s - my_vehicle.s;
            if (forward_gap < 0) forward_gap = LARGE_VALUE;

            BehaviorPlanner planner(my_vehicle, other_vehicles);
            Behavior behavior = planner.get_behavior(forward_gap);

            int n_steps = TIME_STEPS - path_size;
            double time_horizon = n_steps * TIME_INCREMENT;
            my_vehicle.realize_behavior(behavior, forward_vehicle, time_horizon);

            MapPath next_path = map_waypoints.generate_path(
                my_vehicle.get_s_trajectory(),
                my_vehicle.get_d_trajectory(),
                TIME_INCREMENT, 0, n_steps);

            planned_path.X.insert(planned_path.X.end(),
                                  next_path.X.begin(),
                                  next_path.X.end());
            planned_path.Y.insert(planned_path.Y.end(),
                                  next_path.Y.begin(),
                                  next_path.Y.end());
          }

          // Send updated path plan to simulator
          json json_message;
          json_message["next_x"] = planned_path.X;
          json_message["next_y"] = planned_path.Y;

          auto msg = "42[\"control\"," + json_message.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {});

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;

  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
