#include "helper.h"
#include "json.hpp"
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    //std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];
          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"];
          double v   = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Transform from global coordinates to car coordinates.
          // The global position of the car is (px, py).
          std::vector<double> ptsx_car(ptsx.size());
          std::vector<double> ptsy_car(ptsy.size());
          for (size_t i = 0; i < ptsx.size(); ++i) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            ptsx_car.push_back(dx * cos(-psi) - dy * sin(-psi));
            ptsy_car.push_back(dx * sin(-psi) + dy * cos(-psi));
          }

          Eigen::Map<Eigen::VectorXd> waypoints_x(ptsx_car.data(), ptsx_car.size());
          Eigen::Map<Eigen::VectorXd> waypoints_y(ptsy_car.data(), ptsy_car.size());

          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);
          auto coeffs_p = derivative(coeffs);

          // Set the initial state.
          // Since we're car coordinates for our calculations, the initial position is (0, 0)
          // rather than (px, py). Also the initial value for the steering angle will be 0.

          // The initial CTE is just the difference between the function evaluated
          // at x and the initial value of y.
          double cte  = polyeval(coeffs, 0);  // f(px) - py
          // The initial orientation error is just the difference between the arctangent
          // of the function derivative evaluated at x and the initial value of psi.
          // Multiply by -1 to account for the difference between angles in our equations
          // vs the simulator.
          //double epsi = -atan(polyeval(coeffs_p, 0)); // arctan(f'(px)) - psi
          double epsi = -atan(coeffs[1]);  // p

          Eigen::VectorXd state(6);
          //state << px, py, psi, v, cte, epsi;
          state << 0.0, 0.0, 0.0, v, cte, epsi;

          // Solve.
          auto vars = mpc.Solve(state, coeffs);

          // Exract the calculated steering and throttle values.
          // Multiply the steering angle by -1: in the update equation for delta
          // a positive value indicates a counter-clockwise (left) turn while in
          // the simulator a positive value indicates a clockwise (right) turn.
          // We also divide by the delta_limit value used to limit steering changes
          // in the model. This translates the steering angle from the range
          // [-delta_limit,+deltaLimit] to [-1,_1] as required by the simulator.
          double steer_value    = -vars[0] / mpc.delta_limit;
          double throttle_value = vars[1];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"]       = throttle_value;

          //Display the MPC predicted trajectory
          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (size_t i = 2; i < vars.size() - 1; ++i, ++i) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (double x = 0; x < 100; x += 2.5) {
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs, x));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be able to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
