#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
    /*

    Proportional gain regulates how large the change in the output will be for a given change in the error. If the proportional gain is too high, the system can become unstable (see p controller gif above).
    Integral gain contributes in proportion to both the magnitude of the error and the duration of the error. In this way controller is able to eliminate the residual steady-state error that occurs with a pure proportional controller (i.e. a purely proportional controller operates only when error is non-zero) and is able to deal with systematic biases.
    Derivative gain decides how much the error's rate of change is taken into account when computing the response. In other words, if the desired setpoint is getting closer (= error is decreasing) the response must be smoothed in order not to overshoot the target. Derivative component benefits the system's stability and settling time.
    */
    // P: steer in proportion to the crosstrack error
    
    // I: steer more when there is sustained error to counter the systematic bias we have from e.g. misaligned wheels.
    
    // D: When the car has turned enough to reduce CTE, it counter-steers
    // to avoid overshooting
    
    //////////// -------------------  \\\\\\\\\\\\\\\\\\
    // https://github.com/vsingla2/Self-Driving-Car-NanoDegree-Udacity/tree/master/Term2-Sensor-Fusion-Localization-and-Control/Project4-PID-Controller
    // https://github.com/ilopezfr/Self-Driving-Car-Engineering/tree/master/CarND-P09-PID-Control
  double K_p =0.3;//0.025;        // 2.9331227688652457;
  double K_i =0.001;//0.00009;      // 0.49316041639454505;
  double K_d =2;//0.9;         // 10.326589894591526; 
  


  pid.Init(K_p,K_i,K_d);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle_val = 0.3;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid.UpdateError(cte);
          steer_value =  pid.TotalError();
          if (steer_value < -1){
            //std::cout << "ana 3ndy steer as3'ar mn ----------------------------------------------------------1" << std::endl;
            steer_value = -1;
          }
          else if(steer_value > 1){
            steer_value = 1;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl << std::endl << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_val;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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
