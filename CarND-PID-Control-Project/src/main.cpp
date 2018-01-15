#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Send Reset Signal
void resetSimulator(uWS::WebSocket<uWS::SERVER> ws) {
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  bool twiddle = false;

  double tol = 0.2;

  double best_err = -1;

  //pd index
  int pd_i = 0;

  int term = 0;

  PID pid;
  // TODO: Initialize the pid variable.
  double params[3] = {1.0,0.0,0.0};
  double paramds[3] = {0.5,0.5,0.5};
  if (twiddle) {
    pid.Init(params[0], params[1], params[2]);
  } else {
    pid.Init(0.3, 0.0005, 3);
  }

  h.onMessage([&pid, &twiddle, &tol, &pd_i, &paramds, &term, &best_err](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          // std::cout << pid.GetKp() << " " << pid.GetKi() << " " << pid.GetKd() << std::endl;
          steer_value = pid.TotalError();

          if (steer_value > 1.0) {
            steer_value = 1.0;
          }

          if (steer_value < -1.0) {
            steer_value = -1.0;
          }

          if (twiddle) {
            if (best_err < 0) {
              best_err = pid.SquaredError();
            }
            if (pid.Iteration() >= 1000) {
              if (pid.SquaredError() < tol) {
                twiddle = false;
                std::cout << "*****************************************************" << std::endl;
                std::cout << pid.GetKp() << " " << pid.GetKi() << " " << pid.GetKd() << std::endl;
                std::cout << "*****************************************************" << std::endl;
              } else {
                std::cout << pd_i << " " << term  << std::endl;
                switch(pd_i){
                  case 0: {
                    if (term == 0) {
                      pid.UpdateKp(paramds[0]);
                      term = 1;
                    } else if (term == 1) {
                      if (pid.SquaredError() < best_err) {
                        best_err = pid.SquaredError();
                        paramds[0] *= 1.1;
                        term = 0;
                        pd_i = 1;
                      } else {
                        pid.UpdateKp(-2 * paramds[0]);
                        term = 2;
                      }
                    } else if (term == 2) {
                      if (pid.SquaredError() < best_err) {
                        best_err = pid.SquaredError();
                        paramds[0] *= 1.1;
                      } else {
                        pid.UpdateKp(paramds[0]);
                        paramds[0] *= 0.9;
                      }
                      term = 0;
                      pd_i = 1;
                    }
                    break;
                  }
                  case 1:{
                    if (term == 0) {
                      pid.UpdateKi(paramds[1]);
                      term = 1;
                    } else if (term == 1) {
                      if (pid.SquaredError() < best_err) {
                        best_err = pid.SquaredError();
                        paramds[1] *= 1.1;
                        term = 0;
                        pd_i = 2;
                      } else {
                        pid.UpdateKi(-2 * paramds[1]);
                        term = 2;
                      }
                    } else if (term == 2) {
                      if (pid.SquaredError() < best_err) {
                        best_err = pid.SquaredError();
                        paramds[1] *= 1.1;
                      } else {
                        pid.UpdateKi(paramds[1]);
                        paramds[1] *= 0.9;
                      }
                      term = 0;
                      pd_i = 2;
                    }
                    break;
                  }
                  case 2:{
                    if (term == 0) {
                      pid.UpdateKd(paramds[2]);
                      term = 1;
                    } else if (term == 1) {
                      if (pid.SquaredError() < best_err) {
                        best_err = pid.SquaredError();
                        paramds[2] *= 1.1;
                        term = 0;
                        pd_i = 0;
                      } else {
                        pid.UpdateKd(-2 * paramds[2]);
                        term = 2;
                      }
                    } else if (term == 2) {
                      if (pid.SquaredError() < best_err) {
                        best_err = pid.SquaredError();
                        paramds[2] *= 1.1;
                      } else {
                        pid.UpdateKd(paramds[2]);
                        paramds[2] *= 0.9;
                      }
                      term = 0;
                      pd_i = 0;
                    }
                    break;
                  }
                }
              }
              pid.Reset();
              resetSimulator(ws);
            }
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
