#include <fstream>
#include <algorithm>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"

#include "coordinates.h"

// https://github.com/lava/matplotlib-cpp

// https://github.com/jal278/plotpipe
#include "decision_making.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;
    CoordinateTransformer transform;
    Planner planner(transform);

    h.onMessage([
                        &planner,
                        &transform
                ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {

        const double target_max_vel = 49.5;
        const double collision_dist_high = 60;
        const double collision_dist_low = 20;
        const double collision_speed_high = 50;
        const double collision_speed_low = 40;
        const int max_reused_points = 900;
        const int number_steps_planned = 50;

        const double speed_decrement = -.4;
        const double speed_increment = .2;

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {

                    // j[1] is the data JSON object
                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    int num_unused = j[1]["previous_path_x"].size();

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    vector<Neighbor> neighbors;
                    for (auto sf : sensor_fusion) {
                        //sf = id, x, y, vx, vy, s, d
                        neighbors.push_back(Neighbor(transform, sf[0], sf[1], sf[2], sf[3], sf[4]));
                    }

                    vector<vector<double>> next_xy_vals = planner.make_plan(
                            {.x=car_x, .y=car_y, .yaw=car_yaw},
                            car_speed,
                            num_unused,
                            neighbors
                    );

                    // Pack our message in a json object.
                    json msgJson;
                    msgJson["next_x"] = next_xy_vals[0];
                    msgJson["next_y"] = next_xy_vals[1];

                    // Send.
                    auto msg = "42[\"control\"," + msgJson.dump() + "]";
                    //this_thread::sleep_for(chrono::milliseconds(1000));
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
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
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
