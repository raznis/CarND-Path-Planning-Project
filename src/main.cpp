#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;
using namespace utils;
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

  // Create the "Road" and the "Path Planner" instances
  string waypoints_file = "../data/highway_map.csv";  // Waypoint map to read from
  double speed_limit = mph2ms(50); // in m/s
  int num_lanes = 3;
  double lane_width = 4; // meters
  Map highway_map = Map(speed_limit, num_lanes, lane_width, waypoints_file);

  //cout << "Road created!!" << endl;

  BehavioralPlanner BP = BehavioralPlanner(highway_map);

  //default lane and ref velocity
  int lane = 1;
  double max_velocity = 49.5;
  double ref_velocity = 0.0;
  h.onMessage([&lane, &max_velocity, &ref_velocity, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	vector<double> EgoData = { car_x, car_y, car_s, car_d, car_yaw, car_speed };

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

		    bool too_close = false;

          	int prev_size = previous_path_x.size();

          	if(prev_size > 0)
          	{
          		car_s = end_path_s;
          	}

          	for (int i = 0; i < sensor_fusion.size(); ++i) {
				float check_car_d = sensor_fusion[i][6];
          		//car is in my lane
          		if(check_car_d < 2+4*lane+2 && check_car_d > 2+4*lane-2)
          		{
          			double check_car_vx = sensor_fusion[i][3];
          			double check_car_vy = sensor_fusion[i][4];
          			double check_car_speed = sqrt(check_car_vx*check_car_vx+check_car_vy*check_car_vy);
          			double check_car_s = sensor_fusion[i][5];

          			//projection to where our leftover points end (assuming car maintains near 0 lat velocity)
          			check_car_s +=((double)prev_size*0.02*check_car_speed);
          			if((check_car_s > car_s) && (check_car_s-car_s) < 70)
          			{
          				//TODO consider car for follow distance
          				too_close = true;
          			}
          		}
			}

          	if(too_close)
          		ref_velocity -= 0.224;
          	else if(ref_velocity < max_velocity)
          		ref_velocity += 0.224;


		    vector<double> ptsx;
          	vector<double> ptsy;

          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);
          	if(prev_size < 2)
          	{
          		double prev_car_x = car_x - cos(car_yaw);
          		double prev_car_y = car_y - sin(car_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(car_x);
          		ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
          	}
          	else
          	{
          		ref_x = previous_path_x[prev_size - 1];
          		ref_y = previous_path_y[prev_size - 1];
          		double ref_x_prev = previous_path_x[prev_size - 2];
          		double ref_y_prev = previous_path_y[prev_size - 2];
          		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

          		ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);
				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
          	}

          	vector<double> next_wp0 = getXY(car_s+30.0, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+60.0, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s+90.0, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);
			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);

			for(int i = 0 ; i < ptsx.size(); i++)
			{
				//shift car reference angle and position to reference point
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;
				ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
				ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
			}

			vector<double> next_x_vals;
          	vector<double> next_y_vals;

			for(int i = 0 ; i < previous_path_x.size() ; i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			//create a spline
			tk::spline s;
			//set (x,y) points to the spline
			s.set_points(ptsx,ptsy);

			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt(target_x*target_x + target_y*target_y);

			double x_add_on = 0;

			for(int i=1; i<=50-previous_path_x.size(); i++)
			{
				double N = target_dist/(0.02*ref_velocity/2.24);
				//creating uniform points on the spline
				double x_point = x_add_on + (target_x/N);
				double y_point = s(x_point);

				x_add_on = x_point;

				//rotate and shift back to global because we are in relative frame
				double x_ref = x_point;
				double y_ref = y_point;

				x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

				x_point +=ref_x;
				y_point +=ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}

//			for(int i=0; i < 50 ; i++){
//				cout<< "(" << next_x_vals[i] << "," << next_y_vals[i] <<") ";
//			}
//			cout << endl;


          	json msgJson;
			msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
