#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "constants.hpp"
#include "math_helper.hpp"
#include "trajectory_helper.hpp"

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

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  // Planned speed of vehicle
  double planned_speed_mps(0);
  unsigned planned_lane_idx(1);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &planned_speed_mps, &planned_lane_idx](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	// Previous path data given to the Planner (without the points that have already been passed by)
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
            std::cout << "telemetry: (s, d) = (" << car_s << ", " << car_d << "), (x, y) = (" << car_x << ", " << car_y << "), v = " << car_speed << "\n";
            std::cout << "           Previous path size: " << previous_path_x.size() << "\n";
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // Helper data for processing steps below
            unsigned lane_idx((car_d <= 0.0) ? 0 : NUMBER_OF_LANES-1);
            for ( unsigned i=0; i<NUMBER_OF_LANES; i++ )
            {
              if ( (i * LANE_WIDTH_M < car_d) && (car_d <= (i+1) * LANE_WIDTH_M) )
              {
                lane_idx = i;
                break;
              }
            }
            int prev_size = previous_path_x.size();

            ////////////////////////////////////////////////////////////////////////
            // Check other cars

            // check trajectories for safety
            double check_s( (prev_size > 0) ? end_path_s : car_s );
            bool need_lane_adjustment(false), need_speed_reduction(false);

            // (1) check current behavior
            if ( lane_idx == planned_lane_idx )
            {
              // factor 1.5 to consider lane changes early when approaching another vehicle
              need_lane_adjustment = !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.5, true, lane_idx, sensor_fusion);
            }
            else if ( std::abs(lane_idx - planned_lane_idx) == 1 )
            {
              need_lane_adjustment = !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 0.5, true,  lane_idx,         sensor_fusion)
                                  || !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, false, planned_lane_idx, sensor_fusion)
                                  || !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, true,  planned_lane_idx, sensor_fusion);
            }
            else
            {
              need_lane_adjustment = !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 0.5, true,  lane_idx,                      sensor_fusion)
                                  || !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, false, (lane_idx+planned_lane_idx)/2, sensor_fusion)
                                  || !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, true,  (lane_idx+planned_lane_idx)/2, sensor_fusion)
                                  || !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, false, planned_lane_idx,              sensor_fusion)
                                  || !isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, true,  planned_lane_idx,              sensor_fusion);
            }

            // (2) if current behavior is not safe, check if lane change helps
            if ( need_lane_adjustment )
            {
              // pessimistic... ;-)
              need_speed_reduction = true;
              std::vector<unsigned> lane_candidate_idx;
              double max_safe_dist(0.0);

              // collect possible (yet unchecked) adjacent alternative lanes
              lane_candidate_idx.clear();
              if ( (0                <= int(lane_idx) - 1)
                && (planned_lane_idx != lane_idx      - 1) )
              {
                lane_candidate_idx.push_back(lane_idx - 1);
              }
              if ( (NUMBER_OF_LANES   > lane_idx + 1)
                && (planned_lane_idx != lane_idx + 1) )
              {
                lane_candidate_idx.push_back(lane_idx + 1);
              }

              // evaluate alternative lane candidates
              for ( unsigned i=0; i<lane_candidate_idx.size(); i++ )
              {
                if ( isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 0.5, true,  lane_idx,              sensor_fusion)
                  && isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, false, lane_candidate_idx[i], sensor_fusion)
                  && isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, true,  lane_candidate_idx[i], sensor_fusion) )
                {
                  double safe_dist = getMinDistance(prev_size*TIME_INCREMENT_S, check_s, true, lane_candidate_idx[i], sensor_fusion);
                  if ( safe_dist > max_safe_dist )
                  {
                    max_safe_dist        = safe_dist;
                    need_speed_reduction = false;
                    planned_lane_idx     = lane_candidate_idx[i];
                  }
                }
              }

              // collect possible (yet unchecked) distant alternative lanes
              lane_candidate_idx.clear();
              if ( (0                <= int(lane_idx) - 2)
                && (planned_lane_idx != lane_idx      - 2) )
              {
                lane_candidate_idx.push_back(lane_idx - 2);
              }
              if ( (NUMBER_OF_LANES   > lane_idx + 2)
                && (planned_lane_idx != lane_idx + 2) )
              {
                lane_candidate_idx.push_back(lane_idx + 2);
              }

              // evaluate alternative lane candidates
              for ( unsigned i=0; i<lane_candidate_idx.size(); i++ )
              {
                if ( isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 0.5, true,  lane_idx,                           sensor_fusion)
                  && isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, false, (lane_idx+lane_candidate_idx[i])/2, sensor_fusion)
                  && isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, true,  (lane_idx+lane_candidate_idx[i])/2, sensor_fusion)
                  && isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, false, lane_candidate_idx[i],              sensor_fusion)
                  && isSafeLane(prev_size*TIME_INCREMENT_S, check_s, planned_speed_mps, 1.0, true,  lane_candidate_idx[i],              sensor_fusion) )
                {
                  double safe_dist = getMinDistance(prev_size*TIME_INCREMENT_S, check_s, true, lane_candidate_idx[i], sensor_fusion);
                  if ( safe_dist > max_safe_dist )
                  {
                    max_safe_dist        = safe_dist;
                    need_speed_reduction = false;
                    planned_lane_idx     = lane_candidate_idx[i];
                  }
                }
              }
            }
            
            // (3) adjust speed if necessary or desired
            static const double delta_speed_mps = 0.224;
            if ( need_speed_reduction && planned_speed_mps >= delta_speed_mps )
            {
              planned_speed_mps -= delta_speed_mps;
            }
            else if ( planned_speed_mps + delta_speed_mps <= GOAL_SPEED_MPS )
            {
              planned_speed_mps += delta_speed_mps;
            }
            std::cout << "           Planned speed: v = " << planned_speed_mps << "\n";

            ////////////////////////////////////////////////////////////////////////
            // Smooth road with spline

            // widely spaced (x, y) waypoints, evenly spaced at 30m
            // later we will interpolate these waypoints with a spline and fill it in with more points that control speed
            vector<double> ptsx, ptsy;

            // reference x, y, 
            double ref_x(car_x), ref_y(car_y), ref_yaw(deg2rad(car_yaw)), ref_s(car_s);

            // if previous size is almost empty, use the car as starting reference
            if ( prev_size < 2 )
            {
              // use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            // use the previous path's end point as starting reference
            else
            {
              // redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
              std::vector<double> sd = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
              ref_s = sd[0];

              // use two points that make the path tangent to the car
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            // in Frenet add evenly 30m spaced points ahead of the starting reference
            unsigned used_lane_idx(lane_idx);
            for ( int i=0; i<3; i++ )
            {
              if      ( planned_lane_idx > used_lane_idx ) used_lane_idx++;
              else if ( planned_lane_idx < used_lane_idx ) used_lane_idx--;
              vector<double> next_wp = getXY(ref_s+(i+1)*30, (used_lane_idx+0.5)*LANE_WIDTH_M, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_wp[0]);
              ptsy.push_back(next_wp[1]);
            }
            // transform from world coordinate system to car coordinate system
            for ( int i=0; i<ptsx.size(); i++ )
            {
              // shift car reference angle to 0 degrees
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
              ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
              //std::cout << "           ... spline_point[" << i << "]: (x, y) = (" << ptsx[i] << ", " << ptsy[i] << ") ...\n";
            }

            // create a spline
            tk::spline spline_along_road_ahead;
            spline_along_road_ahead.set_points(ptsx, ptsy);

            ////////////////////////////////////////////////////////////////////////
          	// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // start with all points from last time
            for ( int i=0; i<previous_path_x.size(); i++ )
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // calculate how to break up spline points so that we travel at our desired reference speed:
            // target_dist = nbr_points * 0.02 s * speed m/s => nbr_points
            double target_x(30.0), target_y( spline_along_road_ahead(target_x) );
            double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
            double nbr_points( target_dist / (TIME_INCREMENT_S * planned_speed_mps) );

            // break up points
            double x_add_on(0.0);
            for ( int i=1; i<=50-previous_path_x.size(); i++ )
            {
              double x_point( x_add_on + (target_x / nbr_points) ), y_point( spline_along_road_ahead(x_point) );
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // transform back from car coordinate system to world coordinate system
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              // add points to next points
              std::cout << "           ... adding (x, y) = (" << x_point << ", " << y_point << ") ...\n";
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
            
            std::cout << "           Next path size: " << next_x_vals.size() << "\n";

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
