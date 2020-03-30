#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int get_lane(float d){
    
    int lane;
    if(d > 0 && d <= 4){
        lane = 0;
    } else if (d > 4 && d <= 8){
        lane = 1;
    } else if (d > 8 && d < 12){ 
        lane = 2;
    }
    
    return lane;
}

vector<string> successor_states(string state, int curr_lane){
    
    vector<string> states;
    
    if(state.compare("KL") == 0){
        if(curr_lane > 0){
            states.push_back("LCL");
            
            if(curr_lane < 2){
                states.push_back("LCR");
            }
        }
    }
    
    states.push_back("KL");
    
    return states;
}

float lane_speed(int lane, vector< vector<double> > sensor_fusion){
    
    for(int i = 0; i < sensor_fusion.size(); ++i){
        
        float d = sensor_fusion[i][6];
        
        if(d < (4 * lane + 4) && d > (4 * lane)){
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double speed = sqrt(vx * vx + vy * vy);
            
            return speed;
        }
    }
    
    return -1.0;
    
}

float get_vehicle_ahead(int lane, int prev_size, vector< vector<double> > sensor_fusion, float car_s){
        
    float ds = 99999;
    float inquired_car_s = -1.0;
    
    for(int i = 0; i < sensor_fusion.size(); ++i){
        
        float d = sensor_fusion[i][6];
        // front car is in my lane
        if(d < (4 * lane + 4) && d > (4 * lane)){
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
                  
            check_car_s += ((double)prev_size * .02 * check_speed);
            
            if(check_car_s > car_s && ((check_car_s - car_s) < ds)){
                inquired_car_s = check_car_s;
                ds = check_car_s - car_s;
            }
        }
    }
    
    return inquired_car_s;
}

float get_vehicle_behind(int lane, int prev_size, vector< vector<double> > sensor_fusion, float car_s){
    
    float ds = -99999;
    float inquired_car_s = -1.0;
    
    for(int i = 0; i < sensor_fusion.size(); ++i){
        
        float d = sensor_fusion[i][6];
        // front car is in my lane
        if(d < (4 * lane + 4) && d > (4 * lane)){
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
                  
            check_car_s += ((double)prev_size * .02 * check_speed);
            
            if(check_car_s < car_s && ((check_car_s - car_s) > ds)){
                inquired_car_s = check_car_s;
                ds = check_car_s - car_s;
            }
        }
    }
    
    return inquired_car_s;
}

vector<double> realize_next_state(string& state, int curr_lane, int prev_size, vector< vector<double> > sensor_fusion, string best_next_state, float car_s, float end_path_s){
    
    float velocity = lane_speed(curr_lane, sensor_fusion);
    if(velocity < 0){velocity = 49.5;};
    int lane = curr_lane;
    state = "KL";
    
    if(prev_size > 0){
        car_s = end_path_s;
    }
    
    if (best_next_state.compare("LCL") == 0){
        
        float s_ahead = get_vehicle_ahead(curr_lane - 1, prev_size, sensor_fusion, car_s);
        float s_behind = get_vehicle_behind(curr_lane - 1, prev_size, sensor_fusion, car_s);
        
        if((s_ahead == -1.0 || (s_ahead - car_s) > 5) && (s_behind == -1.0 || (s_behind - car_s) < -5)){
            velocity = lane_speed(curr_lane - 1, sensor_fusion);
            if(velocity < 0){velocity = 49.5;}
            lane = curr_lane - 1;
            state = "LCL";
        }
    } else if (best_next_state.compare("LCR") == 0){
        
        float s_ahead = get_vehicle_ahead(curr_lane + 1, prev_size, sensor_fusion, car_s);
        float s_behind = get_vehicle_behind(curr_lane + 1, prev_size, sensor_fusion, car_s);
        
        if((s_ahead == -1.0 || (s_ahead - car_s) > 5) && (s_behind == -1.0 || (s_behind - car_s) < -5)){
            velocity = lane_speed(curr_lane + 1, sensor_fusion);
            if(velocity < 0){velocity = 49.5;}
            lane = curr_lane + 1;
            state = "LCR";
        }        
    }
    
    return {velocity, (double)lane};
}

float calculate_cost(int curr_lane, string possible_state, vector< vector<double> > sensor_fusion){
    
    float cost;
    int change_lane;
    if(possible_state.compare("KL") == 0){
        change_lane = 0;
    } else if (possible_state.compare("LCL") == 0){
        change_lane = -1;
    } else {
        change_lane = 1;
    }
    
    int next_lane = curr_lane + change_lane;
    
    float final_lane_speed = lane_speed(next_lane, sensor_fusion);
    
    if(final_lane_speed < 0){
        cost = 0;
    } else {
        cost = (49.5 - final_lane_speed) / 49.5;
    }
    
    return cost;
}

vector<double> choose_next_state(int prev_size, string& state, float car_d, vector< vector<double> > sensor_fusion, float car_s, float end_path_s){
    
    int curr_lane = get_lane(car_d);
    
    vector<string> possible_successor_states = successor_states(state, curr_lane);
    vector<float> costs;
    
    for(int i = 0; i < possible_successor_states.size(); ++i){
        float cost = calculate_cost(curr_lane, possible_successor_states[i], sensor_fusion);
        costs.push_back(cost);
    }
    
    string best_next_state = "";
    float min_cost = 9999999;
    
    for(int i = 0; i < costs.size(); ++i){
        if(costs[i] < min_cost){
            min_cost = costs[i];
            best_next_state = possible_successor_states[i];
        }
    }
    
    std::cout << state << std::endl;
    
    return realize_next_state(state, curr_lane, prev_size, sensor_fusion, best_next_state, car_s, end_path_s);
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
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  
  // velocity reference mph
  double vel_ref = 0;
  // lane number
  int lane_nb = 1;
  // initial state
  string state = "KL";

  h.onMessage([&vel_ref,&lane_nb,&state,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          
          // Previous path's size
          int prev_size = previous_path_x.size();
          
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          
          /*
          if(prev_size > 0){
              car_s = end_path_s;
          }
          
          bool too_close = false;
          
          for(int i = 0; i < sensor_fusion.size(); ++i){
              
              float d = sensor_fusion[i][6];
              // front car is in my lane
              if(d < (4 * lane_nb + 4) && d > (4 * lane_nb)){
                  
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx * vx + vy * vy);
                  double check_car_s = sensor_fusion[i][5];
                  
                  check_car_s += ((double)prev_size * .02 * check_speed);
                  
                  if((check_car_s > car_s) && ((check_car_s - car_s) < 25)){
                      //vel_ref = check_speed / 0.447;
                      too_close = true;
                      if(lane_nb > 0){
                          lane_nb = 0;
                      }
                  }
              }
          }
          
          if(too_close){
              vel_ref -= .224;
          }
          else if(vel_ref < 49.5){
              vel_ref += .224;
          }
          */
          
          vector<double> refs = choose_next_state(prev_size, state, car_d, sensor_fusion, car_s, end_path_s);
          float velocity = refs[0];
          lane_nb = (int)refs[1];
          
          std::cout << "vel: " << velocity << " lane: " << lane_nb << std::endl;
          
          if(vel_ref > velocity){
              vel_ref -= .224;
          }
          else if(vel_ref < velocity){
              vel_ref += .224;
          }
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // add two points into the path
          if(prev_size < 2){
              // find tangent previous points
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          }
          else{
              // take last two waypoints from previous path
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
              
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }
          
          // Add 25m spaced points in frenet
          vector<double> next_wp0 = getXY(car_s + 25, (2 + 4*lane_nb), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 50, (2 + 4*lane_nb), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 75, (2 + 4*lane_nb), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // shift the coordinate to the car's frame
          for(int i = 0; i < ptsx.size(); ++i){
            
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
            ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
          }
          
          tk::spline s;
          
          s.set_points(ptsx, ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for(int i = 0; i < prev_size; ++i){
              // copy from previous path
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          
          double x_prime = 0;
          
          for (int i = 0; i < 50 - prev_size; ++i) {
            
            double N = target_dist / (.02 * vel_ref * 0.447);
            double x_point = x_prime + target_x / N;
            double y_point = s(x_point);
            
            x_prime = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // transform back to map coordiante
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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