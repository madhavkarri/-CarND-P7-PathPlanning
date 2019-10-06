#include <iostream>
#include <math.h>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <fstream>
#include <uWS/uWS.h>

#include "json.hpp"
#include "helpers.h"
#include "spline.h"

using namespace std;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // load map values for waypoint's x,y,s, and d normalized vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  string line;
  while (getline(in_map_, line))
  {
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

  // car lane, at time t = 0 (middle lane)
  int lane = 1;

  // initialize reference velocity
  double ref_vel = 0.0; // mph
  
  h.onMessage([&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
  	          (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
  {
  	// "42" at the start of the message means there's a websocket message event.
    // 4 signifies a websocket message
    // 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);
      if (s != "")
      {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          
          // ego car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // previous path data given to planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // sensor fusion data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // previous path point size
          int prev_size = previous_path_x.size();
          
          // prevent collisions
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }

          // sensing and prediction: analyse other cars location and behavior
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
      			// sense lanes of other cars in the neighborhood
      			float d = sensor_fusion[i][6];
      			int car_lane = -1;
            if ( d > 0 && d < 4 )
            {
              car_lane = 0;
            }
            else if ( d > 4 && d < 8 )
            {
              car_lane = 1;
            }
            else if ( d > 8 && d < 12 )
            {
              car_lane = 2;
            }
            if (car_lane < 0)
            {
              continue;
            }			
      			// find other cars speed
      			double oc_vx = sensor_fusion[i][3];
      			double oc_vy = sensor_fusion[i][4];
      			double oc_speed = sqrt(oc_vx*oc_vx + oc_vy*oc_vy);
      			double oc_car_s = sensor_fusion[i][5];
      			// estimate other cars s position after executing previous trajectory
      			oc_car_s += ((double)prev_size*0.02*oc_speed);

      			// identify lanes of other cars w.r.t ego car
            // safe distance between ego car and other cars
            double gap = 20; // atleast maintain 20 m
      			// car ahead of us
      			if ( car_lane == lane )
      			{
      				car_ahead = car_ahead | ((oc_car_s > car_s) && (oc_car_s - car_s < gap));
            }
            // car in left lane
            else if ( car_lane - lane == -1 )
            {
            	car_left = car_left | ((car_s - gap < oc_car_s) && (car_s + gap > oc_car_s));
            }
            // car in right lane
            else if ( car_lane - lane == 1 ) 
            {
                car_right = car_right | ((car_s - gap < oc_car_s) && (car_s + gap > oc_car_s));
            }

          }

          // behavior planning
          double speed_diff = 0;
          // max allowable seed 50 mph
          // should be below 50 mph, but close to 50 mph to finish 1 lap under 6 mins
          const double max_speed = 47.5; 
          // max allowable acceleration 10 m/s2
          // max allowable jerk (rate of change of acceleration) 10 m/s3
          // if acceleration maintained under 10 m/s2 jerk will also be under 10 m/s3
          // translate max allowable acceleration to change in vehicle speed (in mph) to be applied/updated
          // 10 m/s2 translates to change in vehicle speed from 0 to 10 m/s in 1 second 
          // or 22.3694 mph in 1 second to be applied/updated
          // simulator cycles every 20 ms (milli seconds), 50 ms makes 1 second
          // 22.3694/50 = approx 0.45 mph every 20 ms to be applied/updated
          // to maintain a factor of safety cut 0.45 mph by 25 %, would be about 0.335 mph
          const double max_acc = 0.335; // max allowable accel
          
          // car head
          if ( car_ahead )
          {

          	// if no car on left and left lane exists
          	if ( !car_left && lane > 0 )
          	{
          		lane--; // change lane left
            }
            // if no car on right and right lane exists
            else if ( !car_right && lane != 2 )
            {
                lane++; // Change lane right
            } 
            else 
            {
                speed_diff -= max_acc;
            }
          } 
          
          else
          {
          	// shift ego car to center lane, when possible
          	if ( lane != 1 )
          	{
	            if ( ( lane == 0 && !car_right ) || ( lane == 2 && !car_left ) )
	            {
	            	lane = 1; // revert back to center lane
	            }
            }
            // retain ego car vehicle speed close to maximum, when possible
            if ( ref_vel < max_speed )
            {
                speed_diff += max_acc;
            }
          }

          // create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    		  // later interpolate these waypoints with a spline and fill it with more points to control speed

    		  vector<double> ptsx;
    		  vector<double> ptsy;

    		  // reference x, y, yaw states
    		  // either reference starting point or where car is or at previous paths end points
    		  double ref_x = car_x;
    		  double ref_y = car_y;
    		  double ref_yaw = deg2rad(car_yaw);
          	
          // if previous size is almost empty, use car as starting reference
          if(prev_size < 2)
          {
          	// use two points that make path tangent to the car
          	double prev_car_x = car_x - cos(car_yaw);
          	double prev_car_y = car_y - sin(car_yaw);

          	ptsx.push_back(prev_car_x);
          	ptsx.push_back(car_x);

          	ptsy.push_back(prev_car_y);
          	ptsy.push_back(car_y);     	

          }
          // use previous paths end point as starting reference
          else
          {
          	// redefine reference state as previous path end point
          	ref_x = previous_path_x[prev_size-1];
          	ref_y = previous_path_y[prev_size-1];

          	double ref_x_prev = previous_path_x[prev_size-2];
          	double ref_y_prev = previous_path_y[prev_size-2];
          	ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

          	// use two points that makes the path tangent to the previous points end point
          	ptsx.push_back(ref_x_prev);
          	ptsx.push_back(ref_x);

          	ptsy.push_back(ref_y_prev);
          	ptsy.push_back(ref_y);

          }

          // in Frenet add evenly spaced points (30 m incs) ahead of starting reference
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++)
          {
          	// shift car reference angle to 0 degrees
          	double shift_x = ptsx[i]-ref_x;
          	double shift_y = ptsy[i]-ref_y;

          	ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          	ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

          }

          // create spline
          tk::spline s;
          // set (x, y) points to spline
          s.set_points(ptsx,ptsy);

          // define actual (x, y) points for planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all previous path points from last time
          for(int i = 0; i < prev_size; i++)
          {
          	next_x_vals.push_back(previous_path_x[i]);
          	next_y_vals.push_back(previous_path_y[i]);
          }

          // calcualte number of break points for spline curve to maintain speed at reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          // fill rest of path planner after filling it with previous points
          // this will always output 50 points
          for(int i = 1; i < 50-prev_size; i++)
          {
            
          	ref_vel += speed_diff;
          	if (ref_vel > max_speed)
          	{
          		ref_vel = max_speed;
          	}
          	else if (ref_vel<max_acc)
          	{
          		ref_vel = max_acc;
          	}
            
          	double N = target_dist/(0.02*ref_vel/2.24);
          	double x_point = x_add_on+target_x/N;
          	double y_point = s(x_point);

          	x_add_on = x_point;

          	double x_ref = x_point;
          	double y_ref = y_point;

          	// rotate back to normal, after prior rotation
          	x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
          	y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

          	x_point += ref_x;
          	y_point += ref_y;

          	next_x_vals.push_back(x_point);
          	next_y_vals.push_back(y_point);

          }

          json msgJson;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if

      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }

    } // end websocket if
  }); // end h.onMessage


  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
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
