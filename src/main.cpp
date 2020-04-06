#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

   /////define lane
  int lane = 1;

  /////reference_speed
  double ref_vel = 0.0 ;//mph
  

   

//////add variable need add in h.onmessage[&variable_name]

  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
 
          
          

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
         
          

          /////get previous path size
          int pre_path_size = previous_path_x.size();
          
          
          

          



         /////set ego car s value
         if (pre_path_size > 0)
         {
           car_s = end_path_s;
         }
         
         

         ////define distance bool values
         bool too_close = false;

         /////judge car in sensor fusion is front of ego car
         for (int i = 0; i < sensor_fusion.size(); i++)   //////bug5 for loop  need to attention parentheses
         {
            float d = sensor_fusion[i][6];
            if (d< (2+4*lane+2)&& d > (2+4*lane -2))
            {
             double vx = sensor_fusion[i][3];
             double vy = sensor_fusion[i][4];
             double check_speed = sqrt(vx * vx + vy*vy);
             double check_car_s = sensor_fusion[i][5];


             check_car_s +=((double)pre_path_size*.02*check_speed);
            
            
             
             
           
             ////judge distance of front car
             if ((check_car_s > car_s) &&((check_car_s - car_s) < 30))
              {
                //ref_vel = 25; //mph
                too_close = true;
                lane = 0;
              }
 
            }
           
          }


           //////if too close,slowly deacclerate
          if (too_close)
          {
            ref_vel -= .224;
          }
           /////slowly acclerate
          else if(ref_vel < 49.5)
          {
            ref_vel += .224;
          }
          
          
          

          ////creat points vector
          vector<double> pts_x ;
          vector<double> pts_y ;

          ////vehicle state
          double ref_x = car_x ;
          double ref_y = car_y ;
          double ref_yaw = deg2rad(car_yaw) ;

          /////judge pre_path size
          if (pre_path_size < 2){

            /////use two points that make the path tangent to the car
            double car_pre_x = car_x - cos(car_yaw);
            double car_pre_y = car_y - sin(car_yaw);

            pts_x.push_back(car_pre_x);
            pts_x.push_back(car_x);

            pts_y.push_back(car_pre_y);
            pts_y.push_back(car_y);

          }


          ////use previous path's end points as starting reference
          else
          {

            ///// redefine reference state as previous path end point
            ref_x = previous_path_x[pre_path_size - 1];
            ref_y = previous_path_y[pre_path_size - 1];


            double car_pre_second_x = previous_path_x [pre_path_size - 2];
            double car_pre_second_y = previous_path_y [pre_path_size -2 ];

            ref_yaw = atan2(ref_y - car_pre_second_y,ref_x - car_pre_second_x);
            

            ////use two points that make the path tangent to the previous path's and point
            pts_x.push_back(car_pre_second_x);
            pts_x.push_back(ref_x);

            pts_y.push_back(car_pre_second_y);
            pts_y.push_back(ref_y);
          }
          

          ////define three goal point to pts
          vector<double> target1_xy = getXY(car_s +30, (2 + 4 *lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> target2_xy = getXY(car_s +60, (2 + 4 *lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> target3_xy = getXY(car_s +90, (2 + 4 *lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ////add three point to pts
          pts_x.push_back(target1_xy[0]);
          pts_x.push_back(target2_xy[0]);
          pts_x.push_back(target3_xy[0]);

          pts_y.push_back(target1_xy[1]);
          pts_y.push_back(target2_xy[1]);
          pts_y.push_back(target3_xy[1]);

          /////transform pts points to zero degree coordinate /bug1: pts_x.size not pre_path.size
          for (int i = 0; i < pts_x.size(); i++)
          {
             double trans_x = pts_x[i] - ref_x;
             double trans_y = pts_y[i] - ref_y;

             pts_x[i] = (trans_x*cos(0 - ref_yaw) - trans_y*sin(0 - ref_yaw));
             pts_y[i] = (trans_x*sin(0 - ref_yaw) + trans_y*cos(0 -ref_yaw));

            
          }

          ////define spline
          tk::spline s;

          /////set pts to s
          s.set_points(pts_x, pts_y);

          ////define the actual(x, y) we will use for planner
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;


          /////add previous(x,y) to next_x_vals
          for (int i = 0; i < previous_path_x.size(); i++)
          {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
          }
          
          ////calculate how to split up the spline
          double targetx = 30.0;
          double targety = s(targetx);
          double distance = sqrt((targetx)* (targetx) + (targety)* (targety));

          double x_add_on = 0; ////// bug4 :variable type,can't use int here

          /////add previous points to planner
          for (int i = 1; i <= 50- previous_path_x.size(); i++)   ////bug3: under for loop attention identation 
          {
             double N = (distance/(.02* ref_vel/2.24)) ;
             double x_point = x_add_on + (targetx) / N;
             double y_point = s(x_point);
            
             x_add_on = x_point;

             double ref2_x = x_point;  ///bug2 :differ from ref_x ,attention attention
             double ref2_y = y_point;
            
          //rotate back to global coordinate, just reverse that former
             x_point = (ref2_x*cos(ref_yaw) - ref2_y*sin(ref_yaw));
             y_point = (ref2_x*sin(ref_yaw) + ref2_y*cos(ref_yaw));

             x_point += ref_x;
             y_point += ref_y;

             next_x_vals.push_back(x_point);
             next_y_vals.push_back(y_point);

          }
          

          

          

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          /*step1 try try try by myself
          double dist_inc = 0.5;
          for(int i = 0; i < 50; i++)
          {
            double next_s = car_s + (i + 1)*dist_inc;
            double next_d = 6;
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);


            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          */
          //END
          
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