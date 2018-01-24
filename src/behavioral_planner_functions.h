/*
 * behavioral_planner_functions.h
 * Key functions for the behavioral planner
 */

#ifndef BEHAVIORAL_PLANNER_FUNCTIONS_H_
#define BEHAVIORAL_PLANNER_FUNCTIONS_H_

#include <math.h>
#include <vector>
#include "helpers.h"
#include "spline.h"

// constants
const double SAFETY_DISTANCE = 20.0;

// Helper structures
struct behaviour
{
  int lane;
  double speed;
};

struct car_data
{
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

// ----------------- 0. Specific Helper Functions ----------------

// Helper function that returns the closest car forward on any given lane.
car_data closest_in_lane(int lane, double ref_s,
  vector<vector<double>> sensor_fusion) {

  car_data car = {.x=0, .y=0, .s=0, .d=0, .yaw=0, .speed=0};

  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    // Note: we do not measure yaw directly from sensors
    car_data check_car = {
      .x=sensor_fusion[i][1],
      .y=sensor_fusion[i][2],
      .s=sensor_fusion[i][5],
      .d=sensor_fusion[i][6],
      .yaw=0,
      .speed=sqrt(pow(sensor_fusion[i][3],2)+pow(sensor_fusion[i][4],2))
    };

    // If car is in considered lane
    if(check_car.d < (2+4*lane+2) && check_car.d > (2+4*lane-2) && check_car.s - ref_s > 0)
    {
      if(car.d == 0 && car.s == 0) { // i.e. we have not identified any car yet
        car = check_car;
      }
      else if(check_car.s < car.s) { // New car is closer
        car = check_car;
      }
    }
  }

  return car;
}

// ------------- 1. Behavior planning ----------------

// Very simple behavior planning:
// Finite state machine with 3 states:
//    1. Stay in your lane and gradually match target speed (49.5 mph)
//    2. Switch to lane with maximum available space in front
//    3. (if 2. not possible) Stay in your lane and decelerate until front car at safe distance
behaviour behavior_planner_simple(
  car_data car,
  behaviour previous,
  vector<vector<double>> sensor_fusion) {

  bool too_close = false;
  double ref_vel = previous.speed;
  int lane = previous.lane;

  auto car_in_front = closest_in_lane(previous.lane, car.s, sensor_fusion);

  if(car_in_front.d != 0 && car_in_front.s != 0) { // i.e. there is one in front
      // cout << "Closest car in lane at: " << car_in_front.s - car.s << " meters!" << endl;
      if(car_in_front.s - car.s < SAFETY_DISTANCE) too_close = true;
  }
  else {
    // cout << "No car !!" << endl;
  }


  int target_lane = -1;
  double target_free_space = 0;
  car_data car_in_lane;
  for(int i = 0; i < 3; i++){
    if(abs(i - previous.lane) == 1){
      car_in_lane = closest_in_lane(i, car.s - SAFETY_DISTANCE, sensor_fusion); // we check 30 meters behind
      // cout << "===> Dist for lane " << i << ": " << car_in_lane.s - car.s << endl;
      // cout << "===> car in lane s: " << car_in_lane.s << " | car s " << car.s << endl;
      if((car_in_lane.s == 0 && car_in_lane.d == 0) || abs(car_in_lane.s - car.s) > 1.5*SAFETY_DISTANCE) {
        double current_free_space = abs(closest_in_lane(i, car.s, sensor_fusion).s - car.s);
        // cout << "=========> Option | Lane: "  << i << " | Space: " << current_free_space << endl;
        if(target_lane < 0 || current_free_space > target_free_space) {
          target_lane = i;
          target_free_space = current_free_space;
        }
      }
    }
  }
  // cout << "Target Lane: " << target_lane << " | Previous Lane: " << previous.lane << endl;
  if(too_close && target_lane == -1) ref_vel -= (car.speed - car_in_front.speed) / 40.0; // Follow vehicle
  else if(too_close && target_lane != -1) lane = target_lane;   // Switch line
  else if(ref_vel < 49.5) ref_vel += 0.4; //(ref_vel - car.speed) / 40.0; // Accelerate

  behaviour target = {.lane=lane, .speed=ref_vel};
  return target;
  }

// ----------------- 2. Trajectory generation ----------------

// Function taking a target lane and velocity using splines,
// outputs the trajectory (x & y).
// We also take into account the previous trajectory still to be completed
// in order to avoid brutal changes.
vector<vector<double>> generate_trajectory_spline(int lane, double velocity,
  double car_x, double car_y, double car_s, double car_d, double car_speed, double car_yaw,
  vector<double> previous_path_x, vector<double> previous_path_y,
  vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s) {

  // Define actual (x,y) points we will use for the Planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Remaining waypoints from previous path
  int prev_size = previous_path_x.size();

  // 1. Create a list of widely spaced (x,y) waypoints
  //    in order to fit a spline.
  vector<double> ptsx, ptsy;
  // vector<double> ptsy;

  // Reference x, y, yaw states
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);


  // If previous size is almost empty, use car as starting Reference
  if(prev_size < 2)
  {
    // Use 2 points to make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else // use the previous path end point as starting Reference
  {
    // Redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw  = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Use two points that make the path tengent to the previous
    // path end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evently 30m spaced points ahead of starting Reference
  vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Switching to local coordinates
  vector<vector<double>> locals = globalToLocalCoords(ptsx, ptsy, ref_x, ref_y, ref_yaw);
  ptsx = locals[0];
  ptsy = locals[1];

  // Create the spline
  tk::spline s;

  // Set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // Start with all of the previous path points from last time
  for(int i = 0; i < previous_path_x.size(); i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up the spline points so that we travel at our desired ref velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x,2)+pow(target_y, 2));

  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  vector<double> x_points;
  vector<double> y_points;
  for(int i = 1; i <= 50-previous_path_x.size(); i++)
  {
    double N = target_dist / (.02*velocity/2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;
    x_points.push_back(x_point);
    y_points.push_back(y_point);
  }
  // Rotate back to normal (global) coordinates
  auto globals = localToGlobalCoords(x_points, y_points, ref_x, ref_y, ref_yaw);

  next_x_vals.insert(next_x_vals.end(), globals[0].begin(), globals[0].end());
  next_y_vals.insert(next_y_vals.end(), globals[1].begin(), globals[1].end());

  return {next_x_vals, next_y_vals};
}

#endif
