//
// Created by tsbertalan on 7/6/18.
//

#include "decision_making.h"

#include "parameters.h"

CostDecision Planner::get_cost(Trajectory &plan, vector<Neighbor> neighbors, string label, bool heading) {

  vector<const char *> cost_names;
  vector<double> cost_parts;


  // TODO: Interleave all time-dependent costs in one loop.


  //// Check whether the plan entails likely collisions.
  // TODO: Put neighbor loop inside time loop, for interleaving with other time-dependent costs.
  // TODO: Apply a discount factor for cost borne further in the future.
  double cost_dist = 0;
  int i_neighbor = -1;
  for (auto neighbor : neighbors) {
    i_neighbor++;

    for (double t = 0; t <= plan.t_max(); t += .02) {
      double s = plan.s(t);
      double d = plan.d(t);
      FrenetPose other = neighbor.future_position_frenet(t);
      double ds = s - other.s;
      double dd = d - other.d;

      double cost = 1;

      if (
        // Treat collisions as fatal.
        // (not /2 because it's our radius plus their radius (distance between centers))
          ds <= CAR_LENGTH
              && ds >= -CAR_LENGTH
              && dd <= CAR_WIDTH
              && dd >= -CAR_WIDTH
          ) {
        cost = PENALTY_FATAL;

      } else {
        // Passing is mostly fine.
        cost *= max(0., min(1.,
                            line(dd, -CAR_WIDTH - DISTANCE_ZERO_COST_BESIDE, 0, -CAR_WIDTH, 1)
        ));
        cost *= max(0., min(1.,
                            line(dd, CAR_WIDTH + DISTANCE_ZERO_COST_BESIDE, 0, CAR_WIDTH, 1)
        ));

        // Following or being followed is linearly costly with s-distance.
        if (ds > 0)
          cost *= max(line(ds, CAR_LENGTH, 1, CAR_LENGTH + DISTANCE_ZERO_COST_LEAD, 0), 0.);
        else
          cost *= max(line(ds, -CAR_LENGTH, 1, -CAR_LENGTH + DISTANCE_ZERO_COST_FOLLOW, 0), 0.);
      }

      if (cost > cost_dist) {
        // Take the largest such cost of the whole trajectory.
        cost_dist = cost;
      }
    }
  }
  cost_parts.push_back(cost_dist*FACTOR_DISTANCE);
  cost_names.push_back("dist");


  //// Find the mean deviation from goal velocity; penalizing larger differences more.
  double dv = plan.speed(plan.t_max()) - GOAL_SPEED;
  double cost_v_deviation;
  if (dv < 0) {
    if (dv < VDEV_MID)
      cost_v_deviation = line(dv, VDEV_MID, SPEED_COST_MID, -GOAL_SPEED, SPEED_COST_ZERO);
    else
      cost_v_deviation = line(dv, 0, 0, VDEV_MID, SPEED_COST_MID);
  } else
    cost_v_deviation = line(dv, 0, 0, SPEED_LIMIT - GOAL_SPEED, SPEED_COST_LIMIT);

  cost_parts.push_back(max(cost_v_deviation*FACTOR_VDEV, 0.));
  cost_names.push_back("vdev");


  //// Penalize any lane shifts.
  int plan_goal_lane = get_lane(plan);
  cost_parts.push_back(abs(goal_lane - plan_goal_lane)*FACTOR_LANE_SW);
  cost_names.push_back("sw");


  //// Penalise quickly repeated lane shifts.
  double cost_fastsw = 0;
  if (goal_lane!=plan_goal_lane) {
    cost_fastsw = expit(cost_evaluation_time - last_lane_change_time_ms, CRITICAL_SWITCHTIME, -SCALE_SWITCHTIME);
  }
  cost_parts.push_back(cost_fastsw*FACTOR_FASTSW);
  cost_names.push_back("fastsw");


  // Make a lane-centering and out-of-road cost (CRP).
  double cost_road_profile = 0;


  // Make boolean costs for exceeding limits.
  double max_speed = 0;
  double max_accel = 0;
  double max_jerk = 0;

  // Construct the CRP cost piecewise from lines.
  for (double t = 0; t <= plan.t_max(); t += .02) {
    double d = plan.d(t);
    double s = plan.s(t);
    // Hard-code in a problem area in the map to be avoided.
    double variable_right_lane_definition = in_bad_region_pseudobool(s)*BAD_MAP_DRIFT_LEFT + LANE_DEFINITION_RIGHT;
    if (d < 0)
      cost_road_profile += line(d, -1, PENALTY_OFF_ROAD, 0, PENALTY_LINE_SOLID);
    else if (d < 2)
      cost_road_profile += line(d, 0, PENALTY_LINE_SOLID, LANE_DEFINITION_LEFT, PENALTY_LANE_LEFT);
    else if (d < 4)
      cost_road_profile += line(d, LANE_DEFINITION_LEFT, PENALTY_LANE_LEFT, 4, PENALTY_LINE_DASHED);
    else if (d < 6)
      cost_road_profile += line(d, 4, PENALTY_LINE_DASHED, LANE_DEFINITION_CENTER, PENALTY_LANE_CENTER);
    else if (d < 8)
      cost_road_profile += line(d, LANE_DEFINITION_CENTER, PENALTY_LANE_CENTER, 8, PENALTY_LINE_DASHED);
    else if (d < 10)
      cost_road_profile += line(d, 8, PENALTY_LINE_DASHED, variable_right_lane_definition, PENALTY_LANE_RIGHT);
    else if (d < 12)
      cost_road_profile += line(d, variable_right_lane_definition, PENALTY_LANE_RIGHT, 12, PENALTY_LINE_SOLID);
    else
      cost_road_profile += line(d, 12, PENALTY_LINE_SOLID, 13, PENALTY_OFF_ROAD);

    // Track the maximum speed, acceleration, and jerk.
    max_speed = max(plan.speed(t), max_speed);
    max_accel = max(fabs(plan.accel(t)), max_accel);
    max_jerk = max(fabs(plan.jerk(t)), max_jerk);

  }
  cost_parts.push_back(cost_road_profile*FACTOR_CRP);
  cost_names.push_back("CRP");


  //// Add a binary max-speed cost.
  cost_parts.push_back((double) (max_speed > CRITICAL_SPEED_EXCESS)*FACTOR_SPEED_EXCESS);
  cost_names.push_back("maxspd");


  //// Add a binary max-accel cost.
  cost_parts.push_back((double) (max_accel > CRITICAL_ACCEL_EXCESS)*FACTOR_ACCEL_EXCESS);
  cost_names.push_back("maxaccel");


  //// Add a smooth max-accel cost.
  // TODO: Penalize latitudinal (d) and longitudinal (s) acceleration separately, and nonlinearly (increase cost sharply past limits).
  cost_parts.push_back(max_accel*FACTOR_ACCEL);
  cost_names.push_back("accel");


  //// Add a smooth max-jerk cost.
  cost_parts.push_back(max_jerk*FACTOR_JERK);
  cost_names.push_back("jerk");


  //// Add a binary cost if there are neighbors ahead in the target lane.
  bool cars_ahead = false;
  for (auto neighbor : neighbors) {
    // If the neighbor is ahead of us, but not too far...
    if (neighbor.current_fp.s > plan.s(0) && neighbor.current_fp.s < plan.s(plan.t_max())) {
      if (get_lane(neighbor.current_fp)==get_lane(plan)) {
        cars_ahead = true;
      }
    }
  }
  cost_parts.push_back(FACTOR_CARS_AHEAD*cars_ahead);
  cost_names.push_back("ahead");



  // TODO: Add a distance-traveled negative cost.



  // TODO: Add a positive cost for long-time trajectories (dump the vdeviation cost?).



  ////// Total the cost.
  double cost = 0;
  for (double cp : cost_parts)
    cost += cp;

  // Print the parts.
  int print_width = 12;

  if (label.length() > 0 && heading) {
    cout << "           ";
    for (const char *n : cost_names) {
      printf("%*s", print_width, n);
    }
    cout << endl;
  }

  int largest_component = argmax(cost_parts);
  const char *reason = cost_names[largest_component];
  if (label.length() > 0) {
    cout << "cost = sum(";
    const char *fmt = "%*.3f";
    for (double cp : cost_parts) {
      printf(fmt, print_width - 1, cp);
      if (cp==cost_parts[largest_component])
        printf("*");
      else
        printf(" ");
    }
    cout << ") =";
    printf(fmt, print_width, cost);
    cout << " for " << label << "." << endl;
  }

  CostDecision cd = {.cost=cost, .reason=reason, .cost_parts=cost_parts, .cost_part_names=cost_names};
  return cd;
}
double Planner::in_bad_region_pseudobool(double s) const {
  return expit(s, BAD_MAP_BEGIN, .2) - expit(s, BAD_MAP_END, .2);
}
