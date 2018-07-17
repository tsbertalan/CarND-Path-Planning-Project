//
// Created by tsbertalan on 7/6/18.
//

#include "decision_making.h"

#include "parameters.h"

CostDecision Planner::get_cost(Trajectory &plan, vector<Neighbor> neighbors, string label, bool heading) {

  vector<const char *> cost_names;
  vector<double> cost_parts;


  // TODO: Reformulate costs as [0,1]-valued functions.

  // TODO: Interleave all time-dependent costs in one loop.


  //// Check whether the plan entails likely collisions.
  // TODO: Analyze and consider side-swipe scenarios.
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
      double ds = fabs(s - other.s);
      double dd = fabs(d - other.d);

      double cost;

      if (
        // Treat collisions as fatal.
        // (not /2 because it's our radius plus their radius (distance between centers))
          ds <= CAR_LENGTH
              && ds >= -CAR_LENGTH
              && dd <= CAR_WIDTH
              && dd >= -CAR_WIDTH
          ) {
        cost = PENALTY_FATAL;

      } else if (dd < -CAR_WIDTH || dd > CAR_WIDTH) {
        // Passing is fine.
        cost = 0;

      } else {
        // Following or being followed is linearly costly with s-distance.
        if (ds > 0)
          cost = max(line(ds, CAR_LENGTH, 1, CAR_LENGTH + DISTANCE_ZERO_COST_LEAD, 0), 0.);
        else
          cost = max(line(ds, -CAR_LENGTH, 1, -CAR_LENGTH + DISTANCE_ZERO_COST_FOLLOW, 0), 0.);
      }

      if (cost > cost_dist) {
        // Take the largest such cost of the whole trajectory.
        cost_dist = cost;
      }
    }
  }
  cost_parts.push_back(cost_dist*FACTOR_DISTANCE);
  cost_names.push_back("dist");


  //// Add up the total acceleration.
  // TODO: Penalize latitudinal (d) and longitudinal (s) acceleration separately, and nonlinearly (increase cost sharply past limits).
  double cost_accel = 0;
  int num_accel = 0;
  double cost_jerk = 0;
  int num_jerk = 0;
  for (double t = 0; t <= plan.t_max(); t += .02) {
    double accel = plan.accel(t);
    if (accel > cost_accel)
      cost_accel = accel;
    double jerk = plan.jerk(t);
    if (jerk > cost_jerk)
      cost_jerk = jerk;
  }

  cost_parts.push_back(cost_accel*FACTOR_ACCEL);
  cost_names.push_back("accel");

  cost_parts.push_back(cost_jerk*FACTOR_JERK);
  cost_names.push_back("jerk");


  // Put a pseudo-boolean cost on trajectories whose average accel exceeds


  //// Find the mean deviation from goal velocity; penalizing larger differences more.
  double cost_vdeviation = plan.speed(plan.t_max()) - GOAL_SPEED;
  if (cost_vdeviation > 0)
    cost_vdeviation *= FACTOR_POSITIVE_SPEED_DEVIATION;
  else
    cost_vdeviation *= -FACTOR_NEGATIVE_SPEED_DEVIATION;
  cost_parts.push_back(cost_vdeviation*FACTOR_VDEV);
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


  // Make a lane-centering and out-of-road cost.
  double cost_road_profile = 0;

  // Construct the cost piecewise from lines.
  for (double t = 0; t <= plan.t_max(); t += .02) {
    double d = plan.d(t);
    double s = plan.s(t);
    // Hard-code in a problem area in the map to be avoided.
    double variable_bad_map_penalty = (expit(s, BAD_MAP_BEGIN, .2) - expit(s, BAD_MAP_END, .2))*PENALTY_BAD_MAP;
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
      cost_road_profile +=
          line(d, 8, PENALTY_LINE_DASHED, LANE_DEFINITION_RIGHT, PENALTY_LANE_RIGHT + variable_bad_map_penalty);
    else if (d < 12)
      cost_road_profile += line(d,
                                LANE_DEFINITION_RIGHT,
                                PENALTY_LANE_RIGHT + variable_bad_map_penalty,
                                12,
                                PENALTY_LINE_SOLID + variable_bad_map_penalty);
    else
      cost_road_profile +=
          line(d, 12, PENALTY_LINE_SOLID + variable_bad_map_penalty, 13, PENALTY_OFF_ROAD + variable_bad_map_penalty);
  }
  cost_parts.push_back(cost_road_profile*FACTOR_CRP);
  cost_names.push_back("CRP");



  //// TODO: Add a max-jerk cost.



  //// TODO: Add a distance-traveled negative cost.



  //// TODO: Add a positive cost for long-time trajectories (dump the vdeviation cost?).



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
