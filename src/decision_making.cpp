//
// Created by tsbertalan on 7/3/18.
//
#include <map>
#include "decision_making.h"

#include "parameters.h"

using namespace std;
using namespace std::chrono;

vector<vector<double>>
Planner::make_plan(WorldPose current, double current_speed, int num_unused, vector<Neighbor> neighbors,
                   const double dt) {


  // Prepare things.
  long start_planner_ms = now();

  current_speed *= MIPH_TO_MPS;
  transform.set_reference(current);
  current_lane = get_lane(current);

  vector<Trajectory> plans;
  vector<string> plan_names;

  log.set_status(LOGGING);

  double t_reuse = .02*(last_plan_length - num_unused);
  double t_replan = t_reuse + MIN_REUSE_TIME;
  if (last_plan.t_max() < t_replan)
    t_replan = last_plan.t_max();

  log.begin_item(now());


  // Generate multiple plans.
  // TODO: Consider multipart maneuvers, now that we have piecewise-polynomial trajectories.
  FrenetPose current_frenet = transform.to_frenet(current);
  for (int iplan = 0; iplan < NUM_PLANS; iplan++) {

    Trajectory plan = random_plan(current_speed, current_frenet, t_reuse, t_replan, last_plan);

    plan_names.push_back(describe_plan(plan, current_speed, plan.speed(plan.t_max()), plan.t_max()));
    plans.push_back(std::move(plan));
  }


  // Evaluate costs.
  double costs_arr[plans.size()]; // Use arrays to allow for OpenMP.
  CostDecision decisions_arr[plans.size()];
  cost_evaluation_time = now();
  // Unfortunately OpenMP messes up the of the costs table slightly.
#pragma omp parallel for
  for (int i = 0; i < plans.size(); i++) {
    Trajectory plan = plans[i];
    string planName = plan_names[i];
    ostringstream oss;
    string label;
    if (SHOW_COSTS_TABLE) {
      oss << "#" << i << ": " << plan_names[i];
      label = oss.str();
    }
    CostDecision decision = get_cost(plan, neighbors, label, i==0);
    costs_arr[i] = decision.cost;
    decisions_arr[i] = decision;
  }

  vector<double> costs;
  vector<CostDecision> decisions;
  for (int i = 0; i < plans.size(); i++) {
    costs.push_back(costs_arr[i]);
    decisions.push_back(decisions_arr[i]);
  }


  // Choose a plan.
  int best_plan = argmin(costs);
  double lowest_cost = costs[best_plan];
  Trajectory plan = plans[best_plan];
  last_plan = plan;

  if (plan_changes_goal(plan) || SHOW_COSTS_TABLE || DEBUG)
    cout << "Chose plan " << best_plan << endl << "    " << plan_names[best_plan] << endl;
  CostDecision best_dec = decisions[best_plan];

  // Say why we chose.
  if (plan_changes_goal(plan) || SHOW_COSTS_TABLE || DEBUG)
    cout << "    " << declare_reasons(decisions, best_dec) << "." << endl;

  // Record the time that a lane switch was planned.
  if (plan_changes_goal(plan)) {
    last_lane_change_time_ms = now();
  }

  if (SHOW_MAP) show_map(plans, neighbors, costs);

  long end_planner_ms = now();
  if (plan_changes_goal(plan) || DEBUG) {
    long plantime = (end_planner_ms - start_planner_ms);
    cout << " == planner took " << plantime << " [ms];";
    running_total_plantime += plantime;
    count_calls++;
    cout << " running mean is " << running_total_plantime/count_calls << " [ms] ==" << endl;
  }


  // Log the plan.
  if (LOGGING) {
    log("t_reuse", t_reuse);
    log("t_replan", t_replan);
    log("num_unused", num_unused);
    log("last_plan_length", last_plan_length);
    log("plan", plan_names[best_plan], plan);
    double tproj = 0;
    if (plan.t_max() > 0)
      tproj = plan.t_max() - 0;
    log("neighbors", neighbors, tproj);
    vector<vector<double>> sdytpath;
    vector<vector<double>> xyytpath;
    for (double t = 0; t < plan.t_max(); t += .02) {
      FrenetPose fp = plan.frenet(t);
      WorldPose wp = plan.world(t);
      sdytpath.push_back({fp.s, fp.d, fp.yaw, t});
      xyytpath.push_back({wp.x, wp.y, wp.yaw, t});
    }
    log("plan_sdyt", sdytpath);
    log("plan_xyyt", xyytpath);
    log("reasons", declare_reasons(decisions, best_dec));
    log("planner_time", end_planner_ms - start_planner_ms);

    log.end_item();
  }

  goal_lane = get_lane(plan);


  // Evaluate and return the discrete plan.
  auto next_xy_vals = plan.decompose(EXT_TIME);
  last_plan_length = next_xy_vals[0].size();
  return next_xy_vals;
}

Trajectory Planner::random_plan(double current_speed,
                                const FrenetPose current_frenet,
                                double t_reuse,
                                double t_replan,
                                Trajectory &starting_plan) {

  // Choose the final lane and therefore d.
  int target_lane = uniform_random(0, 3);
  double plan_target;
  switch (target_lane) {
    case 0:plan_target = LANE_DEFINITION_LEFT;
      break;
    case 1:plan_target = LANE_DEFINITION_CENTER;
      break;
    case 2:plan_target = LANE_DEFINITION_RIGHT;
      // In the bad region of the map, drift a little to the left.
      plan_target += in_bad_region_pseudobool(current_frenet.s)*BAD_MAP_DRIFT_LEFT;
      break;
  }

  // Choose the final speed.
  double speed_difference = uniform_random(
      max(
          MIN_SPEED_DIFFERENCE,
          MIN_TARGET_SPEED - current_speed
      ),
      min(
          MAX_SPEED_DIFFERENCE,
          MAX_TARGET_SPEED - current_speed
      )

  );
  double target_speed = current_speed + speed_difference;

  // Optionally add some extra reuse time.
  double t_reuse_extra = uniform_random(0., 1.);

  // Choose the duration.
  double DT = uniform_random(MIN_DT, MAX_DT);

  // Make the plan.
  Trajectory plan = starting_plan.generate_extension(
      current_frenet,
      t_reuse,
      t_replan + t_reuse_extra,
      DT,
      -1,
      target_speed,
      plan_target
  );

  return plan;

}

long Planner::now() {
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - construction_time;
}

Planner::Planner(CoordinateTransformer &transform) : transform(transform), log(PyLogger(transform)),
                                                     last_plan(&transform) {
  construction_time = 0;
  construction_time = now();
  last_lane_change_time_ms = 0;
  last_plan_length = 0;
  count_calls = 0;
  running_total_plantime = 0;
}

void Planner::show_map(vector<Trajectory> &plans, vector<Neighbor> neighbors, vector<double> costs) {

  WorldPose ego_now = plans[0].world(0);
  transform.set_reference(ego_now);
  double s_now = transform.to_frenet(ego_now).s;

  vector<vector<double>> S, D, T, C;
  vector<string> styles;

  // Make a random selection.
  vector<int> selected_plans;
  for (int iplan = 0; iplan < NUM_PLANS_VISUALIZED; iplan++) {
    int selection = uniform_random(0, plans.size());
    if (std::find(selected_plans.begin(), selected_plans.end(), selection)==selected_plans.end()) {
      selected_plans.push_back(selection);
    }
  }

  for (int selection : selected_plans) {
    Trajectory plan = plans[selection];
    double cost = costs[selection];
    vector<double> s, d, t, c;
    for (double t = 0; t <= plan.t_max(); t += .02) {
      FrenetPose fp = plan.frenet(t);
      s.push_back(fp.s - s_now);
      d.push_back(fp.d);
    }
    for (double tv = 0; tv <= plan.t_max(); tv += .02) {
      t.push_back(tv);
      c.push_back(min(cost, COST_VIZ_CAP));
    }
    S.push_back(s);
    D.push_back(d);
    T.push_back(t);
    C.push_back(c);
    styles.push_back("lines");
  }

  for (auto n: neighbors) {
    vector<double> s, d, c;

    // Don't process guys we've passed.
    if (n.current_fp.s - s_now < VIS_S_THRESHOLD)
      continue;

    for (double t = 0; t <= plans[0].t_max(); t += .02) {
      FrenetPose other = n.future_position_frenet(t);
      s.push_back(other.s - s_now);
      d.push_back(other.d);
      c.push_back(COST_VIZ_CAP);
    }
    styles.push_back("points");
    S.push_back(s);
    D.push_back(d);
    C.push_back(c);
  }

  ostringstream cost_label;
  cost_label << "plan cost (capped at " << COST_VIZ_CAP << ")";
  map_plot.plot_data(
      S, D,
      "s [m]", "d [m]", styles, "some plans and neighbor projections", cost_label.str().c_str(),
      C
  );

}

double Planner::uniform_random(double low, double high) {
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(low, high);
  return dis(gen);
}

int Planner::uniform_random(int low, int high_plus_one) {
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(low, high_plus_one - 1);
  return dis(gen);
}

bool Planner::plan_changes_goal(Trajectory &plan) {
  int plan_goal_lane = get_lane(plan);
  return plan_goal_lane!=goal_lane;
}

string Planner::describe_plan(Trajectory &plan, double current_speed, double target_speed, double DT) {

  int plan_target = get_lane(plan);

  ostringstream oss;
  if (!plan_changes_goal(plan)) {
    if (goal_lane==current_lane)
      oss << "cruise" << current_lane;
    else
      oss << "cont_lane_transfer" << current_lane << "to" << plan_target;
  } else {
    if (plan_target==current_lane)
      oss << "abort_transfer_keep" << current_lane;
    else
      oss << "begin_lane_transfer" << current_lane << "to" << plan_target;
  }
  oss << " to " << target_speed << " from " << current_speed << " [m/s]";
  oss << " in " << DT << "[s]";
  return oss.str();
}

string Planner::declare_reasons(vector<CostDecision> &decisions, CostDecision &best_dec) {
  map<const char *, int> histogram;
  for (auto d: decisions) {
    if (histogram.find(d.reason)==histogram.end())
      histogram[d.reason] = 0;
    histogram[d.reason] += 1;
  }
  vector<const char *> reasons;
  vector<int> counts;
  for (auto const &x : histogram) {
    reasons.push_back(x.first);
    counts.push_back(x.second);
  }
  vector<unsigned long> reason_prevalences = argsort(counts);
  reverse(reason_prevalences.begin(), reason_prevalences.end());
  const char *primary_reason = reasons[reason_prevalences[0]];
  const char *secondary_reason = primary_reason;
  if (counts.size() > 1)
    secondary_reason = reasons[reason_prevalences[1]];

  double pri_reason_val = best_dec.cost_parts[
      find(best_dec.cost_part_names.begin(), best_dec.cost_part_names.end(), primary_reason)
          - best_dec.cost_part_names.begin()
  ];
  double sec_reason_val = best_dec.cost_parts[
      find(best_dec.cost_part_names.begin(), best_dec.cost_part_names.end(), secondary_reason)
          - best_dec.cost_part_names.begin()
  ];

  ostringstream oss;

  if (best_dec.reason==primary_reason) {
    if (best_dec.reason==secondary_reason) {
      oss << "despite high " << primary_reason << "=" << pri_reason_val;
    } else {
      oss << "because of low " << secondary_reason << "=" << sec_reason_val;
      oss << " and despite high " << primary_reason << "=" << pri_reason_val;
    }

  } else {
    oss << "because of low " << primary_reason << "=" << pri_reason_val;
  }
  return oss.str();
}

int Planner::get_lane(double d) {
  if (d < 4)
    return 0;
  else if (d < 8)
    return 1;
  else
    return 2;
}

int Planner::get_lane(FrenetPose fp) {
  return get_lane(fp.d);
}

int Planner::get_lane(WorldPose wp) {
  return get_lane(transform.to_frenet(wp));
}

int Planner::get_lane(Trajectory &plan, bool final) {
  if (final)
    return get_lane(plan.d(plan.t_max()));
  else
    return get_lane(plan.d(0.));
}
