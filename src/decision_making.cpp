//
// Created by tsbertalan on 7/3/18.
//
#include <map>
#include "decision_making.h"

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

    int NUM_PLANS = 128;
    const bool SHOW_ALL_PLANS = true;
    const bool LOGGING = true;
    log.set_status(LOGGING);
    const bool DEBUG = false;
    double MAX_SPEED_DIFFERENCE = 10;
    double MIN_SPEED_DIFFERENCE = -40;
    double MIN_TARGET_SPEED = 1;
    double MAX_TARGET_SPEED = 80;
    double EXT_TIME = 1;
    unsigned int NUM_REUSED = 8;
    const double TAILGATE_BUFFER = 12;

    const double MIN_DT = 2;
    const double MAX_DT = 3;

    double t_reuse = .02 * (last_plan_length - num_unused);
    double t_replan = t_reuse + NUM_REUSED * .02;
    if (last_plan.t_max() < t_replan)
        t_replan = last_plan.t_max();

//    // Consider whether we actually want to replan.
//    if(num_unused > 90) {
//        cout << " === " << endl;
//        cout << t_reuse << "," << t_replan << "," << last_plan.t_max() << endl;
//        cout << t_reuse/.02 << "," << t_replan / .02 << endl;
//        last_plan.cut_start(t_reuse, last_plan.t_max());
//        auto next_xy_vals = last_plan.decompose();
//        last_plan_length = next_xy_vals[0].size();
//        return next_xy_vals;
//    }

    log.begin_item(now());


    // Generate multiple plans.
    FrenetPose current_frenet = transform.to_frenet(current);
    for (int iplan = 0; iplan < NUM_PLANS; iplan++) {
//    for (int iplan = 0; iplan < 1; iplan++) {

        int plan_target = uniform_random(0, 3);
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
        double DT = uniform_random(MIN_DT, MAX_DT);

        // DEBUGGING:
//        int plan_target = 1;
//        double target_speed = 42 * MIPH_TO_MPS;
//        double DT = 2;


        State s_end = {.y=(current_speed + target_speed) / 2 * DT, .yp=target_speed, .ypp=0};
        State d_end = {.y=plan_target * 4. + 2., .yp=0, .ypp=0};


        Trajectory plan = last_plan.generate_extension(
                current_frenet,
                t_reuse,
                t_replan,
                DT,
                -1,
                target_speed,
                plan_target * 4 + 2
        );

        plan_names.push_back(describe_plan(plan, current_speed, target_speed, DT));
        plans.push_back(std::move(plan));
    }


    // Evaluate costs.
    double costsa[plans.size()];
    CostDecision decisionsa[plans.size()];

    //#pragma omp parallel for
    for (int i = 0; i < plans.size(); i++) {
        Trajectory plan = plans[i];
        string planName = plan_names[i];
        ostringstream oss;
        string label;
        if (SHOW_ALL_PLANS) {
            oss << "#" << i << ": " << plan_names[i];
            label = oss.str();
        }
        CostDecision decision = get_cost(plan, neighbors, label, i == 0);
        costsa[i] = decision.cost;
        decisionsa[i] = decision;
    }

    vector<double> costs;
    vector<CostDecision> decisions;
    for (int i = 0; i < plans.size(); i++) {
        costs.push_back(costsa[i]);
        decisions.push_back(decisionsa[i]);
    }


    // Choose a plan.
    int best_plan = argmin(costs);
    double lowest_cost = costs[best_plan];
    Trajectory plan = plans[best_plan];
    last_plan = plan;

    if (plan_changes_goal(plan) || SHOW_ALL_PLANS || DEBUG)
        cout << "Chose plan " << best_plan << endl << "    " << plan_names[best_plan] << endl;
    CostDecision best_dec = decisions[best_plan];

    // Say why we chose.
    if (plan_changes_goal(plan) || SHOW_ALL_PLANS || DEBUG)
        cout << "    " << declare_reasons(decisions, best_dec) << "." << endl;

    // Record the time that a lane switch was planned.
    if (plan_changes_goal(plan)) {
        last_lane_change_time_ms = now();
        cout << " ------------------------ LANE CHANGE ------------------------ " << endl;
    }

    if (DEBUG) show_map(plans, neighbors);

    long end_planner_ms = now();
//    if (plan_changes_goal(plan) || DEBUG)
    cout << " == planner took " << (end_planner_ms - start_planner_ms) << " [ms] == " << endl;// << endl;


    // Log the plan.
    if (LOGGING) {
//        log("prev", "leftover", leftover.subtrajectory(NUM_REUSED + 1, 0, dt));
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

long Planner::now() {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - construction_time;
}

Planner::Planner(CoordinateTransformer &transform) : transform(transform), log(PyLogger(transform)),
                                                     last_plan(&transform) {
    construction_time = 0;
    construction_time = now();
    last_lane_change_time_ms = 0;
    last_plan_length = 0;
}

void Planner::show_map(vector<Trajectory> &plans, vector<Neighbor> neighbors) {

    WorldPose ego_now = plans[0].world(0);

    transform.set_reference(ego_now);

    const double DIST_VIZ_CAP = 30.;

    int iplan = -1;
    vector<vector<double>> X, Y, T, C;
    vector<string> styles;
    for (auto plan: plans) {
        iplan++;
        vector<double> x, y, t, c;
        for (double t = 0; t <= plan.t_max(); t += .02) {
            CarPose cp = transform.to_car(plan.frenet(t));
            x.push_back(cp.x);
            y.push_back(cp.y);
        }
        for (double tv = 0; tv <= plan.t_max(); tv += .02) {
            t.push_back(tv);
            c.push_back(0);
        }
        X.push_back(x);
        Y.push_back(y);
        T.push_back(t);
        C.push_back(c);
        styles.push_back("lines");
    }

    double s_now = transform.to_frenet(ego_now).s;

    for (auto n: neighbors) {
        vector<double> x, y, c;
        for (double t = 0; t <= plans[0].t_max(); t += .02) {
            WorldPose other = n.future_position(t);
            CarPose cp = transform.to_car(other);
            x.push_back(cp.x);
            y.push_back(cp.y);
            double min_dist = 9999;
            for (auto p : plans) {
                min_dist = min(min_dist, get_world_dist(other, p.world(t)));
            }
            c.push_back(min(min_dist, DIST_VIZ_CAP));
        }
        styles.push_back("lines");
        X.push_back(x);
        Y.push_back(y);
        C.push_back(c);
    }

    map_plot.plot_data(
            X, Y,
            "x [m] (car)", "y [m] (car)", styles, "plans and neighbor projections", "capped dist to nearest plan",
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
    int old_goal_lane = goal_lane;
    return plan_goal_lane != goal_lane;
}

bool Planner::cross_lane_plan(Trajectory &plan) {
    int plan_goal_lane = get_lane(plan);
    int old_goal_lane = goal_lane;
    return plan_goal_lane != current_lane;
}

string Planner::describe_plan(Trajectory &plan, double current_speed, double target_speed, double DT) {

//    int plan_target;
//    if(plan.empty())
//        plan_target = get_lane()
    int plan_target = get_lane(plan);

    ostringstream oss;
    if (!plan_changes_goal(plan)) {
        if (goal_lane == current_lane)
            oss << "cruise" << current_lane;
        else
            oss << "cont_lane_transfer" << current_lane << "to" << plan_target;
    } else {
        if (plan_target == current_lane)
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
        if (histogram.find(d.reason) == histogram.end())
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

    if (best_dec.reason == primary_reason) {
        if (best_dec.reason == secondary_reason) {
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
