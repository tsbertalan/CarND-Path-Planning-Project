//
// Created by tsbertalan on 7/3/18.
//
#include <map>
#include "decision_making.h"

using namespace std;
using namespace std::chrono;

Trajectory
Planner::make_plan(WorldPose current, double current_speed, Trajectory leftover, vector<Neighbor> neighbors,
                   const double dt) {



    // Prepare things.
    long start_planner_ms = now();

    current_speed *= MIPH_TO_MPS;
    transform.set_reference(current);
    current_lane = get_lane(current);

    vector<Trajectory> plans;
    vector<string> plan_names;

    int NUM_PLANS = 200;
    const bool SHOW_ALL_PLANS = false;
    const bool LOGGING = true;
    logger.set_status(LOGGING);
    const bool DEBUG = false;
    double MAX_SPEED_CONSIDERED = 49 * MIPH_TO_MPS;
    double MIN_SPEED_CONSIDERED = 5 * MIPH_TO_MPS;
    unsigned int PLAN_LENGTH = 1000;
    double EXT_TIME = 4;
    unsigned int NUM_REUSED = 32;
    const double TAILGATE_BUFFER = 12;

    const double MIN_DT = 2;
    const double MAX_DT = 3;

    logger.begin_item(now());


    // Work with neighbors.
    // Describe the neighbors.
    vector<double> xcdists;
    vector<Neighbor> lane_neighbors;
    for (auto n : neighbors) {
        if (get_lane(n.current_fp) == current_lane) {
            CarPose cp = transform.to_car(n.current_wp);
            lane_neighbors.push_back(n);
            xcdists.push_back(fabs(cp.x));
        }
    }
    if (!lane_neighbors.empty()) {

        long iminc = min_element(xcdists.begin(), xcdists.end()) - xcdists.begin();
        Neighbor closest = lane_neighbors[iminc];
        CarPose closest_cp = transform.to_car(closest.current_wp);

        if (
                xcdists[iminc] < 40
                and xcdists[iminc] > TAILGATE_BUFFER
                and closest_cp.x > 0
                and get_lane(closest.current_fp) == current_lane
                ) {
            // Generate a following trajectory.
            double target_speed = closest.speed();
            int plan_target = get_lane(closest.current_fp);

            Trajectory follow = leftover.subtrajectory(NUM_REUSED + 1, 0, dt);

            WorldPose ultimate;
            if (leftover.empty())
                ultimate = current;
            else
                ultimate = leftover.ultimate();
            double lead_distance = max(
                    closest_cp.x
                    - transform.to_car(ultimate).x, 1.
            );
            if (lead_distance > TAILGATE_BUFFER) {
                lead_distance -= TAILGATE_BUFFER;
            }
            double DT = 2 * lead_distance / (current_speed + target_speed);
            double tmax;
            if (EXT_TIME != -1) {
                tmax = EXT_TIME;
            } else {
                tmax = DT;
                if (!follow.empty())
                    tmax += follow.times[follow.size() - 1];
            }
            follow.JMT_extend(
                    transform,
                    target_speed,
                    PLAN_LENGTH,
                    current,
                    current_speed,
                    tmax,
                    plan_target * 4 + 2,
                    -1,
                    DT
            );
            ostringstream oss;
            oss << "follow_" << closest.id;
            plan_names.push_back(oss.str());
            plans.push_back(follow);
        } else {
            if (NUM_PLANS == 0)
                NUM_PLANS = 32;
        }

    }


    // Generate multiple plans.
    for (int iplan = 0; iplan < NUM_PLANS; iplan++) {

        int plan_target = uniform_random(0, 3);
        double target_speed = uniform_random(MIN_SPEED_CONSIDERED, MAX_SPEED_CONSIDERED);
        double DT = uniform_random(MIN_DT, MAX_DT);
        double DS = -1;

        Trajectory plan = leftover.subtrajectory(NUM_REUSED + 1, 0, dt);
        double tmax;
        if (EXT_TIME != -1) {
            tmax = EXT_TIME;
        } else {
            tmax = DT;
            if (!plan.empty())
                tmax += plan.times[plan.size() - 1];
        }
        plan.JMT_extend(
                transform,
                target_speed,
                PLAN_LENGTH,
                current,
                current_speed,
                tmax,
                plan_target * 4 + 2,
                DS,
                DT
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
    for(int i=0; i<plans.size(); i++) {
        costs.push_back(costsa[i]);
        decisions.push_back(decisionsa[i]);
    }


    // Choose a plan.
    int best_plan = argmin(costs);
    double lowest_cost = costs[best_plan];
    Trajectory plan = plans[best_plan];

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

    FrenetPose fp = transform.to_frenet(current);
    if (DEBUG) {
        cout << "Frenet position: s=" << fp.s << ", d=" << fp.d << ".";
        if (goal_lane != current_lane || plan_changes_goal(plan)) {
            cout << " Goal: " << current_lane << "==>";
            if (plan_changes_goal(plan))
                cout << "(" << goal_lane << "->" << get_lane(plan) << ")";
            else
                cout << goal_lane;
        }
        cout << endl;
    }

    long end_planner_ms = now();
    if (plan_changes_goal(plan) || DEBUG)
        cout << " == planner took " << (end_planner_ms - start_planner_ms) << " [ms] == " << endl;// << endl;


    // Log the plan.
    if(LOGGING) logger("prev", "leftover", leftover.subtrajectory(NUM_REUSED + 1, 0, dt));
    logger("plan", plan_names[best_plan], plan);
    double tproj = 0;
    if(plan.size() > 0)
        tproj = plan.times[plan.size()-1] - plan.times[0];
    logger("neighbors", neighbors, tproj);
    logger("plan_sdt", plan.sdtpath);
    if(LOGGING) logger("reasons", declare_reasons(decisions, best_dec));
    logger("planner_time", end_planner_ms - start_planner_ms);

    logger.end_item();

    goal_lane = get_lane(plan);
    return plan;
}

long Planner::now() {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - construction_time;
}

Planner::Planner(CoordinateTransformer &transform) : transform(transform), logger(PyLogger(transform)){
    construction_time = 0;
    construction_time = now();
    last_lane_change_time_ms = 0;
}

void Planner::show_map(vector<Trajectory> plans, vector<Neighbor> neighbors) {

    WorldPose ego_now = plans[0].initial();

    transform.set_reference(ego_now);

    const double DIST_VIZ_CAP = 30.;

    int iplan = -1;
    vector<vector<double>> X, Y, T, C;
    vector<string> styles;
    for (auto plan: plans) {
        iplan++;
        vector<double> x, y, t, c;
        for (WorldPose pose : plan.poses) {
            CarPose cp = transform.to_car(pose);
            x.push_back(cp.x);
            y.push_back(cp.y);
        }
        for (double tv : plan.times) {
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
        for (int i = 0; i < plans[0].size(); i++) {
            double t0 = plans[0].times[0];
            double t = plans[0].times[i];
            WorldPose other = n.future_position(t - t0);
            CarPose cp = transform.to_car(other);
            x.push_back(cp.x);
            y.push_back(cp.y);
            double min_dist = 9999;
            for (auto p : plans) {
                min_dist = min(min_dist, get_world_dist(other, p.poses[i % p.size()]));
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

bool Planner::plan_changes_goal(Trajectory plan) {
    int plan_goal_lane = get_lane(plan);
    int old_goal_lane = goal_lane;
    return plan_goal_lane != goal_lane;
}

bool Planner::cross_lane_plan(Trajectory plan) {
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

int Planner::get_lane(Trajectory plan, bool final) {
    if (final)
        return get_lane(plan.ultimate());
    else
        return get_lane(plan.initial());
}
