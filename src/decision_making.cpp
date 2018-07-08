//
// Created by tsbertalan on 7/3/18.
//
#include <map>
#include "decision_making.h"

using namespace std;
using namespace std::chrono;

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


Trajectory
Planner::make_plan(
        WorldPose current,
        double current_speed,
        Trajectory leftover,
        vector<Neighbor> neighbors,
        const double dt
) {

    pyfile << "data[" << now() << "] = dict(" << endl;

    // Prepare things.
    long start_planner_ms = now();

    current_speed *= MIPH_TO_MPS;
    transform.set_reference(current);
    current_lane = get_lane(current);


    if(current_speed > 40)
        cout << "Current speed seems rather high." << endl;


    vector<Trajectory> plans;
    vector<string> plan_names;

    int NUM_PLANS = 64;
    const bool SHOW_ALL_PLANS = false;
    const bool DEBUG = false;
    double MAX_SPEED_CONSIDERED = 48 * MIPH_TO_MPS;
    double MIN_SPEED_CONSIDERED = 5 * MIPH_TO_MPS;
    unsigned int PLAN_LENGTH = 1000;
    double EXT_TIME = 2;
    unsigned int NUM_REUSED = 8;
    const double TAILGATE_BUFFER = 8;


    // Work with neighbors.
    // Describe the neighbors.
    vector<double> xcdists;
    vector<Neighbor> lane_neighbors;
    for (auto n : neighbors) {
        if (get_lane(n.current) == current_lane) {
            CarPose cp = transform.to_car(n.current);
            lane_neighbors.push_back(n);
            xcdists.push_back(fabs(cp.x));
        }
    }
    if (!lane_neighbors.empty()) {

        long iminc = min_element(xcdists.begin(), xcdists.end()) - xcdists.begin();
        Neighbor closest = lane_neighbors[iminc];
        CarPose closest_cp = transform.to_car(closest.current);
        cout << "Closest-in-xc neighbor in same lane is #" << closest.id << " at " << xcdists[iminc] << "[m]";
        cout << " (lane=" << get_lane(closest.current) << ", xc=" << closest_cp.x << ", yc=" << closest_cp.y << ")."
             << endl;

        if (
                xcdists[iminc] < 40
                and xcdists[iminc] > TAILGATE_BUFFER
                and closest_cp.x > 0
                and get_lane(closest.current) == current_lane
                ) {
            // Generate a following trajectory.
            double target_speed = closest.speed();
            int plan_target = get_lane(closest.current);

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
            cout << "Added " << oss.str() << endl;
        } else {
            if (NUM_PLANS == 0)
                NUM_PLANS = 32;
        }

    }


    // Generate multiple plans.
    for (int iplan = 0; iplan < NUM_PLANS; iplan++) {

        int plan_target = uniform_random(0, 3);
//        int plan_target = 0;

//        double target_speeds[] = {10, 20, 30};
//        double target_speed = target_speeds[uniform_random(0, 3)];
        double target_speed = uniform_random(MIN_SPEED_CONSIDERED, MAX_SPEED_CONSIDERED);
//        double target_speed = 40 * MIPH_TO_MPS;

//        double DTs[] = {.75, 1};
//        double DT = DTs[uniform_random(0, 2)];
        double DT = uniform_random(.75, 1.5);
//        double DT = .8;

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
        plans.push_back(plan);
    }


    // Evaluate costs.
    vector<CostDecision> decisions;
    vector<double> costs;
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
        costs.push_back(decision.cost);
        decisions.push_back(decision);
    }


    // Choose a plan.
    int best_plan = argmin(costs);
    double lowest_cost = costs[best_plan];
    Trajectory plan = plans[best_plan];
    cout << "Chose plan " << best_plan << endl << "    " << plan_names[best_plan] << endl;
    CostDecision best_dec = decisions[best_plan];

    pyfile << "desc = '" << plan_names[best_plan] << "'," << endl;


    // Print the plan.
    Trajectory tjs[] = {leftover.subtrajectory(NUM_REUSED + 1, 0, dt), plan};

    for(int i=0; i<2; i++) {
        Trajectory tj = tjs[i];
        string tag;
        if(i)
            tag = "";
        else
            tag = "_prev";

        pyfile << "x" << tag << " = [";
        for(WorldPose p : tj.poses)
            pyfile << p.x << ",";
        pyfile << "]," << endl;

        pyfile << "y" << tag << " = [";
        for(WorldPose p : tj.poses)
            pyfile << p.y << ",";
        pyfile << "]," << endl;

        pyfile << "t" << tag << " = [";
        for(double t : tj.times)
            pyfile << t << ",";
        pyfile << "]," << endl;

        pyfile << "s" << tag << " = [";
        for(WorldPose p : tj.poses) {
            FrenetPose fp = transform.to_frenet(p);
            pyfile << fp.s << ",";
        }
        pyfile << "]," << endl;

        pyfile << "d" << tag << " = [";
        for(WorldPose p : tj.poses) {
            FrenetPose fp = transform.to_frenet(p);
            pyfile << fp.d << ",";
        }
        pyfile << "]," << endl;
    }

    pyfile << "neighbors = [";
    for(auto n : neighbors) {
        WorldPose p = n.current;
        pyfile << "(" << p.x << "," << p.y << "," << n.vx << "," << n.vy <<"),";
    }
    pyfile << "]," << endl;





    // Say why we chose.
    cout << "    " << declare_reasons(decisions, best_dec) << "." << endl;


    // Record the time that a lane switch was planned.
    if (plan_changes_goal(plan)) {
        last_lane_change_time_ms = now();
        cout << " ------------------------ LANE CHANGE ------------------------ " << endl;
    }


    if (DEBUG) show_map(plans, neighbors);

    FrenetPose fp = transform.to_frenet(current);
    cout << "Frenet position: s=" << fp.s << ", d=" << fp.d << ".";
    if (goal_lane != current_lane || plan_changes_goal(plan)) {
        cout << " Goal: " << current_lane << "==>";
        if (plan_changes_goal(plan))
            cout << "(" << goal_lane << "->" << get_lane(plan) << ")";
        else
            cout << goal_lane;
    }
    cout << endl;

    long end_planner_ms = now();
    cout << " == planner took " << (end_planner_ms - start_planner_ms) << " [ms] == " << endl;// << endl;

    pyfile << ")" << endl;

    goal_lane = get_lane(plan);
    return plan;
}

long Planner::now() {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - construction_time;
}

Planner::Planner(CoordinateTransformer &transform) : transform(transform) {
    construction_time = 0;
    construction_time = now();
    last_lane_change_time_ms = 0;
    pyfile = ofstream("data.py");
    pyfile << "data = {}" << endl;
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
            WorldPose other = n.future_position(t - t0, transform);
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
            oss << "despite " << primary_reason << "=" << pri_reason_val;
        } else {
            oss << "because of " << secondary_reason << "=" << sec_reason_val;
            oss << " and despite " << primary_reason << "=" << pri_reason_val;
        }

    } else {
        oss << "because of " << primary_reason << "=" << pri_reason_val;
    }
    return oss.str();
}
