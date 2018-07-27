//
// Created by tsbertalan on 7/15/18.
//

#ifndef PATH_PLANNING_PARAMETERS_H
#define PATH_PLANNING_PARAMETERS_H

// PLANNER
extern int NUM_PLANS;
extern bool SHOW_COSTS_TABLE;
extern bool LOGGING;
extern bool DEBUG;

extern double MIPH_TO_MPS;

extern double GOAL_SPEED_MPH;
extern double MAX_TARGET_SPEED;
extern double MIN_TARGET_SPEED;
extern double MAX_SPEED_DIFFERENCE;
extern double MIN_SPEED_DIFFERENCE;
extern double EXT_TIME;

extern double MIN_REUSE_TIME;
extern double MIN_DT;

extern double MAX_DT;
extern double LANE_DEFINITION_LEFT;
extern double LANE_DEFINITION_CENTER;
extern double LANE_DEFINITION_RIGHT;
// TRAJECTORY GENERATION
extern bool DO_CACHE;

// COST
extern double FACTOR_DISTANCE;

extern double CAR_WIDTH;

extern double CAR_LENGTH;
extern double WWARN_RADIUS;
extern double LWARN_RADIUS;
extern double PENALTY_FATAL;

extern double DISTANCE_ZERO_COST_FOLLOW;
extern double DISTANCE_ZERO_COST_LEAD;
extern double DISTANCE_ZERO_COST_BESIDE;

extern double FACTOR_ACCEL;
extern double FACTOR_JERK;

extern double FACTOR_SPEED_EXCESS;
extern double CRITICAL_SPEED_EXCESS;

extern double FACTOR_ACCEL_EXCESS;
extern double CRITICAL_ACCEL_EXCESS;

extern double FACTOR_JERK_EXCESS;
extern double CRITICAL_JERK_EXCESS;

// If goal speed is too close to MAX_TARGET_SPEED,
// we'll be starved for fast-enough trajectories,
// and might drop other criteria.
extern double GOAL_SPEED;

extern double SPEED_LIMIT;
extern double VDEV_MID;
extern double SPEED_COST_ZERO;
extern double SPEED_COST_MID;
extern double SPEED_COST_LIMIT;
extern double FACTOR_VDEV;

extern double FACTOR_LANE_SW;

extern double CRITICAL_SWITCHTIME;

extern double SCALE_SWITCHTIME;
extern double FACTOR_FASTSW;
extern double FACTOR_CRP;

extern double PENALTY_LANE_LEFT;
extern double PENALTY_LANE_CENTER;
extern double PENALTY_LANE_RIGHT;
extern double PENALTY_LINE_SOLID;
extern double PENALTY_LINE_DASHED;
extern double PENALTY_OFF_ROAD;
extern double BAD_MAP_BEGIN;

extern double BAD_MAP_END;
extern double PENALTY_BAD_MAP;
extern double BAD_MAP_DRIFT_LEFT;

extern double COST_VIZ_CAP;
extern int NUM_PLANS_VISUALIZED;
extern double VIS_S_THRESHOLD;

#endif //PATH_PLANNING_PARAMETERS_H
