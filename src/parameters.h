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
extern double MAX_TARGET_SPEED;
extern double MIN_TARGET_SPEED;
extern double MAX_SPEED_DIFFERENCE;
extern double MIN_SPEED_DIFFERENCE;
extern double EXT_TIME;
extern unsigned int NUM_REUSED;

extern double MIN_DT;
extern double MAX_DT;

extern double MIPH_TO_MPS;

// TRAJECTORY GENERATION
extern bool DO_CACHE;

// COST
extern double FACTOR_DISTANCE;

extern double CAR_WIDTH;
extern double CAR_LENGTH;
extern double WWARN_RADIUS;
extern double LWARN_RADIUS;

extern double PENALTY_FATAL;

extern double CRITICAL_DISTANCE_X;
extern double SCALE_DISTANCE_X;

extern double CRITICAL_DISTANCE_Y;
extern double SCALE_DISTANCE_Y;

extern double FACTOR_ACCEL;
extern double FACTOR_JERK;

extern double FACTOR_ACCEL_EXCESS;
extern double CRITICAL_ACCEL_EXCESS;

extern double FACTOR_JERK_EXCESS;
extern double CRITICAL_JERK_EXCESS;

// If goal speed is too close to MAX_SPEED_CONSIDERED,
// we'll be starved for fast-enough trajectories,
// and might drop other criteria.
extern double GOAL_SPEED;
extern double FACTOR_POSITIVE_SPEED_DEVIATION;
extern double FACTOR_NEGATIVE_SPEED_DEVIATION;
extern double FACTOR_VDEV;

extern double FACTOR_LANE_SW;

extern double CRITICAL_SWITCHTIME;
extern double SCALE_SWITCHTIME;
extern double FACTOR_FASTSW;

extern double FACTOR_CRP;
extern double PENALTY_LANE_LEFT;
extern double PENALTY_LANE_RIGHT;
extern double PENALTY_LINE_SOLID;
extern double PENALTY_LINE_DASHED;
extern double PENALTY_OFF_ROAD;

#endif //PATH_PLANNING_PARAMETERS_H
