//
// Created by tsbertalan on 7/15/18.
//

#ifndef PATH_PLANNING_PARAMETERS_H
#define PATH_PLANNING_PARAMETERS_H

// PLANNER
extern int NUM_PLANS;
extern bool SHOW_ALL_PLANS; // = false;
extern bool LOGGING; // = false;
extern bool DEBUG; // = false;
extern double MAX_TARGET_SPEED; // = 47;
extern double MIN_TARGET_SPEED; // = 1;
extern double MAX_SPEED_DIFFERENCE; // = 10;
extern double MIN_SPEED_DIFFERENCE; // = -MAX_TARGET_SPEED + 4;
extern double EXT_TIME; // = 1;
extern unsigned int NUM_REUSED; // = 16;
extern double TAILGATE_BUFFER; // = 12;

extern double MIN_DT; // = 2;
extern double MAX_DT; // = 3;

extern double MIPH_TO_MPS; // = (5280 / 1.) * (1. / 3.2808) * (1 / 3600.);



// TRAJECTORY GENERATION
extern bool DO_CACHE; // = true;



// COST
extern double FACTOR_DISTANCE; // = 5;

extern double CAR_WIDTH; // = 3;
extern double CAR_LENGTH; // = 6;
extern double WWARN_RADIUS; // = CAR_WIDTH - .5;
extern double LWARN_RADIUS; // = CAR_LENGTH + 4;

extern double CRITICAL_DISTANCE_X; // = LWARN_RADIUS;
extern double SCALE_DISTANCE_X; // = .1;
extern double PENALTY_FATAL;

extern double CRITICAL_DISTANCE_Y; // = WWARN_RADIUS;
extern double SCALE_DISTANCE_Y; // = 5;

extern double FACTOR_ACCEL; // = 1. / 24.;
extern double FACTOR_JERK; // = 1. / 96.;

extern double FACTOR_ACCEL_EXCESS; // = 1;
extern double CRITICAL_ACCEL_EXCESS; // = 10;

extern double FACTOR_JERK_EXCESS; // = 1;
extern double CRITICAL_JERK_EXCESS; // = 10;

// If goal speed is too close to MAX_SPEED_CONSIDERED,
// we'll be starved for fast-enough trajectories,
// and might drop other criteria.
extern double GOAL_SPEED; // = 44 * MIPH_TO_MPS;
extern double FACTOR_POSITIVE_SPEED_DEVIATION; // = 1;
extern double FACTOR_NEGATIVE_SPEED_DEVIATION; // = .2;
extern double FACTOR_VDEV; // = .3;

extern double FACTOR_LANE_SW; // = .01;

extern double CRITICAL_SWITCHTIME; // = 500;
extern double SCALE_SWITCHTIME; // = .01;
extern double FACTOR_FASTSW; // = .4;

extern double FACTOR_OOL; // = .001;

#endif //PATH_PLANNING_PARAMETERS_H
