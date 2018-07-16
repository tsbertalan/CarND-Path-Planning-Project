//
// Created by tsbertalan on 7/15/18.
//
#include "parameters.h"


// PLANNER
int NUM_PLANS = 128;
bool SHOW_ALL_PLANS = false;
bool LOGGING = false;
bool DEBUG = false;
double MAX_TARGET_SPEED = 47;
double MIN_TARGET_SPEED = 1;
double MAX_SPEED_DIFFERENCE = 10;
double MIN_SPEED_DIFFERENCE = -MAX_TARGET_SPEED + 4;
double EXT_TIME = 1;
unsigned int NUM_REUSED = 16;

double MIN_DT = 2;
double MAX_DT = 3;

double MIPH_TO_MPS = (5280 / 1.) * (1. / 3.2808) * (1 / 3600.);


// TRAJECTORY GENERATION
bool DO_CACHE = true;


// COST
double FACTOR_DISTANCE = 5;

double CAR_WIDTH = 3;
double CAR_LENGTH = 6;
double WWARN_RADIUS = CAR_WIDTH - .5;
double LWARN_RADIUS = CAR_LENGTH + 4;

double CRITICAL_DISTANCE_X = LWARN_RADIUS;
double SCALE_DISTANCE_X = .1;

double CRITICAL_DISTANCE_Y = WWARN_RADIUS;
double SCALE_DISTANCE_Y = 5;

double FACTOR_ACCEL = 1. / 24.;
double FACTOR_JERK = 1. / 96.;

double FACTOR_ACCEL_EXCESS = 1;
double CRITICAL_ACCEL_EXCESS = 10;

double FACTOR_JERK_EXCESS = 1;
double CRITICAL_JERK_EXCESS = 10;

double GOAL_SPEED = 44 * MIPH_TO_MPS;
double FACTOR_POSITIVE_SPEED_DEVIATION = 1;
double FACTOR_NEGATIVE_SPEED_DEVIATION = .2;
double FACTOR_VDEV = .3;

double FACTOR_LANE_SW = .01;

double CRITICAL_SWITCHTIME = 500;
double SCALE_SWITCHTIME = .01;
double FACTOR_FASTSW = .4;

double FACTOR_OOL = .001;