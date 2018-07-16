//
// Created by tsbertalan on 7/15/18.
//
#include "parameters.h"

// PLANNER
int NUM_PLANS = 128;
bool SHOW_COSTS_TABLE = false;
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

double LANE_DEFINITION_LEFT = 2.1;
double LANE_DEFINITION_CENTER = 6;
double LANE_DEFINITION_RIGHT = 9.9;

double MIPH_TO_MPS = (5280/1.)*(1./3.2808)*(1/3600.);

// TRAJECTORY GENERATION
bool DO_CACHE = true;

// COST
double FACTOR_DISTANCE = 5;

double CAR_WIDTH = 2.5;
double CAR_LENGTH = 4;

double PENALTY_FATAL = 10;

double SCALE_DISTANCE_X = .1;

double FACTOR_ACCEL = 1./45.;
double FACTOR_JERK = 1./96.;

double FACTOR_ACCEL_EXCESS = 1;
double CRITICAL_ACCEL_EXCESS = 10;

double FACTOR_JERK_EXCESS = 1;
double CRITICAL_JERK_EXCESS = 10;

double GOAL_SPEED = 46*MIPH_TO_MPS;
double FACTOR_POSITIVE_SPEED_DEVIATION = 1;
double FACTOR_NEGATIVE_SPEED_DEVIATION = .2;
double FACTOR_VDEV = .4;

double FACTOR_LANE_SW = .01;

double CRITICAL_SWITCHTIME = 500;
double SCALE_SWITCHTIME = .01;
double FACTOR_FASTSW = .4;

double FACTOR_CRP = .003;
double PENALTY_LANE_LEFT = .25;
double PENALTY_LANE_RIGHT = .25;
double PENALTY_LINE_SOLID = 2;
double PENALTY_LINE_DASHED = .5;
double PENALTY_OFF_ROAD = 10;
