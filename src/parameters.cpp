//
// Created by tsbertalan on 7/15/18.
//
#include "parameters.h"

// PLANNER
int NUM_PLANS = 128;
bool SHOW_COSTS_TABLE = false;
bool LOGGING = false;
bool DEBUG = false;

double MIPH_TO_MPS = (5280/1.)*(1./3.2808)*(1/3600.);

double GOAL_SPEED_MPH = 46;
double MAX_TARGET_SPEED = (GOAL_SPEED_MPH + 1)*MIPH_TO_MPS;
double MIN_TARGET_SPEED = 1;
double MAX_SPEED_DIFFERENCE = 10;
double MIN_SPEED_DIFFERENCE = -MAX_TARGET_SPEED + 4;
double EXT_TIME = 1;

unsigned int NUM_REUSED = 16;
double MIN_DT = 2;

double MAX_DT = 3;
double LANE_DEFINITION_LEFT = 2.1;
double LANE_DEFINITION_CENTER = 6;
double LANE_DEFINITION_RIGHT = 9.8;
// TRAJECTORY GENERATION
bool DO_CACHE = true;

// COST
double FACTOR_DISTANCE = 7;

double CAR_WIDTH = 2.25;

double CAR_LENGTH = 4;
double PENALTY_FATAL = 10;

double DISTANCE_ZERO_COST_FOLLOW = -60;
double DISTANCE_ZERO_COST_LEAD = 20;

double FACTOR_ACCEL = 1./32.;
double FACTOR_JERK = 1./96.;

double FACTOR_ACCEL_EXCESS = 2;
double CRITICAL_ACCEL_EXCESS = 9;

double FACTOR_JERK_EXCESS = 2;
double CRITICAL_JERK_EXCESS = 9;

double GOAL_SPEED = GOAL_SPEED_MPH*MIPH_TO_MPS;

double FACTOR_POSITIVE_SPEED_DEVIATION = 1;
double FACTOR_NEGATIVE_SPEED_DEVIATION = .3;
double FACTOR_VDEV = .4;
double FACTOR_LANE_SW = .01;

double CRITICAL_SWITCHTIME = 1750;

double SCALE_SWITCHTIME = .01;
double FACTOR_FASTSW = .4;
double FACTOR_CRP = .003;

double PENALTY_LANE_LEFT = .1;
double PENALTY_LANE_CENTER = 0;
double PENALTY_LANE_RIGHT = .2;
double PENALTY_LINE_SOLID = 2;
double PENALTY_LINE_DASHED = .5;
double PENALTY_OFF_ROAD = 10;

double BAD_MAP_BEGIN = 4900;
double BAD_MAP_END = 5080;
double PENALTY_BAD_MAP = 2;
double BAD_MAP_DRIFT_LEFT = -.25;

// VISUALIZESR
double COST_VIZ_CAP = 10.;
int NUM_PLANS_VISUALIZED = 20;
double VIS_S_THRESHOLD = CAR_LENGTH*2;
