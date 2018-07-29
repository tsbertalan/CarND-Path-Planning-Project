//
// Created by tsbertalan on 7/15/18.
//
#include "parameters.h"



/////////////////////////////////////////////////////
////////// PRINTING, PLOTTING, AND LOGGING //////////
/////////////////////////////////////////////////////

// Should we print out the cost contributions for each candidate path?
bool SHOW_COSTS_TABLE = false;

// Should we log generated paths and other info data to a file?
bool LOGGING = false;

// Should we print decision information on every plan?
bool DEBUG = false;

// Should we generate a live display of some of the candidate plans?
// Requires gnuplot.
bool SHOW_MAP = false;



// Conversion factor from miles per hour to meters per second
double MIPH_TO_MPS = (5280/1.)*(1./3.2808)*(1/3600.);



/////////////////////////////////////////////////////
//////// PLAN SEARCH / TRAJECTORY GENERATION ////////
/////////////////////////////////////////////////////

// How big should our Monte Carlo search be?
int NUM_PLANS = 256;

// Cruise control
double GOAL_SPEED_MPH = 47;

// Lowest speed considered for Monte Carlo search
double MIN_TARGET_SPEED = 1;

// Highest speed considered
double MAX_TARGET_SPEED = (GOAL_SPEED_MPH + 1) * MIPH_TO_MPS;

// Lowest and highest delta from current speed considered
double MIN_SPEED_DIFFERENCE = -MAX_TARGET_SPEED + 1;
double MAX_SPEED_DIFFERENCE = 20;

// Seconds beyond the JMT endpoint to further extend our trajectories for costing
double EXT_TIME = 1;

// Seconds of the previous trajectory before JMT startpoint
double MIN_REUSE_TIME = 32 * .02;

// Shortest and longest JMT duration considered
double MIN_DT = 2;
double MAX_DT = 3;

// Latitudinal (d) positions of the three lanes.
double LANE_DEFINITION_LEFT = 2.1;
double LANE_DEFINITION_CENTER = 6;
double LANE_DEFINITION_RIGHT = 9.8;

// Should we cache plan evaluations?
bool DO_CACHE = true;



/////////////////////////////////////////////////////
/////////////////// COST FUNCTION ///////////////////
/////////////////////////////////////////////////////

// Penalty factor for coming close to or intersecting with other cars
double FACTOR_DISTANCE = 10;

// Car geometry
double CAR_WIDTH = (116. - 10) * 4. / 176.;
double CAR_LENGTH = 214. * 4. / 176.;

// Specific penalty for collisions
double PENALTY_FATAL = 10;

// Distances from other cars at which the cost function goes to zero
double DISTANCE_ZERO_COST_FOLLOW = -6 * CAR_LENGTH;
double DISTANCE_ZERO_COST_LEAD = 3 * CAR_LENGTH;
double DISTANCE_ZERO_COST_BESIDE = .189 * CAR_WIDTH;

// Smooth penalties for acceleration and jerk
double FACTOR_ACCEL = 1./24.;
double FACTOR_JERK = 1./96.;

// Binary penalty for exceeding the speed limit
double FACTOR_SPEED_EXCESS = 4;
double CRITICAL_SPEED_EXCESS = 49.5 * MIPH_TO_MPS;

// Binary penalty for exceeding the accel limit
double FACTOR_ACCEL_EXCESS = 4;
double CRITICAL_ACCEL_EXCESS = 9;

// Smooth penalty function that promotes driving at at the goal speed.
// This function is piecewise linear between
// set costs at particular low, mid, goal, and high speeds.
double FACTOR_VDEV = .5;
double GOAL_SPEED = GOAL_SPEED_MPH*MIPH_TO_MPS;
double SPEED_LIMIT = 50 * MIPH_TO_MPS;
double VDEV_MID = -15 * MIPH_TO_MPS;
double SPEED_COST_ZERO = 2;
double SPEED_COST_MID = 1.5;
double SPEED_COST_LIMIT = 6;

// Penalty for switching lanes at all
double FACTOR_LANE_SW = .01;

// Penalty for switching lanes quickly
double CRITICAL_SWITCHTIME = 3000;
double SCALE_SWITCHTIME = .01;
double FACTOR_FASTSW = .8;

// Cross-road penalty ("CRP") for leaving the road or driving on the lane lines
// We also slightly favor driving in the center lane over the other two.
double FACTOR_CRP = .003;
double PENALTY_LANE_LEFT = .1;
double PENALTY_LANE_CENTER = 0;
double PENALTY_LANE_RIGHT = .1;
double PENALTY_LINE_SOLID = 2;
double PENALTY_LINE_DASHED = .5;
double PENALTY_OFF_ROAD = 10;

// Special fix for one area of the map where driving
// in the right lane is incorrectly marked as off-road.
double BAD_MAP_BEGIN = 4900; // s coordinates [m]
double BAD_MAP_END = 5080;
double BAD_MAP_DRIFT_LEFT = -.25; // d deviation [m]

// Penalty for planning a path to a lane that has cars ahead
// (and therefore which will more likely impede our progress in the future)
double FACTOR_CARS_AHEAD = .5;



/////////////////////////////////////////////////////
/////////////////// VISUALIZATION ///////////////////
/////////////////////////////////////////////////////

// Maximum cost for making cost colormap
double COST_VIZ_CAP = 10.;

// Make plotting run in near-realtime by only showing a random subset of the plans.
int NUM_PLANS_VISUALIZED = 20;

// Don't display neighbors we've passed.
double VIS_S_THRESHOLD = CAR_LENGTH*2;
