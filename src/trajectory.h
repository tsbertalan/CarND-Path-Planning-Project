//
// Created by tsbertalan on 7/4/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <string>
#include <sstream>
#include <map>
#include "coordinates.h"
#include "jmt.h"
#include "spline.h"

// JMT boundary conditions are state, dstate, and ddstate.
struct State {
  double y, yp, ypp;
};

// Complete state has longitudinal and latitudinal components.
struct FullState {
  State s, d;
};

// Trajectory segment in Frenet coordinates.
//
// A TrajectorySegment is a map from [0, DT) to RxR.
// However, we first map time from [t0, t0+DT)
// to this [0, DT) range.
// The polynomials are defined at DT, but this segment's
// domain ("responsibility") is only half-closed.
class TrajectorySegment {

  PolyTrajectory pt;

 public:

  double remap(double t);

  double t_offset;

  double DT;

  TrajectorySegment(double t0, double DT, FullState begin, FullState end);

  FrenetPose operator()(double t);

  double s_derivative(double t, int order = 1);

  double d_derivative(double t, int order = 1);

};

// Combination of a Trajectory segment and the time
// domain for which it is responsible.
struct SegmentRemit {
  TrajectorySegment f;
  double t_responsible_0, t_responsible_1;
};

// Extendable or choppable trajectory;
// assembled piecewise from polynomial SegmentRemits.
class Trajectory {
 private:
  std::map<double, double> s_cache, d_cache;

  std::vector<SegmentRemit> segments;

  FullState state(double t);

  CoordinateTransformer *transform;

 public:

  double t_max();

  Trajectory(CoordinateTransformer *transform);

  // The pose
  FrenetPose frenet(double t);

  WorldPose world(double t);

  double s(double t, bool ignore_tmax = false);

  double d(double t, bool ignore_tmax = false);

  // Derivatives of the pose
  double sp(double t, bool ignore_tmax = false);

  double dp(double t, bool ignore_tmax = false);

  double spp(double t);

  double dpp(double t);

  double sppp(double t);

  double dppp(double t);

  // Special scalar derivatives of the pose.
  double speed(double t);

  double accel(double t);

  double jerk(double t);

  // Create a copy, and extend it.
  Trajectory generate_extension(
      FrenetPose current, double t_reuse, double t_replan, double DT, double DS, double sp,
      double d, double spp = 0, double dp = 0, double dpp = 0
  );

  // Chop of part of the beginning of this trajectory.
  void cut_start(double t_reuse, double t_replan);

  // Which segment is responsible for a given time?
  SegmentRemit &get_remit(double t);

  // Dump the trajectory in a suitable form for logging.
  std::string dumps();

  // Generate and execute matplotlib Python for displaying the trajectory.
  // Useful in a debugger.
  void plot(double t_reuse = -1, double t_replan = -1);

  // Break the trajectory down into an x list and a y list suitable
  // for passing back to the simulator.
  std::vector<std::vector<double>> decompose(double dt_extension = 0);

  int num_segments();

};

#endif //PATH_PLANNING_TRAJECTORY_H