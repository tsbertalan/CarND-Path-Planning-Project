//
// Created by tsbertalan on 7/3/18.
//
// Some abandoned testing code
//
#include <iostream>
#include "coordinates.h"

// https://github.com/jal278/plotpipe
#include "graph.h"

#include "decision_making.h"

using namespace std;

int main() {
  CoordinateTransformer transform;

  WorldPose car_reference = {
      .x = 909.5,
      .y = 1128.7,
      .yaw = M_PI*0.5
  };
  transform.set_reference(car_reference);

  WorldPose test_point = {.x=car_reference.x, .y=car_reference.y + .1, .yaw=M_PI/4};
  CarPose cp = transform.to_car(test_point);
  WorldPose test_recovery = transform.to_world(cp);

  assert(test_point.x==test_recovery.x);
  assert(test_point.y==test_recovery.y);
  assert(test_point.yaw==test_recovery.yaw);

  Planner planner(transform);
  Trajectory leftover(&transform);
  vector<vector<double>> newPlan = planner.make_plan(test_point, 32.0, 0, {});

  vector<double> xpoints, ypoints, spoints, dpoints, times;

  cout << "sdt=[";
  for (int i = 0; i < newPlan[0].size(); i++) {
    WorldPose pose = {.x=newPlan[0][i], .y=newPlan[1][i]};
    xpoints.push_back(pose.x);
    ypoints.push_back(pose.y);

    FrenetPose fp = transform.to_frenet(pose);
    spoints.push_back(fp.s);
    dpoints.push_back(fp.d);

    cout << "(" << fp.s << "," << fp.d << "," << fp.yaw << "),";

  }
  cout << "]" << endl;

  cout << "xyt=[";
  for (int i = 0; i < newPlan[0].size(); i++) {
    WorldPose pose({.x=newPlan[0][i], .y=newPlan[1][i]});
    cout << "(" << pose.x << "," << pose.y << "," << pose.yaw << "),";
  }
  cout << "]" << endl;

  cout << "xyt_ret=[";
  for (int i = 0; i < newPlan[0].size(); i++) {
    WorldPose pose = {.x=newPlan[0][i], .y=newPlan[1][i]};
    WorldPose retransformed = transform.to_world(
        transform.to_frenet(pose)
    );
    cout << "(" << retransformed.x << "," << retransformed.y << "," << retransformed.yaw << "),";
  }
  cout << "]" << endl;

  for (double t = 0; t < newPlan[0].size()*.02; t += .02) {
    times.push_back(t);
  }

//    plot xplot_window;
//    xplot_window.plot_data(times, xpoints, "points", "x vs t");
//
//    plot yplot_window;
//    yplot_window.plot_data(times, ypoints, "points", "y vs t");

  plot xyplot_window;
  xyplot_window.plot_data(xpoints, ypoints, "points", "y vs x");

//    plot splot_window;
//    splot_window.plot_data(times, spoints, "points", "s vs t");
//
//    plot dplot_window;
//    dplot_window.plot_data(times, dpoints, "points", "d vs t");

  plot sdplot_window;
  sdplot_window.plot_data(spoints, dpoints, "points", "d vs s (after intermediate xy form)");

}