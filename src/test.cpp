//
// Created by tsbertalan on 7/3/18.
//
#include <iostream>
#include "coordinates.h"
#include <assert.h>

// https://github.com/jal278/plotpipe
#include "graph.h"

#include "decision_making.h"

using namespace std;

int main() {
    CoordinateTransformer transform;

    WorldPose car_reference = {
            .x = 909.5,
            .y = 1128.7,
            .yaw = M_PI * 0.5
    };
    transform.set_reference(car_reference);

    WorldPose test_point = {.x=car_reference.x, .y=car_reference.y + .1, .yaw=M_PI / 4};
    CarPose cp = transform.toCar(test_point);
    WorldPose test_recovery = transform.toWorld(cp);

    assert(test_point.x == test_recovery.x);
    assert(test_point.y == test_recovery.y);
    assert(test_point.yaw == test_recovery.yaw);

    Planner planner(transform);
    Trajectory leftover;
    Trajectory newPlan = planner.make_plan(test_point, 32.0, leftover, {});

    vector<float> xpoints, ypoints, spoints, dpoints, times;

    cout << "sdt=[";
    for (WorldPose pose : newPlan.poses) {
        xpoints.push_back(pose.x);
        ypoints.push_back(pose.y);

        FrenetPose fp = transform.toFrenet(pose);
        spoints.push_back(fp.s);
        dpoints.push_back(fp.d);


        cout << "(" << fp.s << "," << fp.d << "," << fp.yaw << "),";

    }
    cout << "]" << endl;


    cout << "xyt_ret=[";
    for (WorldPose pose : newPlan.poses) {
        WorldPose retransformed = transform.toWorld(
                transform.toFrenet(pose)
        );
        cout << "(" << retransformed.x << "," << retransformed.y << "," << retransformed.yaw << "),";
    }
    cout << "]" << endl;

    for (double t : newPlan.times) {
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

    cout << cp.x << endl;
}