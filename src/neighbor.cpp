//
// Created by tsbertalan on 7/4/18.
//

#include "neighbor.h"

Neighbor::Neighbor(int id, const WorldPose &current, double vx, double vy)
        : id(id), current(current), vx(vx), vy(vy) {}

Neighbor::Neighbor(int id, double x, double y, double vx, double vy)
        : id(id), vx(vx), vy(vy) {
    double yaw = atan2(vy, vx);
    current = {.x=x, .y=y, .yaw=yaw};
}

WorldPose Neighbor::future_position(double dt, CoordinateTransformer &transform) {
    WorldPose next;
    next.x = current.x + dt * vx;
    next.y = current.y + dt * vy;
    next.yaw = current.yaw;
    return next;
}

double Neighbor::speed() {
    return sqrt(vx * vx + vy * vy);
}
