//
// Created by tsbertalan on 7/4/18.
//

#include "neighbor.h"

Neighbor::Neighbor(CoordinateTransformer &transform, int id, const WorldPose &current, double vx, double vy)
    : id(id), current_wp(current), transform(transform), vx(vx), vy(vy) {
  current_fp = transform.to_frenet(current);
  const double dt = .02;
  WorldPose next = {.x=current.x + vx*dt, .y=current.y + vy*dt, .yaw=current.yaw};
  FrenetPose next_fp = transform.to_frenet(next);
  vs = (next_fp.s - current_fp.s)/dt;
  vd = (next_fp.d - current_fp.d)/dt;
}

Neighbor::Neighbor(CoordinateTransformer &transform, int id, double x, double y, double vx, double vy)
    : id(id), transform(transform), vx(vx), vy(vy) {
  double yaw = atan2(vy, vx);
  current_wp = {.x=x, .y=y, .yaw=yaw};
  current_fp = transform.to_frenet(current_wp);
  const double dt = .02;
  WorldPose next = {.x=current_wp.x + vx*dt, .y=current_wp.y + vy*dt, .yaw=current_wp.yaw};
  FrenetPose next_fp = transform.to_frenet(next);
  vs = (next_fp.s - current_fp.s)/dt;
  vd = (next_fp.d - current_fp.d)/dt;
}

WorldPose Neighbor::future_position(double dt) {
  FrenetPose next = future_position_frenet(dt);
  return transform.to_world(next);
}

FrenetPose Neighbor::future_position_frenet(double dt) {
  // Compute future position by projecting a straight line in Frenet coordinates.
  FrenetPose next;
  next.s = current_fp.s + dt*vs;
  next.d = current_fp.d + dt*vd;
  next.yaw = 0;
  return next;
}

double Neighbor::speed() {
  return sqrt(vs*vs + vd*vd);
}
