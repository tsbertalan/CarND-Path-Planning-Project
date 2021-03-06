//
// Created by tsbertalan on 7/3/18.
//
#include "coordinates.h"

std::vector<double>
get_frenet(
    double x, double y, double theta,
    const std::vector<double> &maps_x, const std::vector<double> &maps_y
) {
  int next_wp = next_waypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp==0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y)/(n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);
  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }
  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Alternative get_frenet that also gives yaw.
std::vector<double>
get_frenet(
    double x, double y, double theta,
    const std::vector<double> &maps_x,
    const std::vector<double> &maps_y,
    const std::vector<double> &maps_dx,
    const std::vector<double> &maps_dy,
    const std::vector<double> &maps_s
) {
  int next_wp = next_waypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp==0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y)/(n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  // Calculate d and see if value is positive or negative by comparing it to a center point
  double frenet_d = distance(x_x, x_y, proj_x, proj_y);
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);
  if (centerToPos <= centerToRef)
    frenet_d *= -1;

  // calculate s value
  double tangent_hyp = sqrt(n_x*n_x + n_y*n_y);
  double frenet_s = maps_s[prev_wp] + proj_norm*tangent_hyp;
  double gamma = atan2(maps_dy[prev_wp], maps_dx[prev_wp]) + pi()/2;
  double frenet_yaw = theta - gamma;

  return {frenet_s, frenet_d, frenet_yaw};
}

std::vector<double> get_world(
    double s, double d, double yaw,
    const std::vector<double> &maps_s,
    const std::vector<double> &maps_x,
    const std::vector<double> &maps_y
) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1)))
    prev_wp++;

  int wp2 = (prev_wp + 1)%maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));

  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);
  double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

  double perpendicular = heading - pi()/2;

  double x = seg_x + d*cos(perpendicular);
  double y = seg_y + d*sin(perpendicular);

  return {x, y, heading + yaw};

}

int closest_waypoint(
    double x, double y,
    const std::vector<double> &maps_x, const std::vector<double> &maps_y
) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  double distances[maps_x.size()];
  //#pragma omp simd
  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    distances[i] = distance(x, y, map_x, map_y);
  }

  for (int i = 0; i < maps_x.size(); i++) {
    double dist = distances[i];
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int
next_waypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y) {

  int closestWaypoint = closest_waypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint==maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

CoordinateTransformer::CoordinateTransformer() {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  // Waypoint map to read from
  const std::string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::vector<double> data_waypoints_x, data_waypoints_y, data_waypoints_s,
      data_waypoints_dx, data_waypoints_dy;

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    data_waypoints_x.push_back(x);
    data_waypoints_y.push_back(y);
    data_waypoints_s.push_back(s);
    data_waypoints_dx.push_back(d_x);
    data_waypoints_dy.push_back(d_y);
  }

  // Add back in the first point to enforce that the spline is a loop.
  data_waypoints_x.push_back(data_waypoints_x[0]);
  data_waypoints_y.push_back(data_waypoints_y[0]);
  data_waypoints_s.push_back(max_s);
  data_waypoints_dx.push_back(data_waypoints_dx[0]);
  data_waypoints_dy.push_back(data_waypoints_dy[0]);

  spline_x.set_points(data_waypoints_s, data_waypoints_x);
  spline_y.set_points(data_waypoints_s, data_waypoints_y);
  spline_dx.set_points(data_waypoints_s, data_waypoints_dx);
  spline_dy.set_points(data_waypoints_s, data_waypoints_dy);

  // Evaluate the spline at a finer level to make a more detailed "map".
  for (double s = 0; s < max_s; s += 1.0) {
    map_waypoints_x.push_back(spline_x(s));
    map_waypoints_y.push_back(spline_y(s));
    map_waypoints_dx.push_back(spline_dx(s));
    map_waypoints_dy.push_back(spline_dy(s));
    map_waypoints_s.push_back(s);
  }
}

void CoordinateTransformer::set_reference(WorldPose car) {
  car_reference = car;
}

WorldPose CoordinateTransformer::to_world(CarPose from) {
  WorldPose wp;

  // Rotation part
  double x = from.x*cos(car_reference.yaw) - from.y*sin(car_reference.yaw);
  double y = from.x*sin(car_reference.yaw) + from.y*cos(car_reference.yaw);

  // Transformation part
  x += car_reference.x;
  y += car_reference.y;
  double yaw = from.yaw + car_reference.yaw;

  wp.x = x;
  wp.y = y;
  wp.yaw = yaw;

  return wp;
}

WorldPose CoordinateTransformer::to_world(FrenetPose from) {

  double s = fmod(from.s, max_s);

  WorldPose wp;

  wp.x = spline_x(s) + from.d*spline_dx(s);
  wp.y = spline_y(s) + from.d*spline_dy(s);

  double theta_antinormal = atan2(spline_dy(s), spline_dx(s));
  double heading = theta_antinormal + pi()/2;
  wp.yaw = heading + from.yaw;

  return wp;
}

CarPose CoordinateTransformer::to_car(WorldPose from) {
  CarPose cp;
  double x = from.x;
  double y = from.y;
  double yaw = from.yaw;

  // Transformation part
  x -= car_reference.x;
  y -= car_reference.y;

  // Rotation part
  cp.x = x*cos(-car_reference.yaw) - y*sin(-car_reference.yaw);
  cp.y = x*sin(-car_reference.yaw) + y*cos(-car_reference.yaw);
  cp.yaw = yaw - car_reference.yaw;

  return cp;
}

CarPose CoordinateTransformer::to_car(FrenetPose from) {
  return to_car(to_world(from));
}

FrenetPose CoordinateTransformer::to_frenet(CarPose from) {
  return to_frenet(to_world(from));
}

FrenetPose CoordinateTransformer::to_frenet(WorldPose from) {
  std::vector<double> sdy = get_frenet(
      from.x, from.y, from.yaw,
      map_waypoints_x, map_waypoints_y,
      map_waypoints_dx, map_waypoints_dy,
      map_waypoints_s
  );

  return {.s=sdy[0], .d=sdy[1], .yaw=sdy[2]};
}

std::vector<std::vector<double>> CoordinateTransformer::get_waypoints() {
  std::vector<std::vector<double>> results;
  results.push_back(map_waypoints_x);
  results.push_back(map_waypoints_y);
  results.push_back(map_waypoints_dx);
  results.push_back(map_waypoints_dy);
  results.push_back(map_waypoints_s);
  return results;
}

double get_world_dist(WorldPose a, WorldPose b) {
  return distance(a.x, a.y, b.x, b.y);
}
