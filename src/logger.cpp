//
// Created by tsbertalan on 7/8/18.
//
#include "logger.h"
using namespace std;

PyLogger::PyLogger(CoordinateTransformer &transform, string filePath) : transform(transform) {

  // Our "logfile" will be an importable python module.
  pyfile = ofstream("data.py");
  pyfile.precision(10);
  logging = true;

  // Lots of the data will be explicit NumPy arrays.
  pyfile << "import numpy as np" << endl;

  // The only object in the module will be a big data dictionary.
  pyfile << "data = {}" << std::endl;

  // By deafult, we'll dump the map waypoints into the datafile for convenience.
  begin_item("map");
  vector<vector<double>> waypoints = transform.get_waypoints();
  (*this)("map_x", waypoints[0]);
  (*this)("map_y", waypoints[1]);
  (*this)("map_dx", waypoints[2]);
  (*this)("map_dy", waypoints[3]);
  (*this)("map_s", waypoints[4]);

  // We'll also dump the lane divider lines,
  // since we can compute them easily here in C++ with our transform object.
  vector<vector<vector<double>>> lines;
  double offset = -12;
  for (int i = 0; i < 7; i++) {
    if (i==3) {
      offset += 4;
      continue;
    }
    vector<vector<double>> line;
    for (double s : waypoints[4]) {
      WorldPose p1 = transform.to_world(FrenetPose({.s=s, .d=offset, .yaw=0}));
      line.push_back({p1.x, p1.y});
    }
    lines.push_back(line);
    offset += 4;
  }
  (*this)("map_lines", lines);
  end_item();
}

void PyLogger::begin_item(long tag) {
  if (!logging) return;
  pyfile << "data[" << tag << "] = dict(" << endl;
}

void PyLogger::begin_item(std::string tag) {
  if (!logging) return;
  pyfile << "data['" << tag << "'] = dict(" << endl;
}

void PyLogger::end_item() {
  if (!logging) return;
  pyfile << ")" << endl;
}

void PyLogger::operator()(std::string name, std::string data) {
  if (!logging) return;
  pyfile << name << " = '" << data << "'," << endl;
}

void PyLogger::operator()(std::string name, std::string desc, Trajectory tj) {
  if (!logging) return;

  pyfile << name << " = dict(" << endl;

  pyfile << "dumps = " << tj.dumps() << "," << endl;

  (*this)("desc", desc);

  pyfile << ")," << endl;
}

void PyLogger::operator()(std::string name, std::vector<Neighbor> neighbors, double tproj) {
  if (!logging) return;
  pyfile << name << " = [";
  for (auto n : neighbors) {
    WorldPose p = n.current_wp;
    pyfile << "(" << n.id << "," << p.x << "," << p.y << "," << n.vx << "," << n.vy << ",";
    // Optionally additionally log the neighbor's predicted future location.
    if (tproj > 0) {
      WorldPose fp = n.future_position(tproj);
      pyfile << fp.x << "," << fp.y << ",";
    }
    pyfile << "),";
  }
  pyfile << "]," << endl;
}

void PyLogger::operator()(std::string name, std::vector<double> vec) {
  if (!logging) return;
  pyfile << name << " = np.array([";
  for (double x : vec) {
    pyfile << x << ",";
  }
  pyfile << "])," << endl;
}

void PyLogger::operator()(std::string name, std::vector<std::vector<double>> arr) {
  if (!logging) return;

  pyfile << name << " = np.array([";
  for (vector<double> vec : arr) {
    pyfile << "[";
    for (double x : vec) {
      pyfile << x << ",";
    }
    pyfile << "], ";
  }
  pyfile << "])," << endl;
}

void PyLogger::operator()(std::string name, std::vector<std::vector<std::vector<double>>> arr) {
  if (!logging) return;
  pyfile << name << " = [";
  for (vector<vector<double>> sheet : arr) {
    pyfile << "[";
    for (vector<double> vec : sheet) {
      pyfile << "[";
      for (double x : vec) {
        pyfile << x << ",";
      }
      pyfile << "], ";
    }
    pyfile << "]," << endl;
  }
  pyfile << "]," << endl;
}

void PyLogger::operator()(std::string name, long int data) {
  if (!logging) return;
  pyfile << name << " = " << data << "," << endl;
}

void PyLogger::operator()(std::string name, int data) {
  if (!logging) return;
  pyfile << name << " = " << data << "," << endl;
}

void PyLogger::operator()(std::string name, double data) {
  if (!logging) return;
  pyfile << name << " = " << data << "," << endl;
}

bool PyLogger::set_status(bool new_status) {
  bool old_status = logging;
  logging = new_status;
  return old_status;
}

