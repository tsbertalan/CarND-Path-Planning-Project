//
// Created by tsbertalan on 7/8/18.
//

#ifndef PATH_PLANNING_LOGGER_H
#define PATH_PLANNING_LOGGER_H

#include <string>
#include <fstream>
#include <vector>

#include "trajectory.h"
#include "coordinates.h"
#include "neighbor.h"

class PyLogger {
 private:
  std::ofstream pyfile;
  CoordinateTransformer &transform;
  bool logging;

 public:
  explicit PyLogger(CoordinateTransformer &transform, std::string filePath = "data.py");

  // Output is basically JSON in python; "items" are dictionaries;
  void begin_item(long tag);
  void begin_item(std::string tag);
  void end_item();

  // Define lots of calling semantics for logging different pieces of data.
  void operator()(std::string name, std::vector<double> vec);
  void operator()(std::string name, std::vector<std::vector<double>> arr);
  void operator()(std::string name, std::vector<std::vector<std::vector<double>>> arr);
  void operator()(std::string name, std::string data);
  void operator()(std::string name, long int data);
  void operator()(std::string name, int data);
  void operator()(std::string name, double data);
  void operator()(std::string name, std::string desc, Trajectory tj);
  void operator()(std::string name, std::vector<Neighbor> neighbors, double tproj = 0);

  // Allow for turning logging on and off at runtime.
  bool set_status(bool new_status);
};

#endif //PATH_PLANNING_LOGGER_H
