//
// Created by tsbertalan on 7/3/18.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include <iostream>
#include <algorithm>

// For debugging.
void print_vec(std::vector<double> v, std::string name, std::string sep = " = ");

// Find the smallest element in a vector, or its index.
long argmin(std::vector<double> v);
double min(std::vector<double> v);

// Find the largest element in a vector, or its index.
long argmax(std::vector<double> v);
long argmax(std::vector<int> v);
double max(std::vector<double> v);

// Find the sequence of indices that sorts a vector.
std::vector<unsigned long> argsort(std::vector<double> v);
std::vector<unsigned long> argsort(std::vector<int> v);

// Tanh-like sigmoid function.
double expit(double x, double x_critical, double scale_factor = 1);

// Evaluate a line defined in two-point form.
double line(double x, double x1, double y1, double x2, double y2);

#endif //PATH_PLANNING_UTILS_H
