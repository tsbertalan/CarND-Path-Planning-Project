//
// Created by tsbertalan on 7/5/18.
//
#include "utils.h"
using namespace std;

long argmin(std::vector<double> v) {
  return min_element(v.begin(), v.end()) - v.begin();
}

double min(std::vector<double> v) {
  return v[argmin(v)];
}

long argmax(std::vector<double> v) {
  return max_element(v.begin(), v.end()) - v.begin();
}

long argmax(std::vector<int> v) {
  return max_element(v.begin(), v.end()) - v.begin();
}

double max(std::vector<double> v) {
  return v[argmax(v)];
}

void print_vec(std::vector<double> v, std::string name, std::string sep) {
  std::cout << name << sep << "[";
  if (v.size() > 0) {
    if (v.size() > 1) {
      for (int i = 0; i < v.size() - 1; i++) {
        std::cout << v[i] << ", ";
      }
    }
    std::cout << v[v.size() - 1];
  }
  std::cout << "]" << std::endl;
}

std::vector<unsigned long> argsort(std::vector<int> v) {
  std::vector<unsigned long> indices(v.size());
  unsigned long i = 0;
  std::iota(indices.begin(), indices.end(), i++);
  sort(indices.begin(), indices.end(), [&](unsigned long i, unsigned long j) { return v[i] < v[j]; });
  return indices;
}

double expit(double x, double x_critical, double scale_factor) {
  return 1./(1. + (double) exp(-(x - x_critical)*scale_factor));
}

double line(double x, double x1, double y1, double x2, double y2) {
  return (y2 - y1)/(x2 - x1)*(x - x1) + y1;
}

