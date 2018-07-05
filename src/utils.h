//
// Created by tsbertalan on 7/3/18.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include <iostream>
#include <algorithm>

void print_vec(std::vector<double> v, std::string name, std::string sep = " = ");


long argmin(std::vector<double> v);

double min(std::vector<double> v);

long argmax(std::vector<double> v);

double max(std::vector<double> v);



#endif //PATH_PLANNING_UTILS_H
