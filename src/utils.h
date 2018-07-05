//
// Created by tsbertalan on 7/3/18.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include <iostream>
#include <algorithm>

void printVec(std::vector<double> v, std::string name, std::string sep = " = ");


long argmin(std::vector<float> v);

float min(std::vector<float> v);

long argmax(std::vector<float> v);

float max(std::vector<float> v);



#endif //PATH_PLANNING_UTILS_H
