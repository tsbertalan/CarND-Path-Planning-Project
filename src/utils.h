//
// Created by tsbertalan on 7/3/18.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include <iostream>

void printVec(std::vector<double> v, std::string name) {
    std::cout << name << " = [";
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


#endif //PATH_PLANNING_UTILS_H
