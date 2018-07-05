//
// Created by tsbertalan on 7/5/18.
//
#include "utils.h"

long argmin(std::vector<float> v) {
    return min_element(v.begin(), v.end()) - v.begin();
}

float min(std::vector<float> v) {
    return v[argmin(v)];
}

long argmax(std::vector<float> v) {
    return max_element(v.begin(), v.end()) - v.begin();
}

float max(std::vector<float> v) {
    return v[argmax(v)];
}

void printVec(std::vector<double> v, std::string name, std::string sep) {
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
