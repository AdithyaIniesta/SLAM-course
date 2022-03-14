//
// Created by adithyainiesta on 03.03.22.
//

#ifndef SRC_RADAR_H
#define SRC_RADAR_H

#include <random>
#include <cstdlib>
#include <limits>
#include <iostream>

#define maxInt std::numeric_limits<int>::max()
#define minInt std::numeric_limits<int>::min()


class RadarSimulation {
public:
    RadarSimulation(float pos, float vel, float alt, float dt = 1);

    ~RadarSimulation();

    void getRange();

    float generateRandomNumbers(int min = minInt,
                                int max = maxInt);

private:
    float pos, vel, alt, dt;
};

#endif //SRC_RADAR_H
