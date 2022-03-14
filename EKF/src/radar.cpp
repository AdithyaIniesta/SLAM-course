//
// Created by adithyainiesta on 03.03.22.
//
#include "radar.h"

RadarSimulation::RadarSimulation(float pos, float vel, float alt, float dt)
        :
        dt(dt), pos(pos), vel(vel), alt(alt) {
}

RadarSimulation::~RadarSimulation() {

    std::cout << "Radar simulation class instance deleted \n";
}

void RadarSimulation::getRange() {

//    vel = vel + generateRandomNumbers(10, 100);
    alt = alt;
    pos = pos + vel * dt;

    std::cout << vel + generateRandomNumbers(10, 100) << "\n";
}

float
RadarSimulation::generateRandomNumbers(int min,
                                       int max) {
    float rd = (float) (rand() % max);
    rd = (rd < min) ? min : rd;
    return rd;
}
