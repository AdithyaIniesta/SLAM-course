//
// Created by adithyainiesta on 02.03.22.
//

#include "ekf.h"
#include "radar.h"

void calculateJacobian(Eigen::MatrixXf &x, Eigen::MatrixXf &H) {

    float horizDist = x(0);
    float altitude = x(1);
    float denominator = sqrt(pow(horizDist, 2) + pow(altitude, 2));
    H << horizDist / denominator, 0, altitude / denominator;
}

void Hx(Eigen::MatrixXf &x, Eigen::MatrixXf &predictedZ) {
    predictedZ << sqrt(square(x(0)) + square(x(2)));
}


int main() {

    ExtendedKalmanFilter *ekf = new ExtendedKalmanFilter(3, 1, 1);
    Eigen::MatrixXf z = Eigen::MatrixXf(1, 1);
    z << 1;
    ekf->predictUpdate(z, &calculateJacobian, &Hx);
    delete ekf;

    RadarSimulation *rds = new RadarSimulation(1,1,1);
    rds->getRange();
    rds->getRange();
    delete rds;

    return EXIT_SUCCESS;
}