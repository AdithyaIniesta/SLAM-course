#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

class kalmanFilter
{ 
    public:
        Eigen::MatrixXd x;
        Eigen::MatrixXd Z;
        Eigen::MatrixXd F;
        Eigen::MatrixXd H;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd I;
        Eigen::MatrixXd P;
        Eigen::MatrixXd R_laser;
        Eigen::MatrixXd R_radar;
        Eigen::MatrixXd Hj;
        Eigen::MatrixXd h;

        kalmanFilter();
        ~kalmanFilter(){};
        Eigen::MatrixXd printMean(){std::cout <<"x = " << x << "\n"; return x;};
        void printCov(){std::cout << "P = " << P << "\n"; };
        void calculateJacobian();
        void linearize();
        void update();
        void updateEKF();
        void predict();
};

#endif