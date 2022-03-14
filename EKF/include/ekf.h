#ifndef EXTENDED_KALMAN_FILTER
#define EXTENDED_KALMAN_FILTER

#include <iostream>
#include <eigen3/Eigen/Dense>

class ExtendedKalmanFilter {

public:
    int dimX, dimZ, dimU;

    Eigen::MatrixXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd F;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd B;
    Eigen::MatrixXd y;
    Eigen::MatrixXd K;
    Eigen::MatrixXd S;
    Eigen::MatrixXd SI;
    Eigen::MatrixXd xPrior;
    Eigen::MatrixXd PPrior;


    ExtendedKalmanFilter(int dimX, int dimZ, int dimU = 0);

    ~ExtendedKalmanFilter();

    void predictX(int u = 0);

    void predict(int u = 0);

    Eigen::MatrixXd logLikelihood();

private:
    Eigen::MatrixXd _I;
    Eigen::MatrixXd _logLikelihood;
    Eigen::MatrixXd _likelihood;
    Eigen::MatrixXd _mahalanobis;
};


#endif