#ifndef SRC_EXTENDED_KALMAN_FILTER
#define SRC_EXTENDED_KALMAN_FILTER

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <cmath>

#define square(a) pow(a, 2)

class ExtendedKalmanFilter {

public:
    const int dimX, dimZ, dimU;

    Eigen::MatrixXf x;
    Eigen::MatrixXf P;
    Eigen::MatrixXf F;
    Eigen::MatrixXf R;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf B;
    Eigen::MatrixXf y;
    Eigen::MatrixXf H;
    Eigen::MatrixXf K;
    Eigen::MatrixXf S;
    Eigen::MatrixXf SI;
    Eigen::MatrixXf xPrior;
    Eigen::MatrixXf PPrior;
    Eigen::MatrixXf xPosterior;
    Eigen::MatrixXf PPosterior;

    ExtendedKalmanFilter(int dimX, int dimZ, int dimU = 0);

    ~ExtendedKalmanFilter();

    void predictUpdate(Eigen::MatrixXf &z,
                       void (*jacobian)(Eigen::MatrixXf &x,
                                        Eigen::MatrixXf &H),
                       void (*Hx)(Eigen::MatrixXf &x,
                                  Eigen::MatrixXf &predictedZ));

    void predictX(Eigen::MatrixXf &u);

    void predict(Eigen::MatrixXf &u);

    float logLikelihood();

private:
    float _logLikelihood;
    float _likelihood;
    float _mahalanobis;
    Eigen::MatrixXf _I;

};


#endif