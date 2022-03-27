#include "ekf.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(int dimX, int dimZ, int dimU) :
        dimX(dimX),
        dimZ(dimZ),
        dimU(dimU) {

    x = Eigen::MatrixXf::Identity(dimX, 1);
    P = Eigen::MatrixXf::Identity(dimX, dimX);
    F = Eigen::MatrixXf::Identity(dimX, dimX);
    R = Eigen::MatrixXf::Identity(dimZ, dimZ);
    Q = Eigen::MatrixXf::Identity(dimX, dimX);
    B = Eigen::MatrixXf(dimX, dimU);
    y = Eigen::MatrixXf::Zero(dimZ, 1);
    H = Eigen::MatrixXf::Identity(1, dimX);
    // line 146, 147 not implemented
    K = Eigen::MatrixXf::Zero(dimX, 1);
    S = Eigen::MatrixXf::Zero(dimZ, dimZ);
    SI = Eigen::MatrixXf::Zero(dimZ, dimZ);

    _I = Eigen::MatrixXf::Identity(dimX, dimX);

    _likelihood = std::numeric_limits<float>::min();
    _logLikelihood = log(_likelihood);
//    _mahalanobis line 162

    xPrior = x;
    PPrior = P;

    xPosterior = x;
    PPosterior = P;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {

    std::cout << "EKF class instance deleted \n";
}

void ExtendedKalmanFilter::predictUpdate(Eigen::MatrixXf &z,
                                         void (*jacobian)(Eigen::MatrixXf &x,
                                                          Eigen::MatrixXf &H),
                                         void (*Hx)(Eigen::MatrixXf &x,
                                                    Eigen::MatrixXf &predictedZ)) {

    int u = 0;
    jacobian(x, H);

    x = F * x + B * u;
    P = F * P * F.transpose() + Q;

    xPrior = x;
    PPrior = PPrior;

    Eigen::MatrixXf PHT = P * H.transpose();
    S = H * PHT + R;

    SI = S.inverse();
    K = PHT * SI;

    Eigen::MatrixXf zPredicted = Eigen::MatrixXf::Zero(z.rows(), z.cols());
    Hx(x, zPredicted);
    y = z - zPredicted;


    x = x + K * y;

    Eigen::MatrixXf I_KH = _I - K * H;

    P = (I_KH * P) * I_KH.transpose() + (K * R) * K.transpose();

//    z = z;
    xPosterior = x;
    PPosterior = P;
}

void update(Eigen::MatrixXf &z,
            void (*jacobian)(Eigen::MatrixXf &x,
                             Eigen::MatrixXf &H),
            void (*Hx)(Eigen::MatrixXf &x,
                       Eigen::MatrixXf &predictedZ)){

    jacobian(x, H);

    Eigen::MatrixXf PHT = P * H.transpose();
    S = H * PHT + R;

    SI = S.inverse();
    K = PHT * SI;


}

void ExtendedKalmanFilter::predictX(Eigen::MatrixXf &u) {

    x = F * x + B * u;
}

void ExtendedKalmanFilter::predict(Eigen::MatrixXf &u) {

    this->predictX(u);
    P = F * P * F.transpose() + Q;
    xPrior = x;
    PPrior = P;
}

