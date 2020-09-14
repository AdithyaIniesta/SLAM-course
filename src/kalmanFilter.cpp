#include "kalmanFilter.h"
#include <cmath>

kalmanFilter::kalmanFilter(){
}

void kalmanFilter::linearize(){
    
    double px = x(0,0);
    double py = x(1,0);
    double vx = x(2,0);
    double vy = x(3,0);

    double a1 = sqrt(px*px + py*py);
    double a2 = atan2(py , px);
    double a3 = (px*vx + py * vy);

    h = Eigen::MatrixXd(3,1);
    h << a1,
        a2,
        a3 / a1;
}

void kalmanFilter::calculateJacobian(){

    double px = x(0,0);
    double py = x(1,0);
    double vx = x(2,0);
    double vy = x(3,0);

    double den = sqrt(px*px + py*py);
    double den_2 = den*den;
    double den_3 = den_2 * den;

    Hj = Eigen::MatrixXd(3,4);
    Hj << px / den, py/den, 0, 0,
          (-1*py)/(den_2), (px) / (den_2), 0, 0,
          py*(vx*py - vy*px) / den_3, px*(vy*px - vx*py) / den_3, px / den, py / den; 

}

void kalmanFilter::predict(){

    x = F * x;
    P = F * P * F.transpose() + Q;
}

void kalmanFilter::update(){ 
    
    Eigen::MatrixXd Y = Z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R_laser;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * Y;
    I = Eigen::MatrixXd::Identity(K.rows(), H.cols());
    P = (I - K * H) * P;  
}

void kalmanFilter::updateEKF(){

    linearize();
    Eigen::MatrixXd Y = Z - h;
    calculateJacobian();
    Eigen::MatrixXd S = Hj * P * Hj.transpose() + R_radar;
    Eigen::MatrixXd K = P * Hj.transpose() * S.inverse();
    x = x + K * Y;
    I = Eigen::MatrixXd::Identity(K.rows(), Hj.cols());
    P = (I - K * Hj) * P;
}



