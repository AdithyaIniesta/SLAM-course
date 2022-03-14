#include "ukf.h"

int main(){

    Eigen::VectorXf mean(2);
    mean << 1,2;
    Eigen::MatrixXf covariance(2,2);
    covariance << 3, 0.5,
                  0.5, 3;

    float alpha = 0.9;
    float beta = 2;
    float kappa = 1;

    unscented_kalman_filter* ukf = new unscented_kalman_filter(mean, covariance);
    ukf->set_free_parameters(alpha, beta, kappa);
    ukf->compute_sigma_points(mean, covariance);
    ukf->display_free_parameters();

    delete ukf;
    return 0;
}