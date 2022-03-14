#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky> 
#include <iostream>
#include <algorithm>

#define square(a) pow(a, 2)

class unscented_kalman_filter{

    private:
        int n;
        int no_of_sigma_points; 
        float alpha;
        float beta;
        float kappa;
        float lambda;
        float n_plus_lambda;
        float gamma;

        Eigen::VectorXf initial_mean;
        Eigen::MatrixXf initial_covariance;
        Eigen::VectorXf instantaneous_mean;
        Eigen::MatrixXf instantaneous_covariance;
        Eigen::MatrixXf sigma_point_matrix;
        Eigen::MatrixXf transformed_sigma_point_matrix;

        float* weights_mean;
        float* weights_covariance;

    public:
        
        unscented_kalman_filter(Eigen::VectorXf initial_mean, Eigen::MatrixXf initial_covariance);
        ~unscented_kalman_filter();

        void display_free_parameters();
        void set_free_parameters(float alpha, float beta, float kappa);
        void compute_sigma_points(Eigen::VectorXf& mean, Eigen::MatrixXf& covariance);
       
        void compute_weights();
        void non_linear_transformation(Eigen::MatrixXf& sigma_points, Eigen::MatrixXf& transformed_sigma_points);
        void recover_gaussian(Eigen::VectorXf& mean, Eigen::MatrixXf& covariance, Eigen::MatrixXf& sigma_points);
        void display_guassian_parameters(Eigen::VectorXf& mean, Eigen::MatrixXf& covariance);
        void linear_transformation(Eigen::MatrixXf& sigma_points, Eigen::MatrixXf& transformed_sigma_points);
};

#endif
