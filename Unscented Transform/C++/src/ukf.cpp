#include "ukf.h"

unscented_kalman_filter::unscented_kalman_filter(
	Eigen::VectorXf mean, Eigen::MatrixXf covariance)
{

	n = mean.rows();
	initial_mean = Eigen::VectorXf(n);
	initial_mean = mean;
	initial_covariance = Eigen::MatrixXf(n, n);
	initial_covariance = covariance;
	no_of_sigma_points = (2 * n + 1);

	instantaneous_mean = Eigen::VectorXf(n);
	instantaneous_covariance = Eigen::MatrixXf(n, n);
	sigma_point_matrix = Eigen::MatrixXf(n, no_of_sigma_points);
	transformed_sigma_point_matrix = Eigen::MatrixXf(n, no_of_sigma_points);

	weights_mean = new float[no_of_sigma_points];
	weights_covariance = new float[no_of_sigma_points];
}

unscented_kalman_filter::~unscented_kalman_filter()
{
	std::cout << "An instance of the class is deleted! \n";
}

void unscented_kalman_filter::display_free_parameters()
{
	std::cout << "alpha: " << alpha << " beta: " << beta << " kappa: " << kappa
				 << " lambda: " << lambda << "\n";

	std::cout << "Weights for computing mean: \n";
	for (int col_idx = 0; col_idx < no_of_sigma_points; col_idx++) {
		std::cout << weights_mean[col_idx] << ", ";
	}

	std::cout << "Weights for recovering covariance: \n";
	for (int col_idx = 0; col_idx < no_of_sigma_points; col_idx++) {
		std::cout << weights_covariance[col_idx] << ", ";
	}
}

void unscented_kalman_filter::set_free_parameters(float a, float b, float k)
{
	alpha = a;
	beta = b;
	kappa = k;
	lambda = square(a) * (n + k) - n;
	n_plus_lambda = n + lambda;
	gamma = sqrt(n_plus_lambda);
}

void unscented_kalman_filter::compute_sigma_points(
	Eigen::VectorXf &mean, Eigen::MatrixXf &covariance)
{

	sigma_point_matrix.col(0) << mean;
	Eigen::MatrixXf square_root_covariance(covariance.llt().matrixL());

	for (int col_idx = 0; col_idx < n; col_idx++) {
		sigma_point_matrix.col(col_idx + 1)
			<< mean + gamma * square_root_covariance.col(col_idx);
		sigma_point_matrix.col(n + col_idx + 1)
			<< mean - gamma * square_root_covariance.col(col_idx);
	}

	this->display_guassian_parameters(mean, covariance);
	std::cout << "Sigma points: \n" << sigma_point_matrix << "\n";
	this->compute_weights();
	this->linear_transformation(
		sigma_point_matrix, transformed_sigma_point_matrix);
	this->recover_gaussian(mean, covariance, transformed_sigma_point_matrix);
	this->display_guassian_parameters(mean, covariance);
}

void unscented_kalman_filter::compute_weights()
{
	weights_mean[0] = float(lambda) / float(n_plus_lambda);
	weights_covariance[0] = weights_mean[0] + (1 - square(alpha) + beta);

	float reciprocal_two_times_n_plus_lambda = 1 / (2 * n_plus_lambda);
	std::fill(weights_mean + 1, weights_mean + no_of_sigma_points,
		reciprocal_two_times_n_plus_lambda);
	std::fill(weights_covariance + 1, weights_covariance + no_of_sigma_points,
		reciprocal_two_times_n_plus_lambda);
}

void unscented_kalman_filter::non_linear_transformation(
	Eigen::MatrixXf &sigma_points, Eigen::MatrixXf &transformed_sigma_points)
{
	for (int col_idx = 0; col_idx < no_of_sigma_points; col_idx++) {
		float x = sigma_points.col(col_idx)[0];
		float y = sigma_points.col(col_idx)[1];

		float r = sqrt(square(x) + square(y));
		float theta = atan2(y, x);
		transformed_sigma_points.col(col_idx) << r, theta;
	}

	std::cout << "Non-linear transformation of sigma points: \n"
				 << transformed_sigma_points << "\n";
}

void unscented_kalman_filter::recover_gaussian(Eigen::VectorXf &mean,
	Eigen::MatrixXf &covariance, Eigen::MatrixXf &sigma_points)
{

	Eigen::VectorXf sum_mean = Eigen::VectorXf::Zero(n);
	Eigen::MatrixXf sum_cov = Eigen::MatrixXf::Zero(n, n);

	for (int col_idx = 0; col_idx < no_of_sigma_points; col_idx++) {
		sum_mean << sum_mean + weights_mean[col_idx] * sigma_points.col(col_idx);
	}

	Eigen::VectorXf sigma_point_minus_mean(n);
	for (int col_idx = 0; col_idx < no_of_sigma_points; col_idx++) {
		sigma_point_minus_mean = sigma_points.col(col_idx) - sum_mean;
		sum_cov = sum_cov
			+ (weights_covariance[col_idx] * sigma_point_minus_mean
				* sigma_point_minus_mean.transpose());
	}

	mean = sum_mean;
	covariance = sum_cov;
}

void unscented_kalman_filter::display_guassian_parameters(
	Eigen::VectorXf &mean, Eigen::MatrixXf &covariance)
{
	std::cout << "Mean of the the Gaussian distribution: \n" << mean << "\n";
	std::cout << "Covariance of the Gaussian distribution: \n"
				 << covariance << "\n";
}

void unscented_kalman_filter::linear_transformation(
	Eigen::MatrixXf &sigma_points, Eigen::MatrixXf &transformed_sigma_points)
{
	for (int col_idx = 0; col_idx < no_of_sigma_points; col_idx++) {
		float x = sigma_points.col(col_idx)[0];
		float y = sigma_points.col(col_idx)[1];

		float x_ = 1.0003 * x;
		float y_ = 1.0003 * y;
		transformed_sigma_points.col(col_idx) << x_, y_;
	}

	std::cout << "Linear transformation of sigma points: \n"
				 << transformed_sigma_points << "\n";
}