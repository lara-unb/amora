/**
 * @file    ukf.cpp
 * @author  George Andrew Brindeiro and Mateus Mendelson
 * @date    06/10/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <artoolkit_localization/ukf.h>

#include <ros/ros.h>

// Standard C libraries
#include <cstdio>

using namespace Eigen;

void UKF::Init()
{
	// Initialize all parameters needed for UKF
	ROS_DEBUG("UKF Init started");

	/* general */
	state_ << 0, 0, 0;
	cov_state_ << 0.25, 0.0, 0.0,
				 0.0, 0.25, 0.0,
				 0.0, 0.0, 0.01;

	/* augment_state */

	// Control noise
	alpha_vv_ = 0.1;
	alpha_vw_ = 0.1;
	alpha_wv_ = 0.5;
	alpha_ww_ = 0.5;

	std_v_ = 1E-7;
	std_w_ = 1E-7;

	// Measurement noise
	std_rho_ = 0.5;
	std_phi_ = 5.0;

	cov_measurement_(0,0) = std_rho_*std_rho_;
	cov_measurement_(0,1) = 0;
	cov_measurement_(1,0) = 0;
	cov_measurement_(1,1) = std_phi_*std_phi_;

	ROS_DEBUG("UKF Init finished");
}

void UKF::localize(std::vector<double> controls, std::map< int, vector<double> > measurements)
{
	//ROS_INFO("Running localization loop...");

	//ROS_INFO("(1) augment_state");
	augmentState(controls, measurements);

	//ROS_INFO("(2) generate_sigma_points");
	generateSigmaPoints();

	if(controls.size() > 0)
	{
		//ROS_INFO("(3) prediction");
		prediction(controls);
	}

	if(measurements.size() > 0)
	{
		update(measurements);
		//ROS_INFO("(4) update");
	}
       
    //cout << cov_state << endl;

	//ROS_INFO("(5) done!");
}

void UKF::augmentState(std::vector<double> controls, std::map< int, vector<double> > measurements)
{
	// Auxiliary variables
	double v_;
	double w_;

	int c = controls.size();

	if(c > 0)
	{
		v_ = controls[0];
		w_ = controls[1];
	}

	m_ = measurements.size();

	a_ = 3 + 2 + 2*m_;

    /* UKF parameters */
	n_     = 3;
	kappa_ = 1.0;
    s_ = 2*a_ + 1;
	wc_ = kappa_/(a_ + kappa_); // p.65: peso central varia para covariância e média
	we_ = 1/(2*(a_ + kappa_));

	// Control covariance matrix
	Matrix2d cov_controls = Matrix2d::Zero();

	cov_controls(0,0) = alpha_vv_*v_*v_ + alpha_vw_*w_*w_ + std_v_;
	cov_controls(1,1) = alpha_wv_*v_*v_ + alpha_ww_*w_*w_ + std_w_;

	// Measurement covariance matrix
	MatrixXd cov_measurements = MatrixXd::Zero(2*m_,2*m_);

	for(int i = 0; i < m_; i++)
	{
		cov_measurements.block<2,2>(2*i,2*i) = cov_measurement_;
	}

	// Augmented state vector
	aug_state_ = VectorXd::Zero(a_);

	aug_state_.block<3,1>(0,0) = state_;

	// Augmented covariance matrix
	cov_aug_state_ = MatrixXd::Zero(a_,a_);

	cov_aug_state_.block<3,3>(0,0) = cov_state_;
	cov_aug_state_.block<2,2>(3,3) = cov_controls;
	cov_aug_state_.block(5,5,2*m_,2*m_) = cov_measurements;
}

void UKF::generateSigmaPoints()
{
	// Initialize all sigma points with appropriate dimensions
    sigma_state_ = MatrixXd::Zero(3,s_);
    sigma_controls_ = MatrixXd::Zero(2,s_);
    sigma_measurements_ = MatrixXd::Zero(2*m_,s_);
    sigma_aug_state_ = MatrixXd::Zero(a_,s_);

	// Generate augmented state vector sigma points
	MatrixXd block_sigma_aug_state = aug_state_.replicate(1,a_);
	MatrixXd sqrt_cov_aug_state = cov_aug_state_.llt().matrixL().transpose();

	sigma_aug_state_.col(0) = aug_state_;
	sigma_aug_state_.block(0,1,a_,a_) = block_sigma_aug_state + sqrt(a_ + kappa_)*sqrt_cov_aug_state;
	sigma_aug_state_.block(0,a_+1,a_,a_) = block_sigma_aug_state - sqrt(a_ + kappa_)*sqrt_cov_aug_state;

	// Copy over to subvector sigma points
	sigma_state_ = sigma_aug_state_.block(0,0,3,s_);
	sigma_controls_ = sigma_aug_state_.block(3,0,2,s_);
	sigma_measurements_ = sigma_aug_state_.block(5,0,2*m_,s_);
}

void UKF::prediction(std::vector<double> controls)
{
	// Auxiliary variables
	Vector2d control(controls[0], controls[1]);
	MatrixXd block_control = control.replicate(1,s_);

	/*sigma_state = */motionModel(block_control + sigma_controls_);

	state_ = weightedMean(sigma_state_);
	cov_state_ = weightedCovariance(sigma_state_, state_);
}

void UKF::update(std::map< int, vector<double> > measurements)
{
	int i = 0;

	/* Update runs once for each measurement */
	for(map< int, vector <double> >::iterator im = measurements.begin(); im != measurements.end(); im++, i++)
	{
		// Auxiliary variables
		int measurement_id = im->first;
		Vector2d measurement(im->second[0], im->second[1]);

		/*sigma_measurement = */measurementModel(measurement_id, sigma_state_, sigma_measurements_.block(2*i,0,2,s_));

		VectorXd measurement_estimate = weightedMean(sigma_measurement_);
		MatrixXd cov_measurement_estimate = weightedCovariance(sigma_measurement_, measurement_estimate);
		MatrixXd cross_cov = weightedCrossCovariance(sigma_state_, state_, sigma_measurement_, measurement_estimate);

		MatrixXd kalman_gain = cross_cov*cov_measurement_estimate.inverse();

		state_ = state_ + kalman_gain*(measurement-measurement_estimate);
		cov_state_ = cov_state_ - kalman_gain*cov_measurement_estimate*kalman_gain.transpose();
	}
}

void UKF::motionModel(MatrixXd sigma_controls)
{
	// Sigma point displacement
	MatrixXd delta_sigma_state(3,s_);

	// Apply motion model to each of the sigma points
	for(int i = 0; i < s_; i++)
	{
		// Auxiliary variables
		double v = sigma_controls_(0,i);
		double w = sigma_controls_(1,i);
		double theta = sigma_state_(2,i);
		double v_w = v/w;

		// Deal with singularity in v_w using L'Hopital's Rule
		if(w != 0)
		{
			delta_sigma_state(0,i) = -v_w*sin(theta) + v_w*sin(theta + w*dt_);
			delta_sigma_state(1,i) =  v_w*cos(theta) - v_w*cos(theta + w*dt_);
			delta_sigma_state(2,i) = w*dt_;
		}
		else
		{
			delta_sigma_state(0,i) = v*cos(theta)*dt_;
			delta_sigma_state(1,i) = v*sin(theta)*dt_;
			delta_sigma_state(2,i) = 0;
		}
	}

	sigma_state_ = sigma_state_ + delta_sigma_state;

}

void UKF::measurementModel(int measurement_id, MatrixXd sigma_state, MatrixXd sigma_measurements)
{
	// Sigma point displacement
	MatrixXd delta_sigma_measurement(2,s_);

	// Apply measurement model to each of the sigma points
	for(int i = 0; i < s_; i++)
	{
		// Auxiliary variables
		vector<double> m = map_.get(measurement_id);
		double dx = sigma_state(0,i)-m[0];
		double dy = sigma_state(1,i)-m[1];
		double theta = sigma_state(2,i);

		delta_sigma_measurement(0,i) = range(dx,dy);
		delta_sigma_measurement(1,i) = bearing(dx,dy,theta);
	}

	sigma_measurement_ = delta_sigma_measurement + sigma_measurements;
}

VectorXd UKF::weightedMean(MatrixXd X)
{
	VectorXd x;

	x = wc_*X.col(0);

	for (int i = 1; i < s_; i++)
	{
		x = x + we_ * X.col(i);
	}

	return x;
}

MatrixXd UKF::weightedCovariance(MatrixXd X, VectorXd x)
{
	MatrixXd P;

	P = wc_*((X.col(0) - x)*((X.col(0) - x).transpose()));

	for (int i = 1; i < s_; i++)
	{
		P = P + we_*((X.col(i) - x)*((X.col(i) - x).transpose()));
	}

	return P;
}

MatrixXd UKF::weightedCrossCovariance(MatrixXd X, VectorXd x, MatrixXd Z, VectorXd z)
{
	MatrixXd cross_cov;

	cross_cov = wc_*((X.col(0) - x)*((Z.col(0) - z).transpose()));

	for (int i = 1; i < s_; i++)
	{
		cross_cov = cross_cov + we_*((X.col(i) - x)*((Z.col(i) - z).transpose()));
	}

	return cross_cov;
}

#if POSE_WITH_COVARIANCE_STAMPED
geometry_msgs::PoseWithCovarianceStamped UKF::poseMsg()
{
	geometry_msgs::PoseWithCovarianceStamped pose_msg;

	pose_msg.header.stamp = ros::Time::now();
	pose_msg.header.frame_id = "pose_no_cov";

	pose_msg.pose.pose.position.x = state_(0);
	pose_msg.pose.pose.position.y = state_(1);
	pose_msg.pose.pose.position.z = 0;

	pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state_(2));

	// TO DO: Add covariance

	return pose_msg;
}
#elif POSE_STAMPED
geometry_msgs::PoseStamped UKF::poseMsg()
{
	geometry_msgs::PoseStamped pose_msg;

	pose_msg.header.stamp = ros::Time::now();
	pose_msg.header.frame_id = "pose_no_cov";

	pose_msg.pose.position.x = state_(0);
	pose_msg.pose.position.y = state_(1);
	pose_msg.pose.position.z = 0;

	pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(state_(2));

	// TO DO: Add covariance

	return pose_msg;
}
#endif