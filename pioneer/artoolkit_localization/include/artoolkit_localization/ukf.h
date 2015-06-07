/**
 * @file     ukf.h
 * @author   George Andrew Brindeiro and Mateus Mendelson
 * @date     06/10/2012
 *
 * @brief Unscented Kalman Filter class for the artoolkit_localization package
 *
 * This class implements UKF localization using ARToolkit markers, based on a
 * map of ARMarkers. The user must provide the node update rate, the filename
 * for the map and tf between the /camera and /odom frames.
 *
 * Contact: georgebrindeiro@lara.unb.br
 *
 * Revisions:
 * [06/10/2012] Created
 */

#ifndef UKF_H
#define UKF_H

 // Operating mode (choose ONLY ONE as true)
#define POSE_WITH_COVARIANCE_STAMPED true // Note: This mode hasn't been completely implemented
#define POSE_STAMPED false

#include <artoolkit_localization/ar_map.h>

// Standard C libraries
#include <cstdio>

// Eigen
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <vector>

#include <angles/angles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

class UKF
{
    public:
        UKF(double dt = 0.1, std::string map_path = "cfg/lara.map")
			: map_(map_path) // <---------------------------------------------------------
        {
            dt_ = dt;

            Init();
        }

        ~UKF() {};

        // Public interface
        void localize(std::vector<double> controls, std::map< int, vector<double> > measurements);

        // Utility functions
		inline static double range(double dx, double dy)
		{
			return sqrt(dx*dx+dy*dy);
		}

		inline static double bearing(double dx, double dy, double theta)
		{
			return angles::normalize_angle(atan2(dy,dx)-theta);
		}

		#if POSE_WITH_COVARIANCE_STAMPED
			geometry_msgs::PoseWithCovarianceStamped poseMsg();
		#elif POSE_STAMPED
			geometry_msgs::PoseStamped poseMsg();
		#endif

        /* UKF Variables */
        Eigen::Vector3d state_;
        Eigen::Matrix3d cov_state_;

        Eigen::VectorXd aug_state_;
        Eigen::MatrixXd cov_aug_state_;

        Eigen::MatrixXd sigma_state_;
        Eigen::MatrixXd sigma_controls_;
        Eigen::MatrixXd sigma_measurements_;
        Eigen::MatrixXd sigma_aug_state_;

        Eigen::MatrixXd sigma_measurement_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        double dt_;			// UKF loop rate in seconds
		ARMap map_;			// UKF map

		void Init();

		void augmentState(std::vector<double> controls, std::map< int, vector<double> > measurements);
		void generateSigmaPoints();
		void prediction(std::vector<double> controls);
		void update(std::map< int, vector<double> > measurements);
		void motionModel(Eigen::MatrixXd sigma_controls);
		void measurementModel(int measurement_id, Eigen::MatrixXd sigma_state, Eigen::MatrixXd sigma_measurements);
		Eigen::VectorXd weightedMean(Eigen::MatrixXd X);
		Eigen::MatrixXd weightedCovariance(Eigen::MatrixXd X, Eigen::VectorXd x);
		Eigen::MatrixXd weightedCrossCovariance(Eigen::MatrixXd X, Eigen::VectorXd x, Eigen::MatrixXd Z, Eigen::VectorXd z);

		/* UKF Parameters */

		// Control Noise
		double alpha_vv_;
		double alpha_vw_;
		double alpha_wv_;
		double alpha_ww_;
		double std_v_;
		double std_w_;

		// Measurement Noise
		double std_rho_;
		double std_phi_;

		Eigen::Matrix2d cov_measurement_;

		// Weights
	    /* UKF parameters */
		int n_;
	    int s_;
		double kappa_;
		double wc_;
		double we_;

		/* Matrix sizes */
		int m_;
		int a_;

        DISALLOW_COPY_AND_ASSIGN(UKF);
};

#endif //UKF_H
