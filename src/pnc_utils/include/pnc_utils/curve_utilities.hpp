#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <vector>


// returns a hermite interpolation (cubic) of the boundary conditions for a given s \in [0,1].

class HermiteCurve{
public:
	HermiteCurve();
	HermiteCurve(const double & start_pos, const double & start_vel, 
				 const double & end_pos, const double & end_vel, const double & duration);
	~HermiteCurve();
	double evaluate(const double & t_in);
	double evaluateFirstDerivative(const double & t_in);
	double evaluateSecondDerivative(const double & t_in);

private:
	double p1;
	double v1;
	double p2;
	double v2;

	double t_dur;

	double s_;

	// by default clamps within 0 and 1.
	double clamp(const double & t_in, double lo = 0.0, double hi = 1.0);

};


// vector version of hermite curve interpolation

class HermiteCurveVec{
public:
	HermiteCurveVec();
	HermiteCurveVec(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, 
				   const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const double & duration);
	~HermiteCurveVec();
	
	void initialize(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, 
					const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const double & duration);
	Eigen::VectorXd evaluate(const double & t_in);
	Eigen::VectorXd evaluateFirstDerivative(const double & t_in);
	Eigen::VectorXd evaluateSecondDerivative(const double & t_in);

private:
	Eigen::VectorXd p1;
	Eigen::VectorXd v1;
	Eigen::VectorXd p2;
	Eigen::VectorXd v2;

	double t_dur;

	std::vector<HermiteCurve> curves;
 	Eigen::VectorXd output;
};


// Hermite Quaternion curve for global frame quaternion trajectory given boundary conditions
// also computes global frame angular velocity and angular acceleration for s \in [0,1]

class HermiteQuaternionCurve{
public:
	HermiteQuaternionCurve();
	HermiteQuaternionCurve(const Eigen::Quaterniond & quat_start, 
						   const Eigen::Vector3d & angular_velocity_start,
						   const Eigen::Quaterniond & quat_end, 
						   const Eigen::Vector3d & angular_velocity_end,
						   double duration);
	~HermiteQuaternionCurve();

	void initialize(const Eigen::Quaterniond& quat_start,
					const Eigen::Vector3d& angular_velocity_start,
					const Eigen::Quaterniond& quat_end,
					const Eigen::Vector3d& angular_velocity_end,
					double duration);

	// All values are expressed in "world frame"
	void evaluate(const double & t_in, Eigen::Quaterniond & quat_out);
	void getAngularVelocity(const double & t_in, Eigen::Vector3d & ang_vel_out);
	void getAngularAcceleration(const double & t_in, Eigen::Vector3d & ang_acc_out);

private:
	double t_dur; // time duration

	Eigen::Quaterniond qa; // Starting quaternion
	Eigen::Vector3d omega_a; // Starting Angular Velocity
	Eigen::Quaterniond qb; // Ending quaternion
	Eigen::Vector3d omega_b; // Ending Angular velocity

	HermiteCurveVec theta_ab; // so3 
};

