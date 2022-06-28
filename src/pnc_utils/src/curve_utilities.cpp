#include <pnc_utils/curve_utilities.hpp>

// Cubic Hermite Spline: 
// From https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Unit_interval_(0,_1)
// p(s) = (2s^3 - 3s^2 + 1)*p1 + (-2*s^3 + 3*s^2)*p2 + (s^3 - 2s^2 + s)*v1 + (s^3 - s^2)*v2
// where 0 <= s <= 1. 

HermiteCurve::HermiteCurve(){
  p1 = 0; v1 = 0;
  p2 = 0; v2 = 0;
  t_dur = 0.5;
  // std::cout << "[Hermite Curve] constructed" << std::endl;
}

HermiteCurve::HermiteCurve(const double & start_pos, const double & start_vel, 
                           const double & end_pos, const double & end_vel, const double & duration): 
                           p1(start_pos), v1(start_vel), p2(end_pos), v2(end_vel), t_dur(duration){
  if(t_dur < 1e-3){
    std::cout<<"given t_dur lower than minimum -> set to min: 0.001" << std::endl;
    t_dur = 1e-3;
  }
  // std::cout << "[Hermite Curve] constructed with values" << std::endl;
}

// Destructor
HermiteCurve::~HermiteCurve(){}

double HermiteCurve::evaluate(const double & t_in){
  double s = this->clamp(t_in/t_dur);
  return p1*(2*std::pow(s,3) - 3*std::pow(s,2) + 1) + 
         p2*(-2*std::pow(s,3) + 3*std::pow(s,2))    + 
         v1*t_dur*(std::pow(s,3) - 2*std::pow(s,2) + s)  + 
         v2*t_dur*(std::pow(s,3) - std::pow(s,2)); 
}

double HermiteCurve::evaluateFirstDerivative(const double & t_in){
  double s = this->clamp(t_in/t_dur);
  return (p1*(6*std::pow(s, 2) - 6*s)     +
         p2*(-6*std::pow(s, 2) + 6*s)     +
         v1*t_dur*(3*std::pow(s, 2) - 4*s + 1) +
         v2*t_dur*(3*std::pow(s, 2) - 2*s))/t_dur;
}

double HermiteCurve::evaluateSecondDerivative(const double & t_in){
  double s = this->clamp(t_in/t_dur);
  return (p1*(12*s - 6)  + 
         p2*(-12*s + 6) +
         v1*t_dur*(6*s - 4)  + 
         v2*t_dur*(6*s - 2)) /t_dur/t_dur; 
}

double HermiteCurve::clamp(const double & s_in, double lo, double hi){
    if (s_in < lo){
        return lo;
    }
    else if(s_in > hi){
        return hi;
    }else{
        return s_in;
    }
}

// Constructor
HermiteCurveVec::HermiteCurveVec(){}
// Destructor 
HermiteCurveVec::~HermiteCurveVec(){}

HermiteCurveVec::HermiteCurveVec(const Eigen::VectorXd& start_pos, 
                                const Eigen::VectorXd& start_vel, 
                                const Eigen::VectorXd& end_pos, 
                                const Eigen::VectorXd& end_vel, 
                                const double & duration){
	initialize(start_pos, start_vel, end_pos, end_vel, duration);
}

void HermiteCurveVec::initialize(const Eigen::VectorXd & start_pos, 
                                const Eigen::VectorXd & start_vel, 
								const Eigen::VectorXd & end_pos, 
                                const Eigen::VectorXd & end_vel, 
                                const double & duration){
	// Clear and 	create N hermite curves with the specified boundary conditions
	curves.clear();
	p1 = start_pos;	v1 = start_vel;
	p2 = end_pos; v2 = end_vel;
	t_dur = duration;

	for(int i = 0; i < start_pos.size(); i++){
		curves.push_back(HermiteCurve(start_pos[i], start_vel[i], end_pos[i], end_vel[i], t_dur));
	}
	output = Eigen::VectorXd::Zero(start_pos.size());
}

// Evaluation functions
Eigen::VectorXd HermiteCurveVec::evaluate(const double & t_in){
	for(int i = 0; i < p1.size(); i++){
		output[i] = curves[i].evaluate(t_in);
	}
	return output;
}

Eigen::VectorXd HermiteCurveVec::evaluateFirstDerivative(const double & t_in){
	for(int i = 0; i < p1.size(); i++){
		output[i] = curves[i].evaluateFirstDerivative(t_in);
	}
	return output;
}

Eigen::VectorXd HermiteCurveVec::evaluateSecondDerivative(const double & t_in){
	for(int i = 0; i < p1.size(); i++){
		output[i] = curves[i].evaluateSecondDerivative(t_in);
	}
	return output;
}

// hermite orientation spline on so(3) 
// Note that we implement the global frame case not the local frame!

HermiteQuaternionCurve::HermiteQuaternionCurve(){ }

HermiteQuaternionCurve::HermiteQuaternionCurve(
    const Eigen::Quaterniond& quat_start,
    const Eigen::Vector3d& angular_velocity_start,
    const Eigen::Quaterniond& quat_end,
    const Eigen::Vector3d& angular_velocity_end,
    double duration) {
  initialize(quat_start, angular_velocity_start, quat_end,
             angular_velocity_end, duration);
}

HermiteQuaternionCurve::~HermiteQuaternionCurve(){}

void HermiteQuaternionCurve::initialize(
    const Eigen::Quaterniond& quat_start,
    const Eigen::Vector3d& angular_velocity_start,
    const Eigen::Quaterniond& quat_end,
    const Eigen::Vector3d& angular_velocity_end,
    double duration) {
  qa = quat_start;
  omega_a = angular_velocity_start;

  qb = quat_end;
  omega_b = angular_velocity_end;

  t_dur = duration;

  // q(t) = exp( theta(t) ) * qa : global frame
  // q(t) = qa * exp( theta(t) ) : local frame
  // where theta(t) is hermite cubic spline with
  // theta(0) = 0, theta(t_dur) = log(delq_ab)
  // dot_theta(0) = omega_a, dot_theta(1) = omega_b

  Eigen::AngleAxisd delq_ab = Eigen::AngleAxisd(qb*qa.inverse());
  // Eigen::AngleAxisd delq_ab = Eigen::AngleAxisd(qa.inverse()*qb);

  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd start_vel = omega_a;
  Eigen::VectorXd end_pos = delq_ab.axis() * delq_ab.angle(); 
  Eigen::VectorXd end_vel = omega_b;

  theta_ab.initialize(start_pos, start_vel, end_pos, end_vel, t_dur);
}

void HermiteQuaternionCurve::evaluate(const double & t_in, Eigen::Quaterniond & quat_out){
  Eigen::VectorXd delq_vec = theta_ab.evaluate(t_in);

  Eigen::Quaterniond delq;
  if(delq_vec.norm() < 1e-6)
    delq = Eigen::Quaterniond(1, 0, 0, 0);
  else 
    delq = Eigen::AngleAxisd(delq_vec.norm(), delq_vec/delq_vec.norm());

  // quat_out = qa * delq; // local frame
  quat_out = delq * qa; // global frame
}

void HermiteQuaternionCurve::getAngularVelocity(const double & t_in, Eigen::Vector3d & ang_vel_out){
  ang_vel_out = theta_ab.evaluateFirstDerivative(t_in);
}

// For world frame
void HermiteQuaternionCurve::getAngularAcceleration(const double & t_in, Eigen::Vector3d & ang_acc_out){
  ang_acc_out = theta_ab.evaluateSecondDerivative(t_in);
  // not sure about this
}
