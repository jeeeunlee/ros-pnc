#pragma once


#include <Eigen/Dense>
#include "pnc_utils/math_utilities.hpp"
#include <deque>

// 2nd order low-pass-filter
class LowPassFilter2{
public:
	LowPassFilter2(){
        zeta = 0.9;
        Ts=0.001;
        input_dim=0;
        b_initialized = false;
    }
	~LowPassFilter2() {}
	
	void initialize(int idim, double cutoff) {
        cut_off_frequency=cutoff;
        double Wn = cut_off_frequency*2.*M_PI;
        cDen[0] = 4.0+4.0*zeta*Ts*Wn + Ts*Wn*Ts*Wn;
		cDen[1] = 2.0*Ts*Wn*Ts*Wn - 8.0;
		cDen[2] = 4.0-4.0*zeta*Ts*Wn + Ts*Wn*Ts*Wn;
		cNum[0] = Ts*Ts*Wn*Wn;
		cNum[1] = 2.0*Ts*Ts*Wn*Wn;
		cNum[2] = Ts*Ts*Wn*Wn;
        input_dim = idim;
        filtered_val_0 = Eigen::VectorXd::Zero(input_dim);
        filtered_val_1 = Eigen::VectorXd::Zero(input_dim);
        filtered_val_2 = Eigen::VectorXd::Zero(input_dim);
        input_val_0 = Eigen::VectorXd::Zero(input_dim);
        input_val_1 = Eigen::VectorXd::Zero(input_dim);
        input_val_2 = Eigen::VectorXd::Zero(input_dim);
        b_initialized = true;
    }
	Eigen::VectorXd update(const Eigen::VectorXd & s_in){
        if(!b_initialized){
            std::cout<<"LowPassFilter2 didn't initialized"<< std::endl;
            initialize(s_in.size(), 30.);
        }
        input_val_0 = s_in;
        filtered_val_0 = ( cNum[0]*input_val_0 + cNum[1]*input_val_1 + cNum[2]*input_val_2
                            - cDen[1]*filtered_val_1 - cDen[2]*filtered_val_2) / cDen[0];
        
        filtered_val_2 = filtered_val_1;
        filtered_val_1 = filtered_val_0;
        input_val_2 = input_val_1;
        input_val_1 = input_val_0;

        return filtered_val_0;
    }
public:
	double cut_off_frequency;
    double zeta;
    double Ts;

private:
    int input_dim;
    bool b_initialized;

    Eigen::Vector3d cDen; // denominator coeff
    Eigen::Vector3d cNum; // numerator coeff

    Eigen::VectorXd filtered_val_0;
    Eigen::VectorXd filtered_val_1;
    Eigen::VectorXd filtered_val_2;

    Eigen::VectorXd input_val_0;
    Eigen::VectorXd input_val_1;
    Eigen::VectorXd input_val_2;
};



class SimpleSystemParam{
    // x(k+1) = F*x(k) + w(k), w(k)~N(0,Q)
    // z(k) = H*x(k) + v(k), v(k)~N(0,R)
    public:
        Eigen::MatrixXd F;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd H;
        Eigen::MatrixXd R;
        SimpleSystemParam(){}
        ~SimpleSystemParam(){}
};

class SimpleKalmanFilter{
public:
	SimpleKalmanFilter() { b_initialize= false;}
	~SimpleKalmanFilter() {}
	
	void initialize(const Eigen::VectorXd& x0,
                    const Eigen::MatrixXd& P0) {
        x_hat_ = x0;
        P_hat_ = P0; 
        n_ = x0.size();
        b_initialize = true;  
        n_err_change_ = 10;
        err_change_.clear();
    }

    void propagate(const Eigen::VectorXd& z, Eigen::VectorXd& xhat, void* system_data = NULL){
        if(system_data) sys_param_ = static_cast<SimpleSystemParam*>(system_data);

        xhat = x_hat_; // prev
        predict();
        update(z);        
        
        // err_change_.push_back( (x_hat_ - xhat).norm());
        err_change_.push_back( std::fabs(x_hat_(0) - xhat(0)) );
        if(err_change_.size()>n_err_change_) err_change_.pop_front();
        xhat = x_hat_;
    }

    void predict() {
        x_pre_ =  sys_param_->F * x_hat_;
        P_pre_ =  sys_param_->F * P_hat_ * sys_param_->F.transpose() + sys_param_->Q;
    }

    void update(const Eigen::VectorXd & z) {
        S_ = sys_param_->H * P_pre_ * sys_param_->H.transpose() + sys_param_->R;
        pnc_utils::pseudoInverse(S_, 0.001, S_inv_);
        K_ = P_pre_ * sys_param_->H.transpose()*S_inv_;
        y_ = z - sys_param_->H * x_pre_;

        P_hat_ = (Eigen::MatrixXd::Identity(n_,n_) - K_*sys_param_->H)*P_pre_;        
        x_hat_ = x_pre_ + K_*y_;        
    }

    double getErrorChange(){
        if(err_change_.size()==n_err_change_) {
            // double sum = 0.;
            // for(auto &val : err_change_) sum += val;
            // return sum/err_change_.size();

            double max = 0.;
            for(auto &val : err_change_) max = max>val ? max:val;
            return max;
        }
        else
            return 1e5;
    }

public:
	bool b_initialize;

private:
    SimpleSystemParam* sys_param_;
    int n_; // x dim

    Eigen::VectorXd x_hat_;    
    Eigen::VectorXd x_pre_; //predict
    Eigen::VectorXd y_; // observation error
    

    Eigen::MatrixXd P_hat_;
    Eigen::MatrixXd P_pre_;
    Eigen::MatrixXd S_; // innovation
    Eigen::MatrixXd S_inv_;
    Eigen::MatrixXd K_; // kalman gain

    // additional function
    std::deque<double> err_change_;
    int n_err_change_;   
};
