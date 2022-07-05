#pragma once

#include <pnc_utils/../../Configuration.h>
#include <magneto_pnc/magneto_estimator/magneto_state_estimator.hpp>
#include <Eigen/Dense>

class MagnetoStateProvider;
class RobotSystem;
class MagnetoSensorData;

class MagnetoHWStateEstimator : public MagnetoStateEstimator{
   public:
    MagnetoHWStateEstimator(RobotSystem* robot);
    ~MagnetoHWStateEstimator();

    void Initialization(MagnetoSensorData*);
    void Update(MagnetoSensorData*);

   protected:
    // MagnetoStateProvider* sp_;
    // RobotSystem* robot_;

    // Eigen::VectorXd curr_config_;
    // Eigen::VectorXd curr_qdot_;
    // Eigen::VectorXd prev_config_;
    // Eigen::VectorXd prev_tau_cmd_;

    void _JointUpdate(MagnetoSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(MagnetoSensorData* data);

    void _EstimateVirtualJointState(MagnetoSensorData* data);
    void _InitializeVirtualJointState(MagnetoSensorData* data);
};
