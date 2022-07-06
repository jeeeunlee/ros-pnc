#pragma once

#include <pnc_utils/../../Configuration.h>
#include <Eigen/Dense>

class MagnetoStateProvider;
class RobotSystem;
class MagnetoSensorData;

class MagnetoStateEstimator {
   public:
    MagnetoStateEstimator(RobotSystem* robot);
    ~MagnetoStateEstimator();

    virtual void Initialization(MagnetoSensorData*);
    virtual void Update(MagnetoSensorData*);

   protected:
    MagnetoStateProvider* sp_;
    RobotSystem* robot_;

    Eigen::VectorXd curr_config_;
    Eigen::VectorXd curr_qdot_;
    Eigen::VectorXd prev_config_;
    Eigen::VectorXd prev_tau_cmd_;

    virtual void _JointUpdate(MagnetoSensorData* data);
    virtual void _ConfigurationAndModelUpdate();
    virtual void _FootContactUpdate(MagnetoSensorData* data);
};
