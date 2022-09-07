#include <my_robot_system/RobotSystem.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateEstimator.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
#include <my_utils/IO/IOUtilities.hpp>

MagnetoStateEstimator::MagnetoStateEstimator(RobotSystem* robot) {
    my_utils::pretty_constructor(1, "Magneto State Estimator");

    robot_ = robot;
    sp_ = MagnetoStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(Magneto::n_dof);
    curr_qdot_ = Eigen::VectorXd::Zero(Magneto::n_dof);
}

MagnetoStateEstimator::~MagnetoStateEstimator() {}

void MagnetoStateEstimator::Initialization(MagnetoSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    sp_->jpos_ini = data->q; //sp_->getActiveJointValue(curr_config_);
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void MagnetoStateEstimator::Update(MagnetoSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void MagnetoStateEstimator::_JointUpdate(MagnetoSensorData* data) {
    curr_config_.setZero();
    curr_qdot_.setZero();
    for (int i = 0; i < Magneto::n_vdof; ++i) {
        curr_config_[Magneto::idx_vdof[i]] = data->virtual_q[i];
        curr_qdot_[Magneto::idx_vdof[i]] = data->virtual_qdot[i];
    }
    for (int i = 0; i < Magneto::n_adof; ++i) {
        curr_config_[Magneto::idx_adof[i]] = data->q[i];
        curr_qdot_[Magneto::idx_adof[i]] = data->qdot[i];
    }

    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
            sp_->foot_grf[foot_idx] = data->foot_wrench[foot_idx];
    }
}

void MagnetoStateEstimator::_ConfigurationAndModelUpdate() {
    robot_->updateSystem(curr_config_, curr_qdot_, true);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
}

void MagnetoStateEstimator::_FootContactUpdate(MagnetoSensorData* data) {
    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        sp_->b_foot_contact_list[foot_idx] 
            = data->b_foot_contact[foot_idx] ? 1:0;
    }   
}
