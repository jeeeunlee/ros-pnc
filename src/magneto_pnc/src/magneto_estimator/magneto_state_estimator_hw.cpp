#include <pnc_utils/robot_system.hpp>
#include <magneto_pnc/magneto_definition.hpp>
#include <magneto_pnc/magneto_interface.hpp>
#include <magneto_pnc/magneto_estimator/magneto_state_estimator_hw.hpp>
#include <magneto_pnc/magneto_state_provider.hpp>
#include <pnc_utils/io_utilities.hpp>
#include <pnc_utils/math_utilities.hpp>

MagnetoHWStateEstimator::MagnetoHWStateEstimator(RobotSystem* robot)
    :MagnetoStateEstimator(robot) {
    // pnc_utils::pretty_constructor(1, "Magneto State Estimator");

    // robot_ = robot;
    // sp_ = MagnetoStateProvider::getStateProvider(robot_);
    // curr_config_ = Eigen::VectorXd::Zero(Magneto::n_dof);
    // prev_config_ = Eigen::VectorXd::Zero(Magneto::n_dof);
    // curr_qdot_ = Eigen::VectorXd::Zero(Magneto::n_dof);    
    // prev_tau_cmd_ = Eigen::VectorXd::Zero(Magneto::n_dof);
}

MagnetoHWStateEstimator::~MagnetoHWStateEstimator() {}

void MagnetoHWStateEstimator::Initialization(MagnetoSensorData* data) {
    std::cout<< "MagnetoHWStateEstimator - Initialization"<<std::endl;
    sp_->jpos_ini = data->q; //sp_->getActiveJointValue(curr_config_);
    _JointUpdate(data);
    _InitializeVirtualJointState(data);
    _ConfigurationAndModelUpdate();    
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void MagnetoHWStateEstimator::Update(MagnetoSensorData* data) {
    _JointUpdate(data);
    _EstimateVirtualJointState(data);
    _ConfigurationAndModelUpdate();
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void MagnetoHWStateEstimator::_JointUpdate(MagnetoSensorData* data) {
    prev_config_ = curr_config_;
    curr_config_.setZero();
    curr_qdot_.setZero();
    prev_tau_cmd_.setZero();
    // for (int i = 0; i < Magneto::n_vdof; ++i) {
    //     curr_config_[Magneto::idx_vdof[i]] = data->virtual_q[i];
    //     curr_qdot_[Magneto::idx_vdof[i]] = data->virtual_qdot[i];
    // }
    for (int i = 0; i < Magneto::n_adof; ++i) {
        curr_config_[Magneto::idx_adof[i]] = data->q[i];
        curr_qdot_[Magneto::idx_adof[i]] = data->qdot[i];
        prev_tau_cmd_[Magneto::idx_adof[i]] = data->tau_cmd_prev[i];
    }
}

void MagnetoHWStateEstimator::_ConfigurationAndModelUpdate() {    
    robot_->updateSystem(curr_config_, curr_qdot_, true);
    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
    sp_->tau_cmd_prev = prev_tau_cmd_;
}

void MagnetoHWStateEstimator::_FootContactUpdate(MagnetoSensorData* data) {
    sp_->b_arfoot_contact = data->arfoot_contact ? 1:0;
    sp_->b_brfoot_contact = data->brfoot_contact ? 1:0;
    sp_->b_alfoot_contact = data->alfoot_contact ? 1:0;
    sp_->b_blfoot_contact = data->blfoot_contact ? 1:0;

    sp_->al_rf = data->alf_wrench;
    sp_->bl_rf = data->blf_wrench;
    sp_->ar_rf = data->arf_wrench;
    sp_->br_rf = data->brf_wrench;

    sp_->surface_normal = data->surface_normal;
}

void MagnetoHWStateEstimator::_EstimateVirtualJointState(MagnetoSensorData* data) {
    // IMU integration will be added later

    // initialize
    for (int i = 0; i < Magneto::n_vdof; ++i) {
        curr_config_[Magneto::idx_vdof[i]] = prev_config_[Magneto::idx_vdof[i]];
        curr_qdot_[Magneto::idx_vdof[i]] = 0.0;
    }

    // contact jacobian
    Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(0,0);
    Eigen::MatrixXd Jtemp;
    int nc = 0;
    std::vector<int> idx_contact_passive={};
    if(data->arfoot_contact){
        nc++;
        Jtemp = robot_->getBodyNodeCoMJacobian(MagnetoFoot::LinkIdx[MagnetoFoot::AL]);
        Jc = pnc_utils::vStack(Jc, Jtemp);        
        idx_contact_passive.insert(idx_contact_passive.end(),
                                    std::begin(Magneto::idx_al_pdof), 
                                    std::end(Magneto::idx_al_pdof));        
    }    
    if(data->brfoot_contact){
        nc++;
        Jtemp = robot_->getBodyNodeCoMJacobian(MagnetoFoot::LinkIdx[MagnetoFoot::AR]);
        Jc = pnc_utils::vStack(Jc, Jtemp);        
        idx_contact_passive.insert(idx_contact_passive.end(),
                                    std::begin(Magneto::idx_ar_pdof), 
                                    std::end(Magneto::idx_ar_pdof));        
    }
    if(data->alfoot_contact){
        nc++;
        Jtemp = robot_->getBodyNodeCoMJacobian(MagnetoFoot::LinkIdx[MagnetoFoot::BL]);
        Jc = pnc_utils::vStack(Jc, Jtemp);        
        idx_contact_passive.insert(idx_contact_passive.end(),
                                    std::begin(Magneto::idx_bl_pdof), 
                                    std::end(Magneto::idx_bl_pdof));        
    }    
    if(data->blfoot_contact){
        nc++;
        Jtemp = robot_->getBodyNodeCoMJacobian(MagnetoFoot::LinkIdx[MagnetoFoot::BR]);
        Jc = pnc_utils::vStack(Jc, Jtemp);        
        idx_contact_passive.insert(idx_contact_passive.end(),
                                    std::begin(Magneto::idx_br_pdof), 
                                    std::end(Magneto::idx_br_pdof));        
    }

    // Assume num of contact >= 3 
    std::cout << "nc = "<< nc << " / ";
    if(Jc.cols()>0){
        // build Jc w.r.t joint type        
        Eigen::MatrixXd Jc_virtual = Jc.leftCols<6>();
        Eigen::MatrixXd Jc_passive = Eigen::MatrixXd::Zero( Jc.rows() , idx_contact_passive.size() );
        Eigen::MatrixXd Jc_active = Eigen::MatrixXd::Zero( Jc.rows() , Magneto::n_adof);        
        for (int i=0; i<idx_contact_passive.size(); ++i){
            Jc_passive.col(i) = Jc.col(idx_contact_passive[i]);
        }
        for (int i =0; i<Magneto::n_adof; ++i){
            Jc_active.col(i) = Jc.col(Magneto::idx_adof[i]);
        }

        // update qdot : // curr_qdot_
        // 0 = xcdot = Jc(a)*qdot(a) + Jc(v)*qdot(v) + Jc(p)*qdot(p)
        // - Jc(a)*qdot(a) = [Jc(v) Jc(p)] * [qdot(v); qdot(p)]
        Eigen::MatrixXd Jc_vnp = pnc_utils::hStack(Jc_virtual, Jc_passive);
        Eigen::MatrixXd Jc_vnp_inv;
        pnc_utils::pseudoInverse(Jc_vnp, 0.0001 , Jc_vnp_inv);
        Eigen::VectorXd qdot_vnp = Jc_vnp_inv*(-Jc_active*robot_->getActiveQdot());
        for (int i = 0; i < 6; ++i){
            curr_qdot_[Magneto::idx_vdof[i]] = qdot_vnp[i];            
        }
        for (int i=0; i<idx_contact_passive.size(); ++i){
            curr_qdot_[idx_contact_passive[i]] = qdot_vnp[i+6];
        }

        // update q = q+delq: // curr_config_ = prev_config_ + delq
        // - Jc(a)*delq(a) = [Jc(v) Jc(p)] * [delq(v); delq(p)]

        Eigen::VectorXd delq_vnp = Jc_vnp_inv*(
                        -Jc_active*(robot_->getActiveJointValue(
                                    curr_config_-prev_config_)));
        for (int i = 0; i < 6; ++i){
            curr_config_[Magneto::idx_vdof[i]] = 
                    prev_config_[Magneto::idx_vdof[i]] 
                    + delq_vnp[i];            
        }
        for (int i=0; i<idx_contact_passive.size(); ++i){
            curr_config_[idx_contact_passive[i]] = 
                    prev_config_[Magneto::idx_vdof[i]] 
                    + delq_vnp[i+6];
        }
    }
    pnc_utils::pretty_print(curr_config_, std::cout, "curr_config_");
    // pnc_utils::pretty_print(curr_qdot_, std::cout, "curr_qdot_");
}

void MagnetoHWStateEstimator::_InitializeVirtualJointState(MagnetoSensorData* data){
    // TODO : solve for passive joint given the surface orientation

    for (int i = 0; i < Magneto::n_vdof; ++i) {
        curr_config_[Magneto::idx_vdof[i]] = 0.0;
        curr_qdot_[Magneto::idx_vdof[i]] = 0.0; 
    }

    // initialize the orientation given the imu value 
    // Assume stationary state & no bias

    Eigen::Vector3d gdir = data->imu_data.linear_acceleration;
    gdir.normalize();
    
    double rx(0.), ry(0.), rz(0.);
    // solve for rz(C)*ry(B)*rx(A) = R
    // (-sin(B), cos(B)sin(A), cos(B)cos(A)) = gdir // C=0.

    ry = std::asin(-gdir(0));
    if( fabs(gdir(2))>1e-5 )
        rx = std::atan(gdir(1)/gdir(2));     

    curr_config_[MagnetoDoF::baseRotY] = ry;
    curr_config_[MagnetoDoF::_base_joint]  = rx;

    std::cout << " Initial Base Configuration is set to = 0,0,0, " 
                << rz << ", " << ry << ", " << rx << std::endl;

}