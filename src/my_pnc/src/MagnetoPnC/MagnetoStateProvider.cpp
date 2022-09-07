#include <my_robot_system/RobotSystem.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
#include <my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>


MagnetoStateProvider* MagnetoStateProvider::getStateProvider(
    RobotSystem* _robot) {
    static MagnetoStateProvider state_provider_(_robot);
    return &state_provider_;
}

MagnetoStateProvider::MagnetoStateProvider(RobotSystem* _robot) {
    my_utils::pretty_constructor(1, "Magneto State Provider");

    b_do_planning = false;

    num_step_copy = 0;
    phase_copy = 0;
    robot_ = _robot;
    stance_foot = MagnetoBodyNode::base_link ; //leftFoot
    curr_time = 0.;
    prev_state_machine_time = 0.;
    planning_moment = 0.;

    q = Eigen::VectorXd::Zero(Magneto::n_dof);
    qdot = Eigen::VectorXd::Zero(Magneto::n_dof);

    

    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        b_foot_contact_list[foot_idx] = 0;
        b_contact_plan_list[foot_idx]=false;
    }
    

    foot_pos_target = Eigen::VectorXd::Zero(3);

    foot_target_list.clear();
    com_des_list.clear();
    feasible_com_list.clear();
    com_pos_ini_step = Eigen::VectorXd::Zero(3);
    com_pos_des_step = Eigen::VectorXd::Zero(3);
    check_com_planner_updated = 0;
    check_foot_planner_updated = 0;

    com_pos = Eigen::VectorXd::Zero(3);
    com_vel = Eigen::VectorXd::Zero(3);
    mom = Eigen::VectorXd::Zero(6);
    est_com_vel = Eigen::VectorXd::Zero(3);
    // TODO linking est_com_vel with stateEstimator.

    com_pos_des = Eigen::VectorXd::Zero(3);
    com_vel_des = Eigen::VectorXd::Zero(3);
    mom_des = Eigen::VectorXd::Zero(6);    

    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        foot_pos[foot_idx] = Eigen::VectorXd::Zero(3);
        foot_vel[foot_idx] = Eigen::VectorXd::Zero(3);
        foot_pos_des[foot_idx] = Eigen::VectorXd::Zero(3);
        foot_vel_des[foot_idx] = Eigen::VectorXd::Zero(3);
        foot_ang_vel[foot_idx] = Eigen::VectorXd::Zero(3);
        foot_ang_vel_des[foot_idx] = Eigen::VectorXd::Zero(3);
        foot_ori_quat[foot_idx] = Eigen::Quaternion<double>::Identity();
        foot_ori_quat_des[foot_idx] = Eigen::Quaternion<double>::Identity();

        foot_grf[foot_idx] = Eigen::VectorXd::Zero(6);
        foot_grf_des[foot_idx] = Eigen::VectorXd::Zero(6);
    }


    des_jacc_cmd = Eigen::VectorXd::Zero(Magneto::n_adof);

}

void MagnetoStateProvider::setContactPlan(const int& moving_foot_idx){
    //b_contact_plan_map
    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        b_contact_plan_list[foot_idx]=(moving_foot_idx!=foot_idx);
    }
}


void MagnetoStateProvider::saveCurrentData() {
    for (int i = 0; i < 3; ++i) {
        com_pos[i] = robot_->getCoMPosition()[i];
        com_vel[i] = robot_->getCoMVelocity()[i];
    }

    mom = robot_->getCentroidMomentum();

    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        foot_pos[foot_idx] = robot_->getBodyNodeIsometry(
            MagnetoFoot::LinkIdx[foot_idx]).translation(); ;
        foot_vel[foot_idx] = robot_->getBodyNodeSpatialVelocity(
            MagnetoFoot::LinkIdx[foot_idx] ).tail(3); 

        foot_ori_quat[foot_idx] = Eigen::Quaternion<double>(
            robot_->getBodyNodeIsometry(MagnetoFoot::LinkIdx[foot_idx] ).linear());;

        foot_ang_vel[foot_idx] = robot_->getBodyNodeSpatialVelocity(
            MagnetoFoot::LinkIdx[foot_idx] ).head(3);
    }

    base_ori = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(MagnetoBodyNode::base_link ).linear()); //base

    base_ang_vel =
        robot_->getBodyNodeSpatialVelocity(MagnetoBodyNode::base_link).head(3); //base

}


Eigen::VectorXd MagnetoStateProvider::getActiveJointValue()
{
    return getActiveJointValue(q);
}

Eigen::VectorXd MagnetoStateProvider::getActiveJointValue(const Eigen::VectorXd& q_full){
    Eigen::VectorXd q_a(Magneto::n_adof);
    for(int i = 0; i < Magneto::n_adof; ++i)        
        q_a[i] = q_full[Magneto::idx_adof[i]]; 

    return  q_a;
}

Eigen::VectorXd MagnetoStateProvider::getVirtualJointValue(){
    return getVirtualJointValue(q);
}

Eigen::VectorXd MagnetoStateProvider::getVirtualJointValue(const Eigen::VectorXd& q_full){
    Eigen::VectorXd q_v(Magneto::n_vdof);
    for(int i = 0; i < Magneto::n_vdof; ++i)    
        q_v[i] = q_full[Magneto::idx_vdof[i]];

    return q_v;
}

Eigen::VectorXd MagnetoStateProvider::getFullJointValue(const Eigen::VectorXd& q_a)
{
    Eigen::VectorXd q_full(Magneto::n_dof);
    for(int i = 0; i < Magneto::n_vdof; ++i)    
        q_full[Magneto::idx_vdof[i]] = 0.0;
    for(int i = 0; i < Magneto::n_adof; ++i)        
        q_full[Magneto::idx_adof[i]] = q_a[i]; 
    return q_full;
}

Eigen::VectorXd MagnetoStateProvider::getFullJointValue(const Eigen::VectorXd& q_a, const Eigen::VectorXd& q_v)
{
    Eigen::VectorXd q_full(Magneto::n_dof);
    for(int i = 0; i < Magneto::n_vdof; ++i)    
        q_full[Magneto::idx_vdof[i]] = q_v[i];
    for(int i = 0; i < Magneto::n_adof; ++i)        
        q_full[Magneto::idx_adof[i]] = q_a[i]; 
    return q_full;
}

void MagnetoStateProvider::divideJoints2AnV(const Eigen::VectorXd& q_full, Eigen::VectorXd& q_a, Eigen::VectorXd& q_v) {
    if(q_full.size() == Magneto::n_dof)
    {
        q_a = getActiveJointValue(q_full);
        q_v = getVirtualJointValue(q_full);
    }    
}