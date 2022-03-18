#include <my_robot_system/RobotSystem.hpp>
#include <my_robot_core/magneto_core/magneto_planner/magneto_com_planner.hpp>

// #include <my_robot_core/magneto_core/magneto_interface.hpp>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>
#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_wbc/Magnet/MagnetSpec.hpp>

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include "my_utils/Math/pseudo_inverse.hpp"

MagnetoCoMPlanner::MagnetoCoMPlanner(RobotSystem* robot) {
    my_utils::pretty_constructor(2, "Magneto CoM Hermite Spline Parameter Planner");

    // robot system
    robot_ = robot;
    sp_ = MagnetoStateProvider::getStateProvider(robot_);

    mass_ = robot_->getRobotMass();
    grav_.setZero();
    grav_(2) = -9.81;
    zero_vel_.setZero();
    Ldot_ = zero_vel_;


    initialized = false;
    Tt_ = 0.0; 
}

void MagnetoCoMPlanner::_solveSwingQuadProg(){
    // Centroidal Dynamics
    // A*Fc = S(s,sddot)*delP + C
    // solve Fc with weighted inverse matirx
    // Fc = invA*(S(s,sddot)*delP + C)

    Eigen::MatrixXd AWA = A_*invWc_*A_.transpose();
    Eigen::MatrixXd AWAinv;
    my_utils::pseudoInverse(AWA, 0.0001, AWAinv);

    Eigen::MatrixXd invA = invWc_*A_.transpose()*AWAinv;
    Eigen::VectorXd dd = -Dc_*invA*C_ + dc_;

    Eigen::MatrixXd invA1 = invA.leftCols<3>();
    Eigen::MatrixXd invA2 = invA.rightCols<3>();

    ssdot_swing_ = -2./Ts_/Ts_;
    Eigen::MatrixXd DD1, DD2, DD;
    // centroidal dynamics soln applied friction cone
    // D( m*(-1)*invA(:,1:3)*skew(g) + m*sddot*invA(:,4:6))*delP + D*invA*C - d >=0
    
    // when s=0
    // DD1*delP - dd >=0
    DD1 = Dc_*( -mass_*invA1*my_utils::skew(grav_) + mass_*ssdot_swing_*invA2);
    // when s=1
    // DD2*delP - dd >=0
    DD2 = Dc_*mass_*ssdot_swing_*invA2;
    DD = my_utils::vStack(DD1,DD2);
    dd = my_utils::vStack(dd,dd);


}

void MagnetoCoMPlanner::_buildFm(
    const std::array<MagnetSpec*, Magneto::n_leg>& f_mag){
    Fmf_ = Eigen::VectorXd::Zero(0);
    Fmc_ = Eigen::VectorXd::Zero(0);
    for(auto &mag : f_mag) {
        Fmf_= my_utils::vStack(Fmf_, -mag->getMagneticForce());
        if(mag->getLinkIdx() !=  swing_foot_idx_){
            Fmc_ = my_utils::vStack(Fmc_, -mag->getMagneticForce());
        }
    }
    
}

void MagnetoCoMPlanner::_buildFrictionCone(
    const std::array<ContactSpec*, Magneto::n_leg>& f_contacts){
    Df_ = Eigen::MatrixXd::Zero(0,0);
    Dc_ = Eigen::MatrixXd::Zero(0,0);
    df_ = Eigen::VectorXd::Zero(0);
    dc_ = Eigen::VectorXd::Zero(0);

    Eigen::MatrixXd Di;
    Eigen::VectorXd di;
    // Di*Fc >= di
    for(auto &contact : f_contacts) {
        ((BodyFramePointContactSpec*)contact)->getRFConstraintMtx(Di);
        ((BodyFramePointContactSpec*)contact)->getRFConstraintVec(di);
        Df_ = my_utils::dStack(Df_, Di);
        df_ = my_utils::vStack(df_, di);
        if(contact->getLinkIdx() != swing_foot_idx_){
            Dc_ = my_utils::dStack(Dc_, Di);
            dc_ = my_utils::vStack(dc_, di);
        }
    }    
}

void MagnetoCoMPlanner::_buildWeightMatrices(
    const std::array<ContactSpec*, Magneto::n_leg>& f_contacts){
    invWf_ = Eigen::MatrixXd::Zero(0,0);
    invWc_ = Eigen::MatrixXd::Zero(0,0);

    double mu;
    Eigen::VectorXd invwi;
    // Di*Fc >= di
    for(auto &contact : f_contacts) {
        mu = contact->getFrictionCoeff();
        // wi << 1./mu, 1./mu, mu;
        invwi = Eigen::VectorXd::Zero(3);
        invwi << mu, mu, 1./mu;
        invWf_ = my_utils::dStack(invWf_, invwi.asDiagonal());
        if(contact->getLinkIdx() != swing_foot_idx_){
            invWc_ = my_utils::dStack(invWc_, invwi.asDiagonal());
        }
    }
}

void MagnetoCoMPlanner::_buildSwingSystem(){
    int dim_f = Rc_.cols();
    A_ = Eigen::MatrixXd::Zero(6, dim_f);

    A_.topRows(3) = Pc_-my_utils::skew(p_goal_)*Rc_;
    A_.bottomRows(3) = Rc_;

    C_ = Eigen::VectorXd::Zero(6);
    C_.head(3) = Ldot_ + (my_utils::skew(p_goal_)*Rc_-Pc_)*Fmc_;
    C_.tail(3) = -mass_*grav_ - Rc_*Fmc_;
}


void MagnetoCoMPlanner::_buildPfRf() {
    Pf_ = Eigen::MatrixXd::Zero(0,0);
    Rf_ = Eigen::MatrixXd::Zero(0,0);
    Pc_ = Eigen::MatrixXd::Zero(0,0);
    Rc_ = Eigen::MatrixXd::Zero(0,0);

    Eigen::MatrixXd PRi, Ri;
    Eigen::VectorXd pi;
    for(int i(0); i<Magneto::n_leg; ++i) {        
        pi = robot_->getBodyNodeIsometry(
            MagnetoFoot::LinkIdx[i]).translation();
        Ri = robot_->getBodyNodeIsometry(
            MagnetoFoot::LinkIdx[i]).linear();            
        PRi = my_utils::skew(pi)*Ri;

        Pf_ = my_utils::hStack(Pf_, PRi);
        Rf_ = my_utils::hStack(Rf_, Ri);
        if(MagnetoFoot::LinkIdx[i] != swing_foot_idx_){
            Pc_ = my_utils::hStack(Pc_, PRi);
            Rc_ = my_utils::hStack(Rc_, Ri);
        }
    }
}



void MagnetoCoMPlanner::computeSequence(const Eigen::Vector3d& pcom_goal,
                                        MotionCommand &_motion_command,
                                        const std::array<ContactSpec*, Magneto::n_leg>& f_contacts,
                                        const std::array<MagnetSpec*, Magneto::n_leg>& f_mag){
    p_init_ = robot_->getCoMPosition();  
    p_goal_ = pcom_goal;

    // next foot configuration
    MOTION_DATA md;    
    _motion_command.get_foot_motion(md, swing_foot_idx_);
    // Tt_ set in advance
    Ts_ = _motion_command.get_foot_motion_period();
    Tf_ = Ts_;

    std::cout<<"Ts="<<Ts_<<std::endl;
    std::cout<<"Tf_="<<Tf_<<std::endl;
    std::cout<<"Tt="<<Tt_<<std::endl;

    // set matrices
    _buildFrictionCone(f_contacts);
    _buildWeightMatrices(f_contacts);
    _buildFm(f_mag);
    _buildPfRf();

    // solve problem
    // _buildSwingSystem();
    // _solveSwingQuadProg(); // get delP_swing_

    delP_swing_.setZero();
    delP_swing_ << -0.0504, 0.0005, 0.0407;
    delP_swing_ = delP_swing_/2.0;

    p_swing_init_ = p_goal_ - delP_swing_;
    v_swing_init_ = 2./(Ts_+2*Tt_)*delP_swing_;
    acc_swing_ = ssdot_swing_*delP_swing_;    
}

ComMotionCommand MagnetoCoMPlanner::getFullSupportCoMCmd() {
    Eigen::Vector3d pa = robot_ ->getCoMPosition();  
    Eigen::Vector3d va = zero_vel_;

    return ComMotionCommand( pa, va, p_swing_init_, v_swing_init_, Tf_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingStartCoMCmd() {
    // constant acc
    Eigen::Vector3d pa = sp_->com_pos_des;
    Eigen::Vector3d va = sp_->com_vel_des;

    return ComMotionCommand( pa, va, acc_swing_, Tt_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingCoMCmd() {
    // constant acc
    Eigen::Vector3d pa = sp_->com_pos_des;
    Eigen::Vector3d va = sp_->com_vel_des;

    return ComMotionCommand( pa, va, acc_swing_, Ts_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingEndCoMCmd() {
    Eigen::Vector3d pa = sp_->com_pos_des;
    Eigen::Vector3d va = sp_->com_vel_des;

    Eigen::Vector3d p_mid = 0.5*(pa + p_goal_);
    
    return ComMotionCommand( pa, va, p_mid, zero_vel_, Tt_ );
}




