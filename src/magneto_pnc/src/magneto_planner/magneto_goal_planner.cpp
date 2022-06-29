
#include <pnc_utils/../../Configuration.h>

#include <pnc_utils/robot_system.hpp>

#include <magneto_pnc/magneto_definition.hpp>
#include <magneto_pnc/magneto_interface.hpp>
#include <magneto_pnc/magneto_command.hpp>

#include <magneto_pnc/magneto_planner/magneto_goal_planner.hpp>
#include <pnc_utils/io_utilities.hpp>
#include <pnc_utils/math_utilities.hpp>


MagnetoGoalPlanner::MagnetoGoalPlanner(RobotSystem* robot) {
    pnc_utils::pretty_constructor(2, "Magneto CoM Goal Planner");
    // robot system
    robot_ = robot;    
    // copy constructor
    robot_planner_ = new RobotSystem(*robot_);

    // set cost function Matrices
    _InitCostFunction();

    // set Constraint
    _InitConstraints();
}

MagnetoGoalPlanner::~MagnetoGoalPlanner() {
  _DeleteConstraints();
}

void MagnetoGoalPlanner::_InitConstraints() {
  _DeleteConstraints();
  for(int i(0); i<Magneto::n_leg; ++i)
    constraint_list.push_back(
        new Constraint(robot_planner_, MagnetoFoot::LinkIdx[i]));
}

void MagnetoGoalPlanner::_DeleteConstraints() {
  for(auto &constraint : constraint_list)
    delete constraint;  
  constraint_list.clear();
}

void MagnetoGoalPlanner::_setDesiredFootPosition(const POSE_DATA &foot_pose_data,
                                                int moving_foot_idx) {
  // contact class / Task class >> Foot Class ?
  
  // initialize joint values
  q_ = robot_->getQ();
  dotq_ = robot_->getQdot(); 
  delq_ = Eigen::VectorXd::Zero(q_.size());
  _UpdateConfiguration(q_);
  _InitCostFunction();
  
  POSE_DATA zero_pose = POSE_DATA();
  for(auto &constraint : constraint_list){
    if(constraint->getLinkIdx() == MagnetoFoot::LinkIdx[moving_foot_idx]) 
      constraint->setDesired(foot_pose_data);
    else
      constraint->setDesired(zero_pose);    
  }
}

void MagnetoGoalPlanner::getGoalConfiguration(Eigen::VectorXd& _q_goal){
  _q_goal = q_goal_;
}

void MagnetoGoalPlanner::getGoalComPosition(Eigen::Vector3d& _pc_goal){
  _pc_goal = pc_goal_;
}


void MagnetoGoalPlanner::computeGoal(MotionCommand &_motion_command) {

  // assume one foot is moving
  SWING_DATA motion_data; 

  _motion_command.get_foot_motion(motion_data);
  _setDesiredFootPosition(motion_data.dpose, motion_data.foot_idx);

  Eigen::Vector3d p_com_curr = robot_planner_->getCoMPosition();;
  // std::cout<<"****************************************"<<std::endl;
  // pnc_utils::pretty_print(q_, std::cout, "q_init");
  // pnc_utils::pretty_print(p_com_curr, std::cout, "p_com_ini");
  

  double tol = 1e-5;
  double err = 1e5;
  int iter(0), max_iter(10000);
  while(err > tol && iter++ < max_iter) {
    // synchronize robot_planner
    _UpdateConfiguration(q_);
    // update constraint with robot_planner
    _UpdateConstraints();
    // compute c, Jacob, JacobNS
    _BuildConstraints();
    _BuildNSConstraints();
    // compute optimal delq
    _UpdateDelQ();
    // update q
    q_ += delq_;
    err = delq_.norm();
  }

  q_goal_ = q_;
  _UpdateConfiguration(q_goal_);
  pc_goal_ = robot_planner_->getCoMPosition();

  // std::cout<<"****************************************"<<std::endl;
  // pnc_utils::pretty_print(q_goal_, std::cout, "q_goal");
  // pnc_utils::pretty_print(pc_goal_, std::cout, "p_com_goal");
  // std::cout<<"****************************************"<<std::endl;
}

void MagnetoGoalPlanner::_UpdateDelQ() {
  // change the problem:
  // q'A q + b'q -> z'Aprime z + bprime'z
  Eigen::MatrixXd Aprime, AprimeInv;
  Eigen::VectorXd bprime;
  _computeCostWeight(Aprime,bprime);
  pnc_utils::pseudoInverse(Aprime, 0.0001, AprimeInv);

  // get optimal z
  Eigen::VectorXd z = - 0.5 * AprimeInv * bprime;
  // get optimal delq
  delq_ = 0.1 * (- cJacobInv_ * ceq_ + cJacobNull_ * z);
}

void MagnetoGoalPlanner::_computeCostWeight(
                          Eigen::MatrixXd& _Aprime,
                          Eigen::VectorXd& _bprime) {

  _Aprime = cJacobNull_.transpose() * A_ * cJacobNull_;
  _bprime = 2.0*cJacobNull_.transpose() * A_ * (q_ - cJacobInv_ *ceq_)
            + cJacobNull_.transpose() * b_;
}

void MagnetoGoalPlanner::_UpdateConstraints() {
  for(auto &constraint : constraint_list) {
      constraint->update();
  }
}

void MagnetoGoalPlanner::_BuildConstraints() {
  // only consider position _BuildConstraints

  // check Size
  int dim_J(0), dim_Jc(0), dim_c(0);
  for(auto &constraint : constraint_list) {
    dim_Jc = constraint->getJointDim();
    dim_J += constraint->getPositionDim();
    dim_c += constraint->getPositionDim();
  }

  // vercat ceq_, cJacobian_
  cJacobian_ = Eigen::MatrixXd::Zero(dim_J, dim_Jc);
  ceq_ = Eigen::VectorXd::Zero(dim_c);

  int dim_Ji(0), dim_ci(0);
  Eigen::MatrixXd Ji;
  Eigen::VectorXd ci;
  dim_J = 0;   dim_c = 0;
  for(auto &constraint : constraint_list) {
    constraint->getPositionJacobian(Ji);
    constraint->getPositionError(ci);

    dim_Ji = constraint->getPositionDim();
    dim_ci = constraint->getPositionDim();

    cJacobian_.block(dim_J,0, dim_Ji,dim_Jc) = Ji;
    ceq_.segment(dim_c, dim_ci) = ci;

    dim_J += dim_Ji;
    dim_c += dim_ci;
  }

  pnc_utils::pseudoInverse(cJacobian_, 0.0001, cJacobInv_); 
}


void MagnetoGoalPlanner::_BuildNSConstraints() {
  // build null space of constraint jacobian
  // cJacobian_ * cJacobNull_ = 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    cJacobian_, 
    Eigen::ComputeThinU | Eigen::ComputeFullV);
  // matrixV(), matrixU(), singularValues()

  int const nrows(svd.singularValues().rows());
  int const Vrows(svd.matrixV().rows());
  int const Vcols(svd.matrixV().cols());

  cJacobNull_ = svd.matrixV().
                  block(0, nrows, Vrows, Vcols-nrows);

  // if there are more singular value
  double const tol = 1e-5;
  for(int ii=0; ii<nrows; ++ii) {
    if(svd.singularValues().coeff(ii) < tol) {
      cJacobNull_ = svd.matrixV().
                  block(0, ii, Vrows, Vcols-ii);
      break;
    }
  }
} 

void MagnetoGoalPlanner::_UpdateConfiguration(
                        const Eigen::VectorXd& q) {
    bool b_centroidal = false;
    robot_planner_->updateSystem(q, 
                              dotq_, 
                              b_centroidal);   
}

void MagnetoGoalPlanner::_InitCostFunction() {
  double theta = 0.0;

  double theta3 = 0.0;
  for(int ii(0); ii<num_joint_dof_; ++ii) {
    if( _checkJoint(ii,MagnetoJointType::FEMUR) ) {
      theta += q_(ii);
    }
    if( _checkJoint(ii, MagnetoJointType::TIBIA) ) {
      theta += q_(ii);
      theta3+= q_(ii);
    }
  }
  theta = theta*0.25;
  theta3 = theta3*0.25; // -pi/2
  // std::cout<<" theta = " << theta << std::endl;
  // J(q) = alpha_1*(q2+q3-theta)^2 + alpha_2*(q3-theta3)^2 + alpha_3*(q1-q1_curr)^2
  // J(q) = q*A*q + b*q
  // J(q) = alpha_1*(q2^2) + 2*alpha_1*(q2*q3) + (alpha_1 + alpha_2)(q3^2)
  //        + alpha_1*(-2*theta)*(q2) + (alpha_1*(-2*theta) + 2*alpha_2*theta3)*q_3
  double alpha_1 = 0.5;
  double alpha_2 = 1.;
  double alpha_3 = 1.;
  // FEMUR:2, TIBIA:3
  num_joint_dof_ = robot_->getNumDofs();
  A_ = Eigen::MatrixXd::Zero(num_joint_dof_,num_joint_dof_);
  b_ = Eigen::VectorXd::Zero(num_joint_dof_);

  // base ori
  double beta = 0.5;
  q_ = robot_->getQ();
  A_(MagnetoDoF::baseRotZ, MagnetoDoF::baseRotZ) = beta;
  A_(MagnetoDoF::baseRotY, MagnetoDoF::baseRotY) = beta;
  A_(MagnetoDoF::_base_joint, MagnetoDoF::_base_joint) = beta;
  b_(MagnetoDoF::baseRotZ) = -2.0*beta*q_(MagnetoDoF::baseRotZ);
  b_(MagnetoDoF::baseRotY) = -2.0*beta*q_(MagnetoDoF::baseRotY);
  b_(MagnetoDoF::_base_joint) = -2.0*beta*q_(MagnetoDoF::_base_joint);

  // leg 
  for(int ii(0); ii<num_joint_dof_; ++ii) {
    // active
    if(_checkJoint(ii,MagnetoJointType::COXA)) {
      A_(ii,ii) = alpha_3;
      // b_(ii) =  -2.0*alpha_3*q_(ii);
      //b_(ii) =  -2.0*alpha_3*(q_(ii) + theta1)/2.0;
      // b_(ii) =  -2.0*alpha_3*theta1;
      b_(ii) =  0.0;
    }
    else if(_checkJoint(ii, MagnetoJointType::FEMUR)) {
      // theta = q_(ii) + q_(ii+1);
      b_(ii) = alpha_1*(-2*theta);
      A_(ii,ii) = alpha_1;
      A_(ii,ii+1) = alpha_1;
    } else if(_checkJoint(ii, MagnetoJointType::TIBIA)) {
      b_(ii) = alpha_1*(-2*theta) + alpha_2*(-theta3-q_(ii));
      A_(ii,ii) = alpha_1+ alpha_2;
      A_(ii,ii-1) = alpha_1;
    }

    // passive
    double gamma = 0.05;
    if(_checkJoint(ii, MagnetoJointType::FOOT1) ||
        _checkJoint(ii, MagnetoJointType::FOOT2) ||
        _checkJoint(ii, MagnetoJointType::FOOT3) ) {
          A_(ii,ii) = gamma;
    }
  }
}



bool MagnetoGoalPlanner::_checkJoint(int joint_idx, MagnetoJointType joint_type)
{
  switch(joint_type){
    case MagnetoJointType::COXA:
    if( joint_idx==MagnetoDoF::AL_coxa_joint ||
        joint_idx==MagnetoDoF::AR_coxa_joint ||
        joint_idx==MagnetoDoF::BL_coxa_joint ||
        joint_idx==MagnetoDoF::BR_coxa_joint)
      return true;
    break;
    case MagnetoJointType::FEMUR:
    if( joint_idx==MagnetoDoF::AL_femur_joint ||
        joint_idx==MagnetoDoF::AR_femur_joint ||
        joint_idx==MagnetoDoF::BL_femur_joint ||
        joint_idx==MagnetoDoF::BR_femur_joint)
      return true;
    break;
    case MagnetoJointType::TIBIA:
    if( joint_idx==MagnetoDoF::AL_tibia_joint ||
        joint_idx==MagnetoDoF::AR_tibia_joint ||
        joint_idx==MagnetoDoF::BL_tibia_joint ||
        joint_idx==MagnetoDoF::BR_tibia_joint)
      return true;
    break;
    case MagnetoJointType::FOOT1:
    if( joint_idx==MagnetoDoF::AL_foot_joint_1 ||
        joint_idx==MagnetoDoF::AR_foot_joint_1 ||
        joint_idx==MagnetoDoF::BL_foot_joint_1 ||
        joint_idx==MagnetoDoF::BR_foot_joint_1)
      return true;
    break;
    case MagnetoJointType::FOOT2:
    if( joint_idx==MagnetoDoF::AL_foot_joint_2 ||
        joint_idx==MagnetoDoF::AR_foot_joint_2 ||
        joint_idx==MagnetoDoF::BL_foot_joint_2 ||
        joint_idx==MagnetoDoF::BR_foot_joint_2)
      return true;
    break;
    case MagnetoJointType::FOOT3:
    if( joint_idx==MagnetoDoF::AL_foot_joint_3 ||
        joint_idx==MagnetoDoF::AR_foot_joint_3 ||
        joint_idx==MagnetoDoF::BL_foot_joint_3 ||
        joint_idx==MagnetoDoF::BR_foot_joint_3)
      return true;
    break;
  }
  return false;
}