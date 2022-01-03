
#include <../my_utils/Configuration.h>

#include <my_robot_system/RobotSystem.hpp>

#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp>

#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoPlanner/MagnetoReachabilityPlanner.hpp>

#include <my_utils/IO/IOUtilities.hpp>
#include "my_utils/Math/pseudo_inverse.hpp"



MagnetoReachabilityNode::MagnetoReachabilityNode(MagnetoReachabilityContact* contact,
                                            const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& dotq,
                                            const bool& is_swing) {
  contact_state_ = contact;
  q_ = q;
  dotq_ = dotq;  
  is_swing_ = is_swing;
}

MagnetoReachabilityNode::~MagnetoReachabilityNode() {}

bool MagnetoReachabilityNode::computeTorque(const Eigen::VectorXd& ddq_des,
                                          Eigen::VectorXd& ddq_plan,
                                          Eigen::VectorXd& tau){                                             
  contact_state_->update(q_, dotq_, ddq_des);
  return contact_state_->solveContactDyn(tau, ddq_plan);
}

bool MagnetoReachabilityNode::computeDdotq(Eigen::VectorXd& tau,
                                          Eigen::VectorXd& ddq){
  return contact_state_->computeDdotq(tau,ddq);
}

// void MagnetoReachabilityNode::computeNextState() {
//   contact_state_->computeNextState(tau, q_next, dotq_next);
// }

MagnetoReachabilityEdge::MagnetoReachabilityEdge(MagnetoReachabilityNode* src_node,
                              MagnetoReachabilityNode* dst_node,
                              const Eigen::VectorXd& trq_atv) {
      src_node_ = src_node; // source
      dst_node_ = dst_node; // destination
      trq_atv_ = trq_atv;
}

MagnetoReachabilityEdge::~MagnetoReachabilityEdge() {}

void MagnetoReachabilityEdge::getReachabilityState(ReachabilityState &rcstate){
  rcstate.trq = this->trq_atv_; 
  
  rcstate.q = this->src_node_->q_;
  rcstate.dq = this->src_node_->dotq_;  
  rcstate.is_swing = this->src_node_->is_swing_;
}

MagnetoReachabilityContact::MagnetoReachabilityContact(RobotSystem* robot_planner)  {
  // robot system
  robot_planner_ = robot_planner;
  is_update_centroid_ = false;

  // dimension
  dim_joint_ = Magneto::n_dof;
  dim_contact_ = 0;

  Sa_ = Eigen::MatrixXd::Zero(Magneto::n_adof, Magneto::n_dof);
  Sv_ = Eigen::MatrixXd::Zero(Magneto::n_vdof, Magneto::n_dof);
  for(int i(0); i<Magneto::n_adof; ++i)
    Sa_(i, Magneto::idx_adof[i]) = 1.;
  for(int i(0); i<Magneto::n_vdof; ++i)
    Sv_(i, Magneto::idx_vdof[i]) = 1.;

  // dyn solver
  wbqpd_param_ = new WbqpdParam();
  wbqpd_result_ = new WbqpdResult();
  wbqpd_ = new WBQPD(Sa_, Sv_); 
}

MagnetoReachabilityContact::~MagnetoReachabilityContact() {
  _deleteContacts();
}

void MagnetoReachabilityContact::initialization(const YAML::Node& node) {
  // torque limit, magnetic force
  double torque_limit(0.);
  try {
    my_utils::readParameter(node["controller_params"], 
                          "torque_limit", torque_limit);

    my_utils::readParameter(node["magnetism_params"], 
                          "magnetic_force", magnetic_force_);  
    my_utils::readParameter(node["magnetism_params"], 
                          "residual_ratio", residual_ratio_);
   
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  Eigen::VectorXd tau_min =
      // sp_->getActiveJointValue(robot_->GetTorqueLowerLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, -torque_limit); // -5.
  Eigen::VectorXd tau_max =
      // sp_->getActiveJointValue(robot_->GetTorqueUpperLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, torque_limit); // 5.

  wbqpd_->setTorqueLimit(tau_min, tau_max);
} 

void MagnetoReachabilityContact::_deleteContacts() {
  for(auto &contact : contact_list_)
    delete contact;  
  contact_list_.clear();
}

void MagnetoReachabilityContact::clearContacts() {
  contact_list_.clear();
  dim_contact_=0;
}

void MagnetoReachabilityContact::addContacts(ContactSpec* contact) {
  contact_list_.push_back(contact);
  dim_contact_ += contact->getDim();
}

void MagnetoReachabilityContact::FinishContactSet() {
  // set parameters dependent on contact list
  Eigen::VectorXd F_magnetic;
  Eigen::MatrixXd Uf_i, Uf;
  Eigen::VectorXd Fr_ieq_i, Fr_ieq, u0;

  // 1. magnetic force vector && dimension check
  F_magnetic = Eigen::VectorXd::Zero(dim_contact_);
  int contact_idx(0), fz_idx;  
  int Uf_row_dim(0), Uf_col_dim(0);
  std::cout<<" ----------------- FinishContactSet : ";
  for(auto &it : contact_list_) {
    std::cout<< ((BodyFramePointContactSpec*)(it))->getLinkIdx() << ", " ;
    // magnetic force
    fz_idx = contact_idx + ((BodyFramePointContactSpec*)(it))->getFzIndex();    
    F_magnetic[fz_idx] = magnetic_force_;
    contact_idx += ((BodyFramePointContactSpec*)(it))->getDim();
    // contact friction cone dimension check
    it->getRFConstraintMtx(Uf_i);
    Uf_row_dim+= Uf_i.rows();
    Uf_col_dim+= Uf_i.cols();
  }
  std::cout << " ----------------- "<< std::endl;
  // std::cout << "magnetic_force_ = " << magnetic_force_ << std::endl;

  // 2. contact friction cone
  Uf = Eigen::MatrixXd::Zero(Uf_row_dim, Uf_col_dim);
  Fr_ieq = Eigen::VectorXd::Zero(Uf_row_dim);
  Uf_row_dim = 0; Uf_col_dim = 0;
  for(auto &contact : contact_list_) {
    contact->getRFConstraintMtx(Uf_i);
    contact->getRFConstraintVec(Fr_ieq_i);
    Uf.block(Uf_row_dim, Uf_col_dim, Uf_i.rows(), Uf_i.cols()) = Uf_i;
    Fr_ieq.segment(Uf_row_dim, Uf_i.rows()) = Fr_ieq_i;
    Uf_row_dim+= Uf_i.rows();
    Uf_col_dim+= Uf_i.cols();
  }
  // my_utils::pretty_print(Uf, std::cout, "Uf");
  // my_utils::pretty_print(Fr_ieq, std::cout, "Fr_ieq");
  Fr_ieq -= Uf*F_magnetic; // todo : sign check
  // my_utils::pretty_print(Fr_ieq, std::cout, "Fr_ieq_mag");

  wbqpd_->setFrictionCone(Uf, Fr_ieq);
}

/*
ddq = MInv_*(Nc_T_*Sa_T*tau - NC_T(b+g) -Jc_T_*AMat_*Jcdotqdot_)
    = A*tau + a0
Fc = -AMat_*Jcdotqdot_+Jc_bar_T_(b+g) - Jc_bar_T_*Sa_T*tau
    = B*tau + b0

A = MInv_*Nc_T_(*Sa_T)
a0 = MInv_*(- NC_T(b+g) -Jc_T_*AMat_*Jcdotqdot_)
B = -Jc_bar_T_(*Sa_T)
b0 = -AMat_*Jcdotqdot_+Jc_bar_T_(b+g)
*/

void MagnetoReachabilityContact::update(const Eigen::VectorXd& q,
                                        const Eigen::VectorXd& dotq,
                                        const Eigen::VectorXd& ddotq) {

  robot_planner_->updateSystem(q, dotq, is_update_centroid_);

  _updateContacts(q, dotq);
  _buildContactJacobian(Jc_);
  _buildContactJcDotQdot(Jcdotqdot_); // Jdotqdot

  // update 
  M_ = robot_planner_->getMassMatrix();
  MInv_ = robot_planner_->getInvMassMatrix();
  cori_grav_ = robot_planner_->getCoriolisGravity();
  AInv_ = Jc_ * MInv_ * Jc_.transpose();
  AMat_ = getPseudoInverse(AInv_);
  Jc_bar_T_ = AMat_ * Jc_* MInv_; // = Jc_bar.transpose() 

  Q_ = getNullSpaceMatrix(Jc_bar_T_); 
  Nc_T_ = Eigen::MatrixXd::Identity(dim_joint_,dim_joint_) - Jc_.transpose()*Jc_bar_T_;

  wbqpd_param_->A = MInv_*Nc_T_;
  wbqpd_param_->a0 = MInv_*(- Nc_T_*cori_grav_ - Jc_.transpose()*AMat_*Jcdotqdot_);
  wbqpd_param_->B = - Jc_bar_T_;
  wbqpd_param_->b0 = - AMat_*Jcdotqdot_ + Jc_bar_T_*cori_grav_;
  
  wbqpd_param_->ddq_des = ddotq;
  wbqpd_param_->Wf = Eigen::VectorXd::Constant(dim_contact_, 1.0);
  // wbqpd_param_->Wq = Eigen::VectorXd::Constant(dim_joint_, 100.);
  Eigen::MatrixXd Saa = Sa_.transpose()*Sa_ ;
  Eigen::MatrixXd Svv = Sv_.transpose()*Sv_ ;
  Eigen::MatrixXd Sbase = Eigen::MatrixXd::Zero(Magneto::n_dof, Magneto::n_dof);
  Sbase.block(0,0,3,3) = 0.0001*Eigen::Matrix3d::Identity(); // pos 1mm
  Sbase.block(3,3,3,3) = Eigen::Matrix3d::Identity(); // ori 5degree
  wbqpd_param_->Wq = 100. * Saa.diagonal() + 0.00001*Svv.diagonal() + 10000.*Sbase.diagonal(); 
  
  int fz_idx(0), contact_idx(0);
  for(auto &it : contact_list_) {
    fz_idx = contact_idx + ((BodyFramePointContactSpec*)(it))->getFzIndex();  
    wbqpd_param_->Wf[fz_idx] = 0.01; // Fz no cost
    contact_idx += it->getDim();
  }
  wbqpd_->updateSetting(wbqpd_param_);
}

bool MagnetoReachabilityContact::solveContactDyn(Eigen::VectorXd& tau, 
                                                Eigen::VectorXd& ddq_plan) {
  double f = wbqpd_->computeTorque(wbqpd_result_);
  tau = wbqpd_result_->tau;
  ddq_plan = wbqpd_result_->ddq;
  bool b_reachable = wbqpd_result_->b_reachable;

  return b_reachable;
}

bool MagnetoReachabilityContact::computeDdotq(Eigen::VectorXd& tau,
                                              Eigen::VectorXd& ddq) {
  return wbqpd_->computeDdotq(tau, ddq);
}

void MagnetoReachabilityContact::computeNextState(const Eigen::VectorXd& tau,
                                                Eigen::VectorXd& q_next,
                                                Eigen::VectorXd& dotq_next) {
  // double timestep = 0.01;
  // bool b_feasible = wbqpd_->computeDdotq(tau, ddotq);
  // q_next = q_ + dotq_*timestep;
  // dotq_next = dotq_ + ddotq*timestep;
}


void MagnetoReachabilityContact::_updateContacts(const Eigen::VectorXd& q,
                                                const Eigen::VectorXd& dotq) {  
  for(auto &contact : contact_list_) {
      contact->updateContactSpec();
  }
}

void MagnetoReachabilityContact::_buildContactJacobian(Eigen::MatrixXd& Jc) {
  // initialize Jc
  Jc = Eigen::MatrixXd::Zero(dim_contact_, dim_joint_);

  // vercat Jc
  Eigen::MatrixXd Ji;
  int dim_Ji(0), dim_J(0);
  for(auto &contact : contact_list_) {
    contact->getContactJacobian(Ji);
    dim_Ji = contact->getDim();
    Jc.block(dim_J, 0, dim_Ji, dim_joint_) = Ji;
    dim_J += dim_Ji;
  }
}

void MagnetoReachabilityContact::_buildContactJcDotQdot(
                                  Eigen::VectorXd&  Jcdotqdot) {
  Jcdotqdot = Eigen::VectorXd::Zero(dim_contact_, dim_joint_);
  // vercat Jcdotqdot
  Eigen::VectorXd Ji;
  int dim_Ji(0), dim_J(0);

  for(auto &contact : contact_list_) {
    contact->getJcDotQdot(Ji);
    dim_Ji = contact->getDim();
    Jcdotqdot.segment(dim_J, dim_Ji) = Ji;
    dim_J += dim_Ji;
  }
}

Eigen::MatrixXd MagnetoReachabilityContact::getPseudoInverse(const Eigen::MatrixXd& Mat) {
  Eigen::MatrixXd MatInv;
  my_utils::pseudoInverse(Mat, 0.0001, MatInv);
  return MatInv;
}


Eigen::MatrixXd MagnetoReachabilityContact::getNullSpaceMatrix(const Eigen::MatrixXd& A) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeFullV);
  int nrows(svd.singularValues().rows());
  int Vrows(svd.matrixV().rows());
  int Vcols(svd.matrixV().cols());

  Eigen::MatrixXd NullA = svd.matrixV().block(0, nrows, Vrows, Vcols-nrows);

  // if there are more singular value
  double const tol = 1e-5;
  for(int ii=0; ii<nrows; ++ii) {
    if(svd.singularValues().coeff(ii) < tol) {
      NullA = svd.matrixV().block(0, ii, Vrows, Vcols-ii);
      break;
    }
  }
  return NullA;
}

MagnetoReachabilityPlanner::MagnetoReachabilityPlanner(RobotSystem* robot, MagnetoControlArchitecture* _ctrl_arch) {
  my_utils::pretty_constructor(2, "Magneto Reachablity Planner");
  // robot system
  robot_ = robot;    
  // copy constructor
  robot_planner_ = new RobotSystem(*robot_);

  ctrl_arch_ = ((MagnetoControlArchitecture*)_ctrl_arch);

  // Set virtual & actuated selection matrix
  Sa_ = Eigen::MatrixXd::Zero(Magneto::n_adof, Magneto::n_dof);
  Sv_ = Eigen::MatrixXd::Zero(Magneto::n_vdof, Magneto::n_dof);
  
  for(int i(0); i<Magneto::n_adof; ++i)
    Sa_(i, Magneto::idx_adof[i]) = 1.;
  for(int i(0); i<Magneto::n_vdof; ++i)
    Sv_(i, Magneto::idx_vdof[i]) = 1.;
  
  // etc
  q_zero_ = Eigen::VectorXd::Zero(Magneto::n_dof);
  is_update_centroid_ = false; // used when to update robotsystem

  // set contact
  mu_ = 0.7; // will be updated later
  // alfoot_contact_ = new BodyFramePointContactSpec(robot_planner_,
  //                             MagnetoBodyNode::AL_foot_link, mu_);
  // blfoot_contact_ = new BodyFramePointContactSpec(robot_planner_,
  //                             MagnetoBodyNode::BL_foot_link, mu_);                          
  // arfoot_contact_ = new BodyFramePointContactSpec(robot_planner_,
  //                             MagnetoBodyNode::AR_foot_link, mu_);
  // brfoot_contact_ = new BodyFramePointContactSpec(robot_planner_,
  //                             MagnetoBodyNode::BR_foot_link, mu_);

  double footx(0.02), footy(0.02);
  alfoot_contact_ = new  BodyFrameSurfaceContactSpec(robot_planner_,
                      MagnetoBodyNode::AL_foot_link, footx, footy, mu_);
  blfoot_contact_ = new BodyFrameSurfaceContactSpec(robot_planner_,
                      MagnetoBodyNode::BL_foot_link, footx, footy, mu_);                          
  arfoot_contact_ = new BodyFrameSurfaceContactSpec(robot_planner_,
                      MagnetoBodyNode::AR_foot_link, footx, footy, mu_);
  brfoot_contact_ = new BodyFrameSurfaceContactSpec(robot_planner_,
                      MagnetoBodyNode::BR_foot_link, footx, footy, mu_);

  full_contact_list_.clear();
  full_contact_list_.push_back(alfoot_contact_);  
  full_contact_list_.push_back(arfoot_contact_);
  full_contact_list_.push_back(blfoot_contact_);
  full_contact_list_.push_back(brfoot_contact_);

  swing_contact_state_ = new MagnetoReachabilityContact(robot_planner_);
  full_contact_state_ = new MagnetoReachabilityContact(robot_planner_);
  
  full_contact_state_->clearContacts();
  for(auto &contact : full_contact_list_)
    full_contact_state_->addContacts(contact); 
}


MagnetoReachabilityPlanner::~MagnetoReachabilityPlanner() {
  full_contact_list_.clear();
  delete alfoot_contact_;
  delete blfoot_contact_;
  delete arfoot_contact_;
  delete brfoot_contact_;

  delete robot_planner_;
  delete full_contact_state_;
  delete swing_contact_state_;
}

void MagnetoReachabilityPlanner::initialization(const YAML::Node& node){

  try {
    my_utils::readParameter(node["contact_params"], "friction", mu_);   
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  for(auto &contact : full_contact_list_)
    ((BodyFramePointContactSpec*)contact)->setFrictionCoeff(mu_);


  full_contact_state_->initialization(node);
  full_contact_state_->FinishContactSet();

  swing_contact_state_->initialization(node);
  moving_foot_idx_ = -1;
} 

void MagnetoReachabilityPlanner::setMovingFoot(int moving_foot) {
  moving_foot_idx_ = moving_foot;
  // set contact list
  swing_contact_state_->clearContacts();
  for(auto &contact : full_contact_list_){
    if( ((BodyFramePointContactSpec*)contact)->getLinkIdx() != moving_foot_idx_ )
      swing_contact_state_->addContacts(contact);
  }
  std::cout << " setMovingFoor " <<  std::endl;
  swing_contact_state_->FinishContactSet();
}

void MagnetoReachabilityPlanner::compute(const Eigen::VectorXd& q_goal) {
  _setInitGoal(q_goal, q_zero_); // assume zero velocity goal

  q_ = q_init_;
  dotq_ = dotq_init_;
  // full contact node
  MagnetoReachabilityNode* node_fc_init =
                new MagnetoReachabilityNode(full_contact_state_, 
                                        q_init_, dotq_init_,false);
  //  
  Eigen::VectorXd tau, ddq;
  node_fc_init->computeTorque(q_zero_, ddq, tau);
  // 
  // swing contact node
  MagnetoReachabilityNode* node_sc_init = 
              new MagnetoReachabilityNode(swing_contact_state_, 
                                        q_init_, dotq_init_,true);
}


void MagnetoReachabilityPlanner::addGraph(const std::vector<ReachabilityState> &state_list){
  // state = (q, dq, ddq, is_swing)

  // DATA CHECK FOR TORQUE
  Eigen::VectorXd q_accum, dq_accum, tau_fb;
  // DATA CHECK FOR TORQUE

  initializeVariables();
  // method 1 : generate all nodes in trajectory -> check edge
  Eigen::VectorXd tau, tau_prev;
  tau_prev = q_zero_;
  ReachabilityState prev_state;
  for( auto &state : state_list ){
    // DATA CHECK FOR TORQUE
    if(node_list_.empty()){
      q_accum = state.q;
      dq_accum = state.dq;
      tau_fb = q_zero_;
    }
    // DATA CHECK FOR TORQUE

    MagnetoReachabilityNode* node;    
    if(state.is_swing){
      // std::cout << "is swing" << state.is_swing << std::endl;
      node = new MagnetoReachabilityNode(swing_contact_state_, state.q, state.dq, state.is_swing);
    } else{
      // std::cout << "is full" << state.is_swing << std::endl;
      node = new MagnetoReachabilityNode(full_contact_state_, state.q, state.dq, state.is_swing);
    }
      
    
    // check edge with the previous node
    bool b_feasible;
    Eigen::VectorXd ddq, ddq_des;
    if(!node_list_.empty()) {
      MagnetoReachabilityNode* prev_node = node_list_.back();
      ddq_des = (state.dq-prev_state.dq)/0.001; //prev_state.ddq; // 
      b_feasible = prev_node->computeTorque(ddq_des, ddq, tau);
      
      // DATA CHECK FOR TORQUE
      tau_fb = q_zero_; 
      for(int i=0; i< Magneto::n_adof; ++i) {
        tau_fb[Magneto::idx_adof[i]] 
            = tau[Magneto::idx_adof[i]] + 
              2.0 * (prev_state.dq[Magneto::idx_adof[i]] - dq_accum[Magneto::idx_adof[i]]) +
              30.0 * (prev_state.q[Magneto::idx_adof[i]] - q_accum[Magneto::idx_adof[i]]);
      }


      // my_utils::pretty_print(tau, std::cout, "tau");
      // my_utils::pretty_print(tau_fb, std::cout, "tau_fb");

      // my_utils::pretty_print(dq_accum, std::cout, "dq_accum");
      // my_utils::pretty_print(prev_state.dq, std::cout, "prev_state.dq");

      // my_utils::pretty_print(q_accum, std::cout, "q_accum");
      // my_utils::pretty_print(prev_state.q, std::cout, "prev_state.q");

      // my_utils::saveVector(ddq_des, "RPlanner_ddq_des");
      // my_utils::saveVector(ddq, "RPlanner_ddq");
      prev_node->computeDdotq(tau_fb, ddq);
      // my_utils::saveVector(ddq, "RPlanner_ddq_fb");

      // my_utils::saveVector(state.q, "RPlanner_q");
      // my_utils::saveVector(q_accum, "RPlanner_q_accum");
      // my_utils::saveVector(state.dq, "RPlanner_dq");
      // my_utils::saveVector(dq_accum, "RPlanner_dq_accum");

      q_accum += dq_accum * 0.001;
      dq_accum += ddq * 0.001;
      

      // DATA CHECK FOR TORQUE

      // my_utils::saveVector(ddq_des, "RPlanner_ddq_des");
      // my_utils::saveVector(ddq, "RPlanner_ddq_act");
      // my_utils::saveVector(tau, "RPlanner_torque");
      
      if(b_feasible) {
        // my_utils::pretty_print(tau, std::cout, "tau");
        MagnetoReachabilityEdge* edge = new MagnetoReachabilityEdge(prev_node, node, tau);
        //edge_list_.push_back(edge);
        edge_path_.push_back(edge);
      }
      else{
        std::cout << "feasible dyn? : " << b_feasible <<" / " << std::endl;
        MagnetoReachabilityEdge* edge = new MagnetoReachabilityEdge(prev_node, node, tau_prev);
        edge_path_.push_back(edge);
      }
      tau_prev = tau;
    }    
    node_list_.push_back(node);
    prev_state = state;
  }

  // method 2 : node -> find edge -> next node
  
}

void MagnetoReachabilityPlanner::getOptimalTraj(std::deque<ReachabilityState> &traj){
  traj.clear();
  ReachabilityState rcState;

  for(auto edge : edge_path_){
    edge->getReachabilityState(rcState);    
    traj.push_back(rcState);
  }
}

void MagnetoReachabilityPlanner::initializeVariables(){
  node_list_.clear();
  edge_list_.clear();
  edge_path_.clear();
}


void MagnetoReachabilityPlanner::_setInitGoal(const Eigen::VectorXd& q_goal,
                                              const Eigen::VectorXd& qdot_goal) {
  q_goal_ = q_goal;
  dotq_goal_ = qdot_goal;
  q_init_ = robot_->getQ();
  dotq_init_ = robot_->getQdot();
  node_list_.clear();
  edge_list_.clear();
}

