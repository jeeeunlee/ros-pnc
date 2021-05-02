#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateMachine/OneStepWalking.hpp>

OneStepWalking::OneStepWalking(const StateIdentifier state_identifier_in,
    MagnetoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  my_utils::pretty_constructor(2, "StateMachine: OneStepWalking");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((MagnetoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);
}

OneStepWalking::~OneStepWalking() {}

void OneStepWalking::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[One Step Walking] Start" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  // -- set current motion param
  MotionCommand mc_curr_ = ctrl_arch_->get_motion_command();
  std::cout << ctrl_arch_->get_num_states() 
            << "states left!" <<std::endl;

  moving_foot_idx_ = mc_curr_.get_moving_foot();



  // ---------------------------------------
  //      Planning
  // ---------------------------------------
  Eigen::VectorXd q_goal; 
  // ctrl_arch_->goal_planner_->computeGoal(mc_curr_);  
  // ctrl_arch_->goal_planner_->getGoalConfiguration(q_goal);

  // ctrl_arch_->reachability_planner_->setMovingFoot(mc_curr_.get_moving_foot());
  // ctrl_arch_->reachability_planner_->compute(q_goal); 
  // ctrl_arch_->reachability_planner_->addGraph

  std::vector<ReachabilityState> state_list;
  if(ctrl_arch_->trajectory_planner_->ParameterizeTrajectory(mc_curr_, 0.5, motion_periods_(0), motion_periods_(1), motion_periods_(2))){    
    double t;
    Eigen::VectorXd q, dotq, ddotq;
    bool is_swing;
    ReachabilityState rchstate;
    int t_end = (int) ((motion_periods_(0)+ motion_periods_(1)+ motion_periods_(2)) / 0.001) + 1;
    for(int i=0; i<t_end; ++i){
      t = (double)i * 0.001;
      ctrl_arch_->trajectory_planner_->update(t, q, dotq, ddotq, is_swing);
      // my_utils::pretty_print(q, std::cout, "q");
      // my_utils::pretty_print(dotq, std::cout, "dotq");
      // my_utils::saveVector(q,"KinPlanner_q");
      // my_utils::saveVector(dotq,"KinPlanner_dotq");
      // my_utils::saveVector(ddotq,"KinPlanner_ddotq");
      rchstate.q = q;
      rchstate.dq = dotq;
      rchstate.ddq = ddotq;
      rchstate.is_swing = is_swing;

      state_list.push_back(rchstate);
    }
    ctrl_arch_->reachability_planner_->setMovingFoot(mc_curr_.get_moving_foot());
    ctrl_arch_->reachability_planner_->addGraph(state_list);

    ctrl_arch_->reachability_planner_->getOptimalTraj(walking_traj_ref_);
  }
}

void OneStepWalking::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  curr_ref_ = walking_traj_ref_.front();
  walking_traj_ref_.pop_front();

  if(curr_ref_.is_swing ){
    ctrl_arch_->taf_container_->set_magnetism(moving_foot_idx_);    
    ctrl_arch_->taf_container_->set_residual_magnetic_force(moving_foot_idx_);
    ctrl_arch_->taf_container_->set_contact_magnetic_force(moving_foot_idx_);
    ctrl_arch_->taf_container_->w_res_ = 1.0;
  }  else{
    ctrl_arch_->taf_container_->set_magnetism(-1);  
    ctrl_arch_->taf_container_->set_residual_magnetic_force(-1);
    ctrl_arch_->taf_container_->set_contact_magnetic_force(-1);
    ctrl_arch_->taf_container_->w_res_ = 0.0;
  }

}

void OneStepWalking::lastVisit() {}

bool OneStepWalking::endOfState() {
  // Also check if footstep list is non-zero
  if ( walking_traj_ref_.size() == 0) { // add contact in advance case
    std::cout << "[One Step Walking] End" << std:: endl;
    my_utils::pretty_print(curr_ref_.q, std::cout, "curr_q_des");
    Eigen::VectorXd q_curr = robot_->getQ();
    my_utils::pretty_print(q_curr, std::cout, "curr_q_act");
    //exit(0);
    return true;
  }
  return false;
}

StateIdentifier OneStepWalking::getNextState() {
  return MAGNETO_STATES::BALANCE;
}

void OneStepWalking::initialization(const YAML::Node& node) {
    try {
    my_utils::readParameter(node, "motion_periods", motion_periods_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }  
}

void OneStepWalking::getCommand(void* _cmd){

  // my_utils::pretty_print(curr_ref_.trq, std::cout, "trq_cmd");
  // my_utils::pretty_print(curr_ref_.q, std::cout, "q_cmd");
  // my_utils::pretty_print(curr_ref_.dq, std::cout, "dq_cmd");

  ((MagnetoCommand*)_cmd)->jtrq = sp_->getActiveJointValue(curr_ref_.trq);
  ((MagnetoCommand*)_cmd)->q = sp_->getActiveJointValue(curr_ref_.q);
  ((MagnetoCommand*)_cmd)->qdot = sp_->getActiveJointValue(curr_ref_.dq); 


  ((MagnetoCommand*)_cmd)->b_magnetism_map = taf_container_->b_magnetism_map_;
}