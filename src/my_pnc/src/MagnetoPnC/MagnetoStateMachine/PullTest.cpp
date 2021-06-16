#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateMachine/PullTest.hpp>

PullTest::PullTest(const StateIdentifier state_identifier_in,
    MagnetoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  my_utils::pretty_constructor(2, "StateMachine: Pull Test ");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((MagnetoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);
}

PullTest::~PullTest() {}

void PullTest::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[Pull Test ] Start" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  // -- set current motion param
  MotionCommand mc_curr_ = ctrl_arch_->get_motion_command();
  std::cout << ctrl_arch_->get_num_states() 
            << "states left!" <<std::endl;

  // ---------------------------------------
  //   Planning - Find minimum contact force
  // ---------------------------------------
  // int foot_idx = mc_curr_->get_moving_foot;
  int foot_idx = MagnetoBodyNode::AL_foot_link;
  // std::cout <<"IM here 1" << std::endl;
  ctrl_arch_->contact_planner_->setNewContact(foot_idx);
  // std::cout <<"IM here 2" << std::endl;
  std::map<int, double> f_pull_max = { {MagnetoBodyNode::BL_foot_link, -30.},
                                       {MagnetoBodyNode::BR_foot_link, -30.},
                                       {MagnetoBodyNode::AR_foot_link, -30.} };
  // std::cout <<"IM here 3" << std::endl;
  std::map<int, double>  friction_coeff = { {MagnetoBodyNode::AL_foot_link, 0.5},
                                            {MagnetoBodyNode::BL_foot_link, 0.5},
                                            {MagnetoBodyNode::BR_foot_link, 0.5},
                                            {MagnetoBodyNode::AR_foot_link, 0.5} }; 
  // std::cout <<"IM here 4" << std::endl;
  ctrl_arch_->contact_planner_->setContactInfo(f_pull_max,
                                                friction_coeff);
  std::cout <<"IM here 4" << std::endl;
  // CASE 1
  // ctrl_arch_->contact_planner_->findMinForcePullOnly(0);
  // ctrl_arch_->contact_planner_->findMinForcePullOnly(1);
  // ctrl_arch_->contact_planner_->findMinForcePullOnly(2);

  std::cout <<"IM here 5" << std::endl;
  // CASE 2
  ctrl_arch_->contact_planner_->findMinForceAll(0);
  ctrl_arch_->contact_planner_->findMinForceAll(1);
  ctrl_arch_->contact_planner_->findMinForceAll(2);

  std::cout <<"IM here 6" << std::endl;

  
  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------

  // --moving foot setting
  // _set_moving_foot_frame();

  // --set com traj
  ctrl_arch_->com_trajectory_manager_
            ->setCoMTrajectory(ctrl_start_time_, &mc_curr_);
  ctrl_duration_ = ctrl_arch_->com_trajectory_manager_->getTrajDuration();
  ctrl_end_time_ = ctrl_arch_->com_trajectory_manager_->getTrajEndTime();
  // -- set base ori traj
  ctrl_arch_->base_ori_trajectory_manager_
            ->setBaseOriTrajectory(ctrl_start_time_, ctrl_duration_);

  // -- set joint traj
  ctrl_arch_->joint_trajectory_manager_
            ->setJointTrajectory(ctrl_start_time_,
                                ctrl_duration_);
 
  // -- set task_list in taf with hierachy
  ctrl_arch_->taf_container_->clear_task_list();
  ctrl_arch_->taf_container_->add_task_list(ctrl_arch_->taf_container_->com_task_);
  ctrl_arch_->taf_container_->add_task_list(ctrl_arch_->taf_container_->base_ori_task_);
  ctrl_arch_->taf_container_->add_task_list(ctrl_arch_->taf_container_->joint_task_);


  // ---------------------------------------
  //      QP PARAM - SET MAGNETISM
  // ---------------------------------------
  // todo later : implement it with magnetic manager
  // simulation/real environment magnetism
  ctrl_arch_->taf_container_->set_magnetism(-1);  
  ctrl_arch_->taf_container_->set_residual_magnetic_force(-1);
  ctrl_arch_->taf_container_->set_contact_magnetic_force(-1);
  ctrl_arch_->taf_container_->w_res_ = 0.0;


  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ctrl_arch_->taf_container_->set_contact_list(-1);

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  
  // ctrl_arch_->max_normal_force_manager_->
  // ctrl_arch_->QPweight_qddot_manager_->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )
  // ctrl_arch_->QPweight_xddot_manager_
  //           ->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )
  // ctrl_arch_->QPweight_reactforce_manager_
  //           ->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )

  ctrl_arch_->taf_container_->set_maxfz_contact(-1);
  // ctrl_arch_->taf_container_->W_qddot_ : will be always same  
  ctrl_arch_->taf_container_->compute_weight_param(-1, 
                      ctrl_arch_->taf_container_->W_xddot_contact_,
                      ctrl_arch_->taf_container_->W_xddot_nocontact_,
                      ctrl_arch_->taf_container_->W_xddot_);
  ctrl_arch_->taf_container_->compute_weight_param(-1, 
                      ctrl_arch_->taf_container_->W_rf_contact_,
                      ctrl_arch_->taf_container_->W_rf_nocontact_,
                      ctrl_arch_->taf_container_->W_rf_);

}

void PullTest::_taskUpdate() {
  // ctrl_arch_->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  ctrl_arch_->com_trajectory_manager_->updateTask(sp_->curr_time,
                                      ctrl_arch_->taf_container_->com_task_);

  // ctrl_arch_->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  ctrl_arch_->base_ori_trajectory_manager_->updateTask(sp_->curr_time,
                                      ctrl_arch_->taf_container_->base_ori_task_);
  
  // ctrl_arch_->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  ctrl_arch_->joint_trajectory_manager_->updateTask(sp_->curr_time,
                                      ctrl_arch_->taf_container_->joint_task_);
}

void PullTest::_weightUpdate() {
  // no change in weight
}

void PullTest::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void PullTest::lastVisit() {}

bool PullTest::endOfState() {
  // Also check if footstep list is non-zero
  if ( state_machine_time_ > ctrl_duration_ && ctrl_arch_->get_num_states() > 0) {
    std::cout << "[Pull Test ] End" << std:: endl;
    return true;
  }
  return false;
}

StateIdentifier PullTest::getNextState() {
  return MAGNETO_STATES::SWING_START_TRANS;
}

void PullTest::initialization(const YAML::Node& node) {}
