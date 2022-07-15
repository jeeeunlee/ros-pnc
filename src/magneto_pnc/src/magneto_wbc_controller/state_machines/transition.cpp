#include <magneto_pnc/magneto_control_architecture/magneto_planner_container.hpp>
#include <magneto_pnc/magneto_control_architecture/magneto_controlspec_container.hpp>
#include <magneto_pnc/magneto_wbc_controller/state_machines/transition.hpp>

Transition::Transition(const StateIdentifier state_identifier_in, 
    MagnetoReferenceGeneratorContainer* rg_container,
    bool contact_start): b_contact_start_(contact_start),
    StateMachine(state_identifier_in, rg_container->robot_) {
  pnc_utils::pretty_constructor(2, "StateMachine: Transition");

  // Set Pointer to Control Architecture
  ws_container_ = rg_container->ws_container_;
  rg_container_ = rg_container;

  // Get State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);
}

Transition::~Transition() {}

void Transition::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[contact transition] Start : " << b_contact_start_ << std::endl;

  ctrl_start_time_ = sp_->curr_time;
 
  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->set_contact_list(-1);  // contain full contact

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------
  // -- set current motion param
  MotionCommand mc_curr_ = sp_->curr_motion_command;
  moving_foot_idx_ = mc_curr_.get_moving_foot();
  moving_foot_link_idx_ = MagnetoFoot::LinkIdx[moving_foot_idx_];
  std::cout << " transition !! - moving_foot_link_idx_=" << moving_foot_link_idx_
            << ", moving_foot_idx_" << moving_foot_idx_ << std::endl;

  

  // --set com traj
  Eigen::Vector3d pcom = robot_->getCoMPosition();
  pnc_utils::pretty_print(pcom, std::cout, "pc_init");
  ComMotionCommand mc_com;
  if(b_contact_start_){
    mc_com = rg_container_->
            com_sequence_planner_->getSwingEndCoMCmd();
  }else{
    mc_com = rg_container_->
            com_sequence_planner_->getSwingStartCoMCmd();
  }
  mc_com.printMotionInfo();
  rg_container_->com_trajectory_manager_
               ->setCoMTrajectory(ctrl_start_time_, mc_com);
  ctrl_duration_ = rg_container_->com_trajectory_manager_->getTrajDuration();
  ctrl_end_time_ = rg_container_->com_trajectory_manager_->getTrajEndTime();

  // -- set base ori traj
  rg_container_->base_ori_trajectory_manager_
            ->setBaseOriTrajectory(ctrl_start_time_,
                                  ctrl_duration_);
  // -- set joint traj
  rg_container_->joint_trajectory_manager_
            ->setJointTrajectory(ctrl_start_time_,
                                ctrl_duration_);

  // -- set foot traj to be stationary
  MotionCommand mc_trans = MotionCommand();
  mc_trans.foot_motion_given = true;
  POSE_DATA foot_dev = POSE_DATA();
  if(b_contact_start_) foot_dev.pos[2] -= 0.05;
  mc_trans.foot_motion_data = SWING_DATA( moving_foot_idx_, 0.0, foot_dev);
  mc_trans.motion_periods = Eigen::VectorXd::Constant(1, ctrl_duration_);
  
  rg_container_->foot_trajectory_manager_
            ->setFootPosTrajectory(ctrl_start_time_, &mc_trans);

  // -- set task_list in taf with hierachy
  ws_container_->clear_task_list();
  ws_container_->add_task_list(ws_container_->com_task_);
  ws_container_->add_task_list(ws_container_->base_ori_task_);
  ws_container_->add_task_list(
                ws_container_->
                get_foot_pos_task(moving_foot_link_idx_));
  ws_container_->add_task_list(
                ws_container_->
                get_foot_ori_task(moving_foot_link_idx_));
  ws_container_->add_task_list(ws_container_->joint_task_);

  // ---------------------------------------
  //      QP PARAM - SET MAGNETISM
  // ---------------------------------------
  // todo later : implement it with magnetic manager
  // simulation/real environment magnetism
  if(b_contact_start_){
    ws_container_->set_foot_magnet_off(-1);
    ws_container_->set_magnet_distance(-1, 0.0);   
  }  else {
    ws_container_->set_foot_magnet_off(moving_foot_link_idx_); // off-magnetism on moving foot
    ws_container_->set_magnet_distance(moving_foot_link_idx_, 0.0);
  }



  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // --------------------------------------- 
  // ws_container_->W_qddot_ : will be always same 
 
  if(b_contact_start_) {
    // moving foot : nocontact -> contact
    rg_container_->max_normal_force_manager_->setTransition(ctrl_start_time_,
                                      ctrl_duration_,
                                      ws_container_->max_rf_z_nocontact_,
                                      ws_container_->max_rf_z_contact_);
    rg_container_->W_xddot_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_xddot_z_nocontact_, 
                                      ws_container_->w_xddot_z_contact_);
    rg_container_->W_rf_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_rf_z_nocontact_, 
                                      ws_container_->w_rf_z_contact_);
  } else {
    // moving foot : contact -> nocontact
    rg_container_->max_normal_force_manager_ ->setTransition(ctrl_start_time_,
                                      ctrl_duration_,
                                      ws_container_->max_rf_z_contact_, 
                                      ws_container_->max_rf_z_nocontact_);
    rg_container_->W_xddot_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_xddot_z_contact_, 
                                      ws_container_->w_xddot_z_nocontact_);
    rg_container_->W_rf_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_rf_z_contact_, 
                                      ws_container_->w_rf_z_nocontact_);
  }
}

void Transition::_taskUpdate() {

  rg_container_->foot_trajectory_manager_->updateTask(sp_->curr_time,
              ws_container_->get_foot_pos_task(moving_foot_link_idx_),
              ws_container_->get_foot_ori_task(moving_foot_link_idx_));

  // rg_container_->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  rg_container_->com_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->com_task_);
  // rg_container_->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  rg_container_->base_ori_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->base_ori_task_);
  // rg_container_->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  rg_container_->joint_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->joint_task_);
}

void Transition::_weightUpdate() {
  // change in weight
  rg_container_->W_xddot_manager_->updateTransition(sp_->curr_time);
  rg_container_->W_rf_manager_->updateTransition(sp_->curr_time);
  // change in normal force in contactSpec
  rg_container_->max_normal_force_manager_->updateTransition(sp_->curr_time);

  ws_container_->set_contact_weight_param(moving_foot_link_idx_);
  ws_container_->set_contact_maxfz(moving_foot_link_idx_);
}


void Transition::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void Transition::lastVisit() {}

bool Transition::endOfState() {
  // Also check if footstep list is non-zero
  if(b_contact_start_){
    if ( state_machine_time_ > ctrl_duration_) {
      switch(moving_foot_idx_){
        case MagnetoFoot::AL:
        if(sp_->b_alfoot_contact){
          std::cout << "[contact transition] End : " << b_contact_start_ << std::endl;
          return true; } break;        
        case MagnetoFoot::BL:
          if(sp_->b_blfoot_contact){
          std::cout << "[contact transition] End : " << b_contact_start_ << std::endl;
          return true; } break;
        case MagnetoFoot::AR:
          if(sp_->b_arfoot_contact){
          std::cout << "[contact transition] End : " << b_contact_start_ << std::endl;
          return true; } break;
        case MagnetoFoot::BR:
          if(sp_->b_brfoot_contact){
          std::cout << "[contact transition] End : " << b_contact_start_ << std::endl;
          return true; } break;
      }
    }
  } else{
    if ( state_machine_time_ > ctrl_duration_) {
      std::cout << "[contact transition] End : " << b_contact_start_ 
                << ", state_machine_time_ > ctrl_duration" <<std::endl;
      return true;
    } else if(state_machine_time_ > 0.9*ctrl_duration_){
      switch(moving_foot_idx_){
      case MagnetoFoot::AL:
      if(!sp_->b_alfoot_contact){
        std::cout << "[contact transition] End : " << b_contact_start_ 
                  << ", " << state_machine_time_ <<" < " << ctrl_duration_ << std::endl;
        return true; } break;        
      case MagnetoFoot::BL:
        if(!sp_->b_blfoot_contact){
        std::cout << "[contact transition] End : " << b_contact_start_ 
                  << ", " << state_machine_time_ <<" < " << ctrl_duration_ << std::endl;
        return true; } break;
      case MagnetoFoot::AR:
        if(!sp_->b_arfoot_contact){
        std::cout << "[contact transition] End : " << b_contact_start_ 
                  << ", " << state_machine_time_ <<" < " << ctrl_duration_ << std::endl;
        return true; } break;
      case MagnetoFoot::BR:
        if(!sp_->b_brfoot_contact){
        std::cout << "[contact transition] End : " << b_contact_start_ 
                  << ", " << state_machine_time_ <<" < " << ctrl_duration_ << std::endl;
        return true; } break;
      }
    }    
  } 
  return false;
}

void Transition::initialization(const YAML::Node& node) {

  try {
    pnc_utils::readParameter(node, "transition_duration", trans_duration_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }  

  rg_container_->com_sequence_planner_->setTransitionDuration(trans_duration_);
  
}
