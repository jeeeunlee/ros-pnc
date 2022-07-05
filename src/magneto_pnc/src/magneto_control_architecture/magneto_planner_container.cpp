#include <magneto_pnc/magneto_control_architecture/magneto_planner_container.hpp>
#include <magneto_pnc/magneto_control_architecture/magneto_controlspec_container.hpp>
#include <pnc_utils/robot_system.hpp>

MagnetoReferenceGeneratorContainer::MagnetoReferenceGeneratorContainer(
  MagnetoControlSpecContainer* _ws_container, RobotSystem* _robot){
  robot_ = _robot;
  ws_container_ = _ws_container;

  // Initialize Trajectory managers
  foot_trajectory_manager_ = new FootPosTrajectoryManager(robot_);                    
  com_trajectory_manager_ = new CoMTrajectoryManager(robot_);
  joint_trajectory_manager_ = new JointTrajectoryManager(robot_);
  base_ori_trajectory_manager_ = new BaseOriTrajectoryManager(robot_, MagnetoBodyNode::base_link);

  max_normal_force_manager_ = new SmoothTransitionManager(&ws_container_->max_rf_z_trans_);
  W_xddot_manager_ = new SmoothTransitionManager(&ws_container_->w_xddot_z_trans_);
  W_rf_manager_ = new SmoothTransitionManager(&ws_container_->w_rf_z_trans_);

  goal_planner_ = new MagnetoGoalPlanner(robot_);
  com_sequence_planner_ = new MagnetoCoMPlanner(robot_);
}

MagnetoReferenceGeneratorContainer::~MagnetoReferenceGeneratorContainer(){
    delete foot_trajectory_manager_;
    delete com_trajectory_manager_;
    delete joint_trajectory_manager_;
    delete base_ori_trajectory_manager_;

    delete max_normal_force_manager_;
    delete W_xddot_manager_;
    delete W_rf_manager_;

    delete goal_planner_;
    delete com_sequence_planner_;
}