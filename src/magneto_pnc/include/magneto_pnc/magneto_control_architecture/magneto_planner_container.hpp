#pragma once

#include <magneto_pnc/magneto_planner/magneto_planner_set.hpp>
#include <magneto_pnc/magneto_planner/reference_generator/reference_generator_set.hpp>

class RobotSystem;
class MagnetoControlSpecContainer;
class MotionCommand;


// Object which publicly contains all the tasks, contacts and reaction forces
class MagnetoReferenceGeneratorContainer {
 public:
  MagnetoReferenceGeneratorContainer(MagnetoControlSpecContainer* _ws_container, RobotSystem* _robot);
  ~MagnetoReferenceGeneratorContainer();

  RobotSystem* robot_;
  MagnetoControlSpecContainer* ws_container_;

  // Trajectory Managers
  FootPosTrajectoryManager* foot_trajectory_manager_;
  CoMTrajectoryManager* com_trajectory_manager_;
  JointTrajectoryManager* joint_trajectory_manager_;
  BaseOriTrajectoryManager* base_ori_trajectory_manager_;

  // QP weight / max force transition manager
  SmoothTransitionManager* max_normal_force_manager_;
  SmoothTransitionManager* W_xddot_manager_;
  SmoothTransitionManager* W_rf_manager_;

  // Planner
  MagnetoGoalPlanner* goal_planner_;
  MagnetoCoMPlanner* com_sequence_planner_;
  // MagnetoTrajectoryManager* trajectory_planner_;

};