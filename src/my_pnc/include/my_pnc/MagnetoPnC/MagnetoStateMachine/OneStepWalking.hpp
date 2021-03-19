#pragma once

#include <my_utils/Math/BSplineBasic.h>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
#include <my_pnc/StateMachine.hpp>
#include <my_pnc/MagnetoPnC/MagnetoPlanner/MagnetoReachabilityPlanner.hpp>


class MagnetoControlArchitecture;
class MagnetoTaskAndForceContainer;

class OneStepWalking : public StateMachine {
 public:
  OneStepWalking(const StateIdentifier state_identifier_in,
                       MagnetoControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot);
  ~OneStepWalking();

  void getCommand(void* _cmd); // only for one step walking

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();  
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  MagnetoStateProvider* sp_;
  MagnetoControlArchitecture* ctrl_arch_;
  MagnetoTaskAndForceContainer* taf_container_;

  std::deque<ReachabilityState> walking_traj_ref_;
  ReachabilityState curr_ref_; 

  Eigen::VectorXd motion_periods_;
  double ctrl_start_time_;
  double ctrl_end_time_;
  double ctrl_duration_;

  int moving_foot_idx_; // link index

};
