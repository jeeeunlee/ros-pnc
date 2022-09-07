#pragma once

#include <my_pnc/InterruptLogic.hpp>

// Forward Declare Control Architecture
class MagnetoControlArchitecture;
class MagnetoStateProvider;
class MotionCommand;

class WalkingInterruptLogic : public InterruptLogic {
 public:
  WalkingInterruptLogic(MagnetoControlArchitecture* ctrl_arch_);
  ~WalkingInterruptLogic();

  void processInterrupts();
  void setInterruptRoutine(const YAML::Node& motion_cfg);

  MagnetoControlArchitecture* ctrl_arch_;
  MagnetoStateProvider* sp_;

  std::deque<MotionCommand> motion_command_script_list_;
  MOTION_DATA motion_data_default_;
  MotionCommand* motion_command_instant_;

};