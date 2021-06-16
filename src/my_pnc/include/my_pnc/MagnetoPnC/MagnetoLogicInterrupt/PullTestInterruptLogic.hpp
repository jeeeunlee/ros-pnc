#pragma once

#include <my_pnc/InterruptLogic.hpp>

// Forward Declare Control Architecture
class MagnetoControlArchitecture;
class MagnetoStateProvider;
class MotionCommand;

class PullTestInterruptLogic : public InterruptLogic {
 public:
  PullTestInterruptLogic(MagnetoControlArchitecture* ctrl_arch_);
  ~PullTestInterruptLogic();

  void processInterrupts();

  MagnetoControlArchitecture* ctrl_arch_;
  MagnetoStateProvider* sp_;

  std::deque<MotionCommand> motion_command_script_list_;
  MOTION_DATA motion_data_default_;

  MotionCommand* motion_command_instant_;

};
