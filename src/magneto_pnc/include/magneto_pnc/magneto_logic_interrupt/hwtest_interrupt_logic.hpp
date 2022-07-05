#pragma once

#include <pnc_core/interrupt_logic.hpp>
#include <pnc_core/control_architecture.hpp>

// Forward Declare 
class MagnetoStateProvider;
class MotionCommand;
class MagnetoUserStateCommand;

class HWTestInterruptLogic : public InterruptLogic {
 public:
  HWTestInterruptLogic(ControlArchitecture* ctrl_arch_);
  ~HWTestInterruptLogic();

  void processInterrupts();
  void setInterruptRoutine(const YAML::Node& motion_cfg);

  ControlArchitecture* ctrl_arch_;
  MagnetoStateProvider* sp_;

  MagnetoUserStateCommand* user_state_cmd_;
  std::deque<MotionCommand> script_user_cmd_deque_;

private:
  void addStateCommand(int _state_id, const MotionCommand& _smc);
};
