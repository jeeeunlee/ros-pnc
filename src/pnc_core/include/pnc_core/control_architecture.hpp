#pragma once

#include <map>

#include <pnc_utils/Configuration.h>
#include <pnc_utils/io_utilities.hpp>

#include <pnc_utils/robot_system.hpp>
#include <pnc_core/state_machine.hpp>

// Generic Control Architecture Object
class ControlArchitecture {
 public:
  ControlArchitecture(RobotSystem* _robot) {
    robot_ = _robot;
  }

  virtual ~ControlArchitecture() {}

  virtual void ControlArchitectureInitialization() = 0;
  virtual void getCommand(void* _command) = 0;  
  virtual void addState(void* _user_state_command) = 0;
  
  int getState() { return state_; }
  int getPrevState() { return prev_state_; }
  RobotSystem* robot_;

  std::map<StateIdentifier, StateMachine*> state_machines_;

 protected:
  int state_;
  int prev_state_;
};
