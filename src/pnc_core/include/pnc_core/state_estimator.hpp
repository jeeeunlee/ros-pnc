#pragma once

#include <pnc_utils/robot_system.hpp>
#include <pnc_core/wbc/Contact/ContactSpec.hpp>
#include <pnc_core/wbc/Task/task.hpp>
#include <pnc_utils/io_utilities.hpp>


class StateEstimator {
 public:
  StateEstimator(RobotSystem* _robot) {
    robot_ = _robot;
  }

  virtual ~StateEstimator() {}

  virtual void evaluate() = 0;
  virtual void initialization(const YAML::Node& node) = 0;

 protected:
  RobotSystem* robot_;              // Pointer to the robot
  
};