#pragma once

#include <vector>

#include <pnc_utils/robot_system.hpp>


#include <pnc_utils/io_utilities.hpp>
#include <pnc_utils/math_utilities.hpp>


#include <pnc_core/wbc/Contact/ContactSpec.hpp>
#include <pnc_core/wbc/Task/task.hpp>

// Object to manage common trajectory primitives.
// Base class for the trajectory manager
// Will be used to updateTask

class TrajectoryManagerBase {
 public:
  TrajectoryManagerBase(RobotSystem* _robot) { 
    robot_ = _robot; 
    traj_start_time_ = 0.;
    traj_end_time_ = 0.;
    traj_duration_ = 0.;
  }
  virtual ~TrajectoryManagerBase() {}

  RobotSystem* robot_;

 protected:
  double traj_start_time_;
  double traj_end_time_;
  double traj_duration_;
};
